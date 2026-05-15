/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "CesiumTilesetNode"
#include "Context"
#include "CesiumIon"
#include "PrepareRenderResources"
#include "Settings"

#include <osgEarth/ElevationPool>
#include <osgEarth/GeoCommon>
#include <osgEarth/MapNode>
#include <osgEarth/Notify>
#include <osgEarth/Progress>
#include <osgEarth/SpatialReference>
#include <osgUtil/CullVisitor>
#include <Cesium3DTilesSelection/BoundingVolume.h>

#include <osg/Array>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/UserDataContainer>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>

using namespace osgEarth::Cesium;

// ---- Result type (forward-declared in header) --------------------------------

namespace osgEarth { namespace Cesium
{
    struct ClampWorkItem
    {
        osg::ref_ptr<osg::Node> tileNode;
        std::vector<ClampGeometrySource> sources;
    };

    struct ClampWorkResult
    {
        osg::ref_ptr<osg::Node> tileNode;
        /// Snapshot of CesiumTilesetNode::_clampGeneration when the job started; stale if terrain changed mid-job.
        unsigned clampGeneration = 0u;
        /// Terrain::getElevationRevision() after sampling (increments on each notifyMapElevationChanged).
        unsigned terrainElevationRevAfterJob = 0u;
        /// Diagnostic: Terrain::getConsumerVisibilitySignature() at worker start / end of accept(visitor).
        std::uint64_t terrainVisibilitySigBeforeJob = 0u;
        std::uint64_t terrainVisibilitySigAfterJob = 0u;
        std::vector<ClampGeometrySource> sources;
        struct GeomUpdate
        {
            osg::observer_ptr<osg::Geometry> geom;
            osg::ref_ptr<osg::Vec3Array> newVerts;
        };
        std::vector<GeomUpdate> updates;
    };
}}

// ---- Anonymous helpers -------------------------------------------------------

namespace
{
    constexpr const char* kClampElevRevKey = "clampElevRev";
    constexpr const char* kClampSourceVertsKey = "clampSourceVerts";

    inline void writeClampDiagnostic(const std::string& message)
    {
        static std::mutex s_fileMutex;
        std::lock_guard<std::mutex> lock(s_fileMutex);
        try
        {
            std::filesystem::create_directories("logs");
            std::ofstream out("logs/cesium-clamp-diagnostics.log", std::ios::app);
            out << message << std::endl;
        }
        catch (...)
        {
        }
    }

    inline void setClampElevRevUser(osg::Node* node, unsigned rev)
    {
        if (!node)
            return;
        node->setUserValue(kClampElevRevKey, std::to_string(rev));
    }

    inline bool getClampElevRevUser(const osg::Node* node, unsigned& out)
    {
        if (!node)
            return false;
        std::string s;
        if (!node->getUserValue(kClampElevRevKey, s) || s.empty())
            return false;
        try
        {
            out = static_cast<unsigned>(std::stoul(s));
        }
        catch (...)
        {
            return false;
        }
        return true;
    }

    inline void clearClampElevRevUser(osg::Node* node)
    {
        if (!node)
            return;
        node->setUserValue(kClampElevRevKey, std::string());
    }

    inline osg::Vec3Array* getOrCreateClampSourceVertices(osg::Geometry* geom, osg::Vec3Array* currentVerts)
    {
        if (!geom || !currentVerts)
            return nullptr;

        osg::UserDataContainer* udc = geom->getUserDataContainer();
        if (udc)
        {
            auto* sourceVerts = dynamic_cast<osg::Vec3Array*>(udc->getUserObject(kClampSourceVertsKey));
            if (sourceVerts)
            {
                if (sourceVerts->size() != currentVerts->size())
                    sourceVerts->assign(currentVerts->begin(), currentVerts->end());
                sourceVerts->setBinding(currentVerts->getBinding());
                sourceVerts->setNormalize(currentVerts->getNormalize());
                return sourceVerts;
            }
        }

        osg::ref_ptr<osg::Vec3Array> sourceVerts = new osg::Vec3Array(*currentVerts);
        sourceVerts->setName(kClampSourceVertsKey);
        sourceVerts->setBinding(currentVerts->getBinding());
        sourceVerts->setNormalize(currentVerts->getNormalize());
        geom->getOrCreateUserDataContainer()->addUserObject(sourceVerts.get());
        return sourceVerts.get();
    }

    double estimateViewZoomLevel(
        const osg::Vec3d& eye,
        double vfovRad,
        double viewportHeightPx)
    {
        constexpr double kEarthRadiusM = 6378137.0;
        constexpr double kTilePixels = 256.0;
        constexpr double kEarthCircumferenceM = 2.0 * 3.14159265358979323846 * kEarthRadiusM;
        const double eyeRadius = eye.length();
        const double height = std::max(eyeRadius - kEarthRadiusM, 1.0);
        const double viewHeightMeters = 2.0 * height * std::tan(vfovRad * 0.5);
        const double mpp = viewHeightMeters / std::max(viewportHeightPx, 1.0);
        return std::log2(kEarthCircumferenceM / (mpp * kTilePixels));
    }

    constexpr std::size_t kElevQueryChunkSize = 2048u;

    // Read-only visitor: traverses a tile sub-graph, reads vertices + transforms,
    // queries terrain elevation on the calling thread, and produces NEW Vec3Arrays
    // with clamped positions. Never modifies the source geometry.
    class AsyncClampVisitor : public osg::NodeVisitor
    {
    public:
        AsyncClampVisitor(osgEarth::ElevationPool* elevationPool, const std::atomic<bool>& stopFlag)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
            , _stopFlag(stopFlag)
        {
            writeClampDiagnostic("[AsyncClampVisitor] ctor begin");
            _pool = elevationPool;
            _mapSRS = _pool ? _pool->getMapSRS() : nullptr;
            _ecef = _mapSRS ? _mapSRS->getGeocentricSRS() : nullptr;
            if (!valid())
            {
                OE_WARN << "[AsyncClampVisitor] invalid elevation pool or SRS; clamp visitor will produce no updates"
                        << std::endl;
            }
            writeClampDiagnostic(valid() ? "[AsyncClampVisitor] ctor valid" : "[AsyncClampVisitor] ctor invalid");
        }

        ~AsyncClampVisitor()
        {
            if (_drawablesSeen == 0u)
                return;
            std::ostringstream diag;
            diag << "[AsyncClampVisitor] drawables=" << _drawablesSeen
                 << "  geomsProduced=" << _geomsProduced
                 << "  vertsTotal=" << _vertsInUpdatedGeoms
                 << "  skipNotGeometry=" << _skipNotGeometry
                 << "  skipBadVertexArray=" << _skipBadVertexArray
                 << "  skipNonVec3Positions=" << _skipNonVec3Positions
                 << "  skipInvertLocalToEcef=" << _skipInvertLocalToEcef
                 << "  skipStoppedMidClamp=" << _skipStoppedMidClamp
                 << "  elevQueryCalls=" << _elevQueryCalls
                 << "  elevQueryFailures=" << _elevQueryFailures
                 << "  elevValid=" << _elevValid
                 << "  elevRange=" << _elevMin << ".." << _elevMax
                 << "  sourceLocalZ=" << _sourceLocalZMin << ".." << _sourceLocalZMax
                 << "  clampedLocalZ=" << _clampedLocalZMin << ".." << _clampedLocalZMax
                 << "  vertsUnchangedNoElev=" << _vertsUnchangedNoElev;
            writeClampDiagnostic(diag.str());
            OE_WARN << diag.str() << std::endl;
            if (_drawablesSeen > 0u && _geomsProduced == 0u)
            {
                OE_WARN << "[AsyncClampVisitor] produced no clamp updates (see skip* counters; common: "
                           "POSITION not osg::Vec3Array/osg::Vec3dArray, empty mesh placeholder, or clamp worker "
                           "stopped mid-elevation query)."
                        << std::endl;
            }
        }

        bool valid() const { return _pool && _mapSRS && _ecef; }
        bool stopped() const { return !_stopFlag.load(std::memory_order_relaxed); }

        void apply(osg::MatrixTransform& mt) override
        {
            if (stopped()) return;
            osg::Matrixd saved = _currentMatrix;
            _currentMatrix.preMult(mt.getMatrix());
            traverse(mt);
            _currentMatrix = saved;
        }

        void apply(osg::Drawable& drawable) override
        {
            if (stopped()) return;

            ++_drawablesSeen;
            if (_drawablesSeen <= 4u)
            {
                std::ostringstream diag;
                diag << "[AsyncClampVisitor] drawable begin count=" << _drawablesSeen
                     << " drawable=" << &drawable;
                writeClampDiagnostic(diag.str());
            }
            auto* geom = drawable.asGeometry();
            if (!geom)
            {
                ++_skipNotGeometry;
                return;
            }

            osg::ref_ptr<osg::Vec3Array> convertedPositions;
            osg::Array* const va = geom->getVertexArray();
            osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(va);
            if (!verts && va)
            {
                if (auto* vd = dynamic_cast<osg::Vec3dArray*>(va))
                {
                    convertedPositions = new osg::Vec3Array(static_cast<unsigned>(vd->size()));
                    for (unsigned i = 0; i < vd->size(); ++i)
                    {
                        const osg::Vec3d& p = (*vd)[i];
                        (*convertedPositions)[i].set(
                            static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z()));
                    }
                    verts = convertedPositions.get();
                }
            }
            if (!verts || verts->empty())
            {
                if (va && !verts)
                    ++_skipNonVec3Positions;
                else
                    ++_skipBadVertexArray;
                return;
            }

            osg::Vec3Array* sourceVerts = getOrCreateClampSourceVertices(geom, verts);
            if (!sourceVerts || sourceVerts->empty())
            {
                ++_skipBadVertexArray;
                return;
            }

            clampGeometry(geom, sourceVerts, _currentMatrix);
        }

        void clampGeometry(osg::Geometry* geom, osg::Vec3Array* sourceVerts, const osg::Matrixd& localToEcef)
        {
            if (!sourceVerts)
            {
                static const std::vector<osg::Vec3> empty;
                clampGeometry(geom, empty, osg::Array::BIND_UNDEFINED, false, localToEcef);
                return;
            }
            const std::vector<osg::Vec3> snapshot(sourceVerts->begin(), sourceVerts->end());
            clampGeometry(geom, snapshot, sourceVerts->getBinding(), sourceVerts->getNormalize(), localToEcef);
        }

        void clampGeometry(
            osg::Geometry* geom,
            const std::vector<osg::Vec3>& sourceVerts,
            osg::Array::Binding binding,
            bool normalize,
            const osg::Matrixd& localToEcef)
        {
            if (stopped()) return;
            if (!geom || sourceVerts.empty())
            {
                ++_skipBadVertexArray;
                return;
            }

            {
                std::ostringstream diag;
                diag << "[AsyncClampVisitor] clampGeometry begin geom=" << geom
                     << " verts=" << sourceVerts.size();
                writeClampDiagnostic(diag.str());
            }

            osg::Matrixd ecefToLocal;
            if (!ecefToLocal.invert(localToEcef))
            {
                ++_skipInvertLocalToEcef;
                return;
            }

            const std::size_t nv = sourceVerts.size();

            for (std::size_t i = 0; i < nv; ++i)
            {
                const double z = static_cast<double>(sourceVerts[i].z());
                _sourceLocalZMin = std::min(_sourceLocalZMin, z);
                _sourceLocalZMax = std::max(_sourceLocalZMax, z);
            }

            writeClampDiagnostic("[AsyncClampVisitor] source z scan done");

            // Transform original vertices to map coordinates so reclamping never samples already-clamped positions.
            std::vector<osg::Vec3d> mapPts(nv, osg::Vec3d(0.0, 0.0, NO_DATA_VALUE));

            // Sample only currently cached terrain in bounded chunks; bail early on stop.
            std::size_t validElevForGeom = 0u;
            std::size_t sampledElevForGeom = 0u;
            unsigned zeroSampleChunksAtStart = 0u;
            for (std::size_t off = 0; off < nv; off += kElevQueryChunkSize)
            {
                if (stopped())
                {
                    ++_skipStoppedMidClamp;
                    return;
                }
                const std::size_t end = std::min(off + kElevQueryChunkSize, nv);
                std::vector<osg::Vec3d> chunk;
                chunk.reserve(end - off);
                if (off > 0u && (off % 100000u) < kElevQueryChunkSize)
                {
                    std::ostringstream diag;
                    diag << "[AsyncClampVisitor] ecef->map progress " << off << "/" << nv;
                    writeClampDiagnostic(diag.str());
                }
                for (std::size_t i = off; i < end; ++i)
                {
                    osg::Vec3d ecef = osg::Vec3d(sourceVerts[i]) * localToEcef;
                    osg::Vec3d mapPoint;
                    if (_ecef->transform(ecef, _mapSRS, mapPoint))
                        chunk.push_back(mapPoint);
                    else
                        chunk.emplace_back(0.0, 0.0, NO_DATA_VALUE);
                }
                ++_elevQueryCalls;
                if (_elevQueryCalls <= 4u)
                {
                    std::ostringstream diag;
                    diag << "[AsyncClampVisitor] cache sample begin call=" << _elevQueryCalls
                         << " off=" << off
                         << " count=" << chunk.size();
                    writeClampDiagnostic(diag.str());
                }
                const int sampled = _pool->sampleMapCoordsFromCache(
                    chunk.begin(), chunk.end(), nullptr, NO_DATA_VALUE);
                if (sampled < 0)
                    ++_elevQueryFailures;
                else
                    sampledElevForGeom += static_cast<std::size_t>(sampled);
                if (sampled == 0 && sampledElevForGeom == 0u)
                    ++zeroSampleChunksAtStart;
                if (_elevQueryCalls <= 4u || (_elevQueryCalls % 16u) == 0u)
                {
                    std::ostringstream diag;
                    diag << "[AsyncClampVisitor] cache sample end call=" << _elevQueryCalls
                         << " off=" << off
                         << " sampled=" << sampled
                         << " sampledTotal=" << sampledElevForGeom
                         << " validTotal=" << validElevForGeom
                         << " failures=" << _elevQueryFailures;
                    writeClampDiagnostic(diag.str());
                }
                for (std::size_t i = 0; i < chunk.size(); ++i)
                {
                    mapPts[off + i] = chunk[i];
                    const double elevation = chunk[i].z();
                    if (std::isfinite(elevation) && elevation != static_cast<double>(NO_DATA_VALUE))
                    {
                        _elevMin = std::min(_elevMin, elevation);
                        _elevMax = std::max(_elevMax, elevation);
                        ++_elevValid;
                        ++validElevForGeom;
                    }
                }
                if (zeroSampleChunksAtStart >= 16u)
                {
                    std::ostringstream diag;
                    diag << "[AsyncClampVisitor] cache miss break geom=" << geom
                         << " checkedVerts=" << (off + chunk.size())
                         << " totalVerts=" << nv
                         << " sampledTotal=" << sampledElevForGeom
                         << " validTotal=" << validElevForGeom;
                    writeClampDiagnostic(diag.str());
                    break;
                }
            }

            if (stopped())
            {
                ++_skipStoppedMidClamp;
                return;
            }

            if (sampledElevForGeom == 0u)
            {
                _vertsUnchangedNoElev += nv;
                std::ostringstream diag;
                diag << "[AsyncClampVisitor] no cached elevation geom=" << geom
                     << " verts=" << nv;
                writeClampDiagnostic(diag.str());
                return;
            }

            // Build a NEW vertex array with clamped positions.
            osg::ref_ptr<osg::Vec3Array> newVerts = new osg::Vec3Array(nv);
            newVerts->setBinding(binding);
            newVerts->setNormalize(normalize);
            newVerts->setDataVariance(osg::Object::DYNAMIC);
            std::vector<osg::Vec3> fallbackVerts;
            if (auto* currentVerts = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray()))
            {
                if (currentVerts->size() == nv)
                    fallbackVerts.assign(currentVerts->begin(), currentVerts->end());
            }
            if (fallbackVerts.size() != nv)
                fallbackVerts = sourceVerts;
            std::size_t unchangedNoElev = 0u;
            for (std::size_t i = 0; i < nv; ++i)
            {
                const double ez = mapPts[i].z();
                if (!std::isfinite(ez) || ez == static_cast<double>(NO_DATA_VALUE))
                {
                    (*newVerts)[i] = fallbackVerts[i];
                    ++unchangedNoElev;
                    continue;
                }
                osg::Vec3d ecefClamped;
                if (_mapSRS->transform(mapPts[i], _ecef, ecefClamped))
                {
                    osg::Vec3d local = ecefClamped * ecefToLocal;
                    (*newVerts)[i].set(
                        static_cast<float>(local.x()),
                        static_cast<float>(local.y()),
                        static_cast<float>(local.z()));
                    _clampedLocalZMin = std::min(_clampedLocalZMin, local.z());
                    _clampedLocalZMax = std::max(_clampedLocalZMax, local.z());
                }
                else
                {
                    (*newVerts)[i] = fallbackVerts[i];
                    ++unchangedNoElev;
                }
            }
            _vertsUnchangedNoElev += unchangedNoElev;

            ClampWorkResult::GeomUpdate update;
            update.geom = geom;
            update.newVerts = newVerts;
            _updates.push_back(std::move(update));
            ++_geomsProduced;
            _vertsInUpdatedGeoms += nv;
        }

        std::vector<ClampWorkResult::GeomUpdate> _updates;

    private:
        unsigned _geomsProduced = 0u;
        unsigned _drawablesSeen = 0u;
        unsigned _skipNotGeometry = 0u;
        unsigned _skipBadVertexArray = 0u;
        unsigned _skipNonVec3Positions = 0u;
        unsigned _skipInvertLocalToEcef = 0u;
        unsigned _skipStoppedMidClamp = 0u;
        unsigned _elevQueryCalls = 0u;
        unsigned _elevQueryFailures = 0u;
        std::size_t _elevValid = 0u;
        std::size_t _vertsInUpdatedGeoms = 0u;
        std::size_t _vertsUnchangedNoElev = 0u;
        double _elevMin = std::numeric_limits<double>::max();
        double _elevMax = -std::numeric_limits<double>::max();
        double _sourceLocalZMin = std::numeric_limits<double>::max();
        double _sourceLocalZMax = -std::numeric_limits<double>::max();
        double _clampedLocalZMin = std::numeric_limits<double>::max();
        double _clampedLocalZMax = -std::numeric_limits<double>::max();

        osg::Matrixd _currentMatrix;
        osgEarth::ElevationPool* _pool = nullptr;
        const osgEarth::SpatialReference* _mapSRS = nullptr;
        const osgEarth::SpatialReference* _ecef = nullptr;
        const std::atomic<bool>& _stopFlag;
    };

}

// ---- Construction / destruction ----------------------------------------------

CesiumTilesetNode::CesiumTilesetNode(unsigned int assetID, const std::string& server, const std::string& token, float maximumScreenSpaceError, std::vector<int> overlays, const TilesetRenderStyleOptions& renderStyle, int minimumRenderableLevel)
{ 
    _minimumRenderableLevel = minimumRenderableLevel;
    _clampToGround = renderStyle.clampToGround;
    Context* context = CesiumIon::instance().getContext(server);

    Cesium3DTilesSelection::TilesetExternals externals{
        context->assetAccessor, context->prepareRenderResources, context->asyncSystem, context->creditSystem, context->logger, nullptr
    };

    Cesium3DTilesSelection::TilesetOptions options;    
    options.maximumScreenSpaceError = maximumScreenSpaceError;
    options.contentOptions.generateMissingNormalsSmooth = true;
    options.rendererOptions = renderStyle;
    Cesium3DTilesSelection::Tileset* tileset = new Cesium3DTilesSelection::Tileset(externals, assetID, token, options, server);

    for (auto overlay: overlays)
    {
        CesiumRasterOverlays::RasterOverlayOptions rasterOptions;
        const auto ionRasterOverlay = new CesiumRasterOverlays::IonRasterOverlay("", overlay, token, rasterOptions);
        tileset->getOverlays().add(ionRasterOverlay);
    }    
    _tileset = tileset;

    setCullingActive(false);    
}

CesiumTilesetNode::CesiumTilesetNode(const std::string& url, const std::string& server, const std::string& token, float maximumScreenSpaceError, std::vector<int> overlays, const TilesetRenderStyleOptions& renderStyle, int minimumRenderableLevel)
{
    _minimumRenderableLevel = minimumRenderableLevel;
    _clampToGround = renderStyle.clampToGround;
    Context* context = CesiumIon::instance().getContext(server);

    Cesium3DTilesSelection::TilesetExternals externals{
        context->assetAccessor, context->prepareRenderResources, context->asyncSystem, context->creditSystem, context->logger, nullptr
    };

    Cesium3DTilesSelection::TilesetOptions options;
    options.maximumScreenSpaceError = maximumScreenSpaceError;
    options.contentOptions.generateMissingNormalsSmooth = true;
    options.rendererOptions = renderStyle;
    Cesium3DTilesSelection::Tileset* tileset = new Cesium3DTilesSelection::Tileset(externals, url, options);
    for (auto overlay : overlays)
    {
        CesiumRasterOverlays::RasterOverlayOptions rasterOptions;
        const auto ionRasterOverlay = new CesiumRasterOverlays::IonRasterOverlay("", overlay, token, rasterOptions, server);
        tileset->getOverlays().add(ionRasterOverlay);
    }
    _tileset = tileset;

    setCullingActive(false);
}


CesiumTilesetNode::~CesiumTilesetNode()
{
    _clampRunning.store(false, std::memory_order_release);
    _clampCV.notify_all();
    if (_clampThread.joinable())
    {
        auto joinFuture = std::async(std::launch::async, [this]() { _clampThread.join(); });
        if (joinFuture.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
        {
            OE_WARN << "[CesiumClamp] worker join timed out - detaching" << std::endl;
            _clampThread.detach();
        }
    }

    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    delete tileset;
}

// ---- Property accessors ------------------------------------------------------

float CesiumTilesetNode::getMaximumScreenSpaceError() const
{
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    return tileset->getOptions().maximumScreenSpaceError;
}

void CesiumTilesetNode::setMaximumScreenSpaceError(float maximumScreenSpaceError)
{
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    tileset->getOptions().maximumScreenSpaceError = maximumScreenSpaceError;
}

bool CesiumTilesetNode::getForbidHoles() const
{
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    return tileset->getOptions().forbidHoles;
}

void CesiumTilesetNode::setForbidHoles(bool forbidHoles)
{
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    tileset->getOptions().forbidHoles = forbidHoles;
}

int CesiumTilesetNode::getMinimumRenderableLevel() const
{
    return _minimumRenderableLevel;
}

void CesiumTilesetNode::setMinimumRenderableLevel(int level)
{
    _minimumRenderableLevel = level;
}

osg::Group*
CesiumTilesetNode::tileParent()
{
    return this;
}

// ---- Async clamp infrastructure ----------------------------------------------

void
CesiumTilesetNode::ensureClampThread()
{
    bool expected = false;
    if (_clampRunning.compare_exchange_strong(expected, true))
    {
        writeClampDiagnostic("[CesiumClamp] worker thread start");
        _clampThread = std::thread(&CesiumTilesetNode::clampWorkerLoop, this);
    }
}

void
CesiumTilesetNode::clampWorkerLoop()
{
    while (_clampRunning.load(std::memory_order_acquire))
    {
        std::shared_ptr<ClampWorkItem> workItem;
        const osgEarth::Map* map = nullptr;
        {
            std::unique_lock<std::mutex> lock(_clampMutex);
            _clampCV.wait(lock, [this]() {
                return !_clampPending.empty() || !_clampRunning.load(std::memory_order_acquire);
            });
            if (!_clampRunning.load(std::memory_order_acquire))
                break;
            if (_clampPending.empty())
                continue;

            workItem = _clampPending.front();
            _clampPending.pop_front();
            map = _workerMap;
        }

        osg::Node* tileNode = workItem ? workItem->tileNode.get() : nullptr;
        if (!tileNode || !map)
        {
            std::ostringstream diag;
            diag << "[CesiumClamp] worker skip tile=" << tileNode
                 << " map=" << map;
            writeClampDiagnostic(diag.str());
            std::lock_guard<std::mutex> lock(_clampMutex);
            _clampInFlight.erase(tileNode);
            continue;
        }

        if (!_clampRunning.load(std::memory_order_acquire))
            break;

        const unsigned clampGenSnapshot = _clampGeneration.load(std::memory_order_acquire);

        writeClampDiagnostic("[CesiumClamp] worker create visitor");
        osgEarth::ElevationPool* elevationPool = map ? map->getElevationPool() : nullptr;
        AsyncClampVisitor visitor(elevationPool, _clampRunning);
        if (!visitor.valid())
        {
            writeClampDiagnostic("[CesiumClamp] worker skip invalid visitor");
            std::lock_guard<std::mutex> lock(_clampMutex);
            _clampInFlight.erase(tileNode);
            continue;
        }

        std::uint64_t sigBefore = 0u;
        if (_cachedTerrain.valid())
            sigBefore = _cachedTerrain->getConsumerVisibilitySignature();

        {
            std::ostringstream diag;
            diag << "[CesiumClamp] worker clamp begin tile=" << tileNode
                 << " sources=" << (workItem ? workItem->sources.size() : 0u);
            writeClampDiagnostic(diag.str());
        }
        if (workItem)
        {
            for (std::size_t sourceIndex = 0; sourceIndex < workItem->sources.size(); ++sourceIndex)
            {
                const ClampGeometrySource& source = workItem->sources[sourceIndex];
                {
                    std::ostringstream diag;
                    diag << "[CesiumClamp] worker source dispatch index=" << sourceIndex
                         << " geom=" << source.geom.get()
                         << " verts=" << source.sourceVerts.size();
                    writeClampDiagnostic(diag.str());
                }
                visitor.clampGeometry(
                    source.geom.get(),
                    source.sourceVerts,
                    source.binding,
                    source.normalize,
                    source.localToEcef);
                {
                    std::ostringstream diag;
                    diag << "[CesiumClamp] worker source done index=" << sourceIndex;
                    writeClampDiagnostic(diag.str());
                }
                if (visitor.stopped())
                    break;
            }
        }
        {
            std::ostringstream diag;
            diag << "[CesiumClamp] worker clamp end tile=" << tileNode;
            writeClampDiagnostic(diag.str());
        }

        std::uint64_t sigAfter = 0u;
        unsigned elevRevAfter = 0u;
        if (_cachedTerrain.valid())
        {
            sigAfter = _cachedTerrain->getConsumerVisibilitySignature();
            elevRevAfter = _cachedTerrain->getElevationRevision();
        }

        if (visitor.stopped())
        {
            std::lock_guard<std::mutex> lock(_clampMutex);
            _clampInFlight.erase(tileNode);
            break;
        }

        auto result = std::make_shared<ClampWorkResult>();
        result->tileNode = tileNode;
        result->clampGeneration = clampGenSnapshot;
        result->terrainVisibilitySigBeforeJob = sigBefore;
        result->terrainVisibilitySigAfterJob = sigAfter;
        result->terrainElevationRevAfterJob = elevRevAfter;
        result->sources = workItem ? workItem->sources : std::vector<ClampGeometrySource>();
        result->updates = std::move(visitor._updates);

        {
            std::lock_guard<std::mutex> lock(_resultMutex);
            _clampResults.push_back(std::move(result));
        }
    }
}

void
CesiumTilesetNode::applyClampResults()
{
    std::deque<std::shared_ptr<ClampWorkResult>> results;
    {
        std::lock_guard<std::mutex> lock(_resultMutex);
        results.swap(_clampResults);
    }

    if (results.empty())
        return;

    const unsigned currentElevRev =
        _cachedTerrain.valid() ? _cachedTerrain->getElevationRevision() : 0u;
    unsigned applied = 0, skippedGeom = 0;
    std::unordered_set<osg::Node*> requeueTiles;
    std::vector<std::shared_ptr<ClampWorkItem>> requeueItems;

    for (auto& result : results)
    {
        const unsigned nowGen = _clampGeneration.load(std::memory_order_acquire);
        const bool generationMismatch = result->clampGeneration != nowGen;

        {
            std::lock_guard<std::mutex> lock(_clampMutex);
            _clampInFlight.erase(result->tileNode.get());
        }

        unsigned geomCount = 0;
        for (auto& update : result->updates)
        {
            osg::ref_ptr<osg::Geometry> geom;
            if (update.geom.lock(geom))
            {
                geom->setVertexArray(update.newVerts.get());
                geom->dirtyBound();
                geom->dirtyGLObjects();
                ++geomCount;
            }
            else
            {
                ++skippedGeom;
            }
        }

        const unsigned nowElevRev =
            _cachedTerrain.valid() ? _cachedTerrain->getElevationRevision() : 0u;

        // Commit vertex work. If elevation revision moved again before apply, re-queue for another sample.
        if (!result->updates.empty() && geomCount > 0u)
        {
            setClampElevRevUser(result->tileNode.get(), result->terrainElevationRevAfterJob);
            ++applied;
            {
                std::ostringstream diag;
                diag << "[CesiumClamp] APPLIED tile=" << result->tileNode.get()
                     << " elevRevSample=" << result->terrainElevationRevAfterJob
                     << " elevRevNow=" << nowElevRev
                     << " sigSample=0x" << std::hex << result->terrainVisibilitySigAfterJob
                     << " sigNow=0x" << (_cachedTerrain.valid() ? _cachedTerrain->getConsumerVisibilitySignature() : 0u)
                     << std::dec
                     << " geoms=" << geomCount
                     << " updates=" << result->updates.size();
                writeClampDiagnostic(diag.str());
            }
            OE_NOTICE << "[CesiumClamp] APPLIED  tile=" << result->tileNode.get()
                      << "  elevRevSample=" << result->terrainElevationRevAfterJob
                      << "  elevRevNow=" << nowElevRev
                      << "  sigSample=0x" << std::hex << result->terrainVisibilitySigAfterJob
                      << "  sigNow=0x" << (_cachedTerrain.valid() ? _cachedTerrain->getConsumerVisibilitySignature() : 0u)
                      << std::dec
                      << "  geoms=" << geomCount
                      << "  updates=" << result->updates.size() << std::endl;
            if ((generationMismatch || result->terrainElevationRevAfterJob != nowElevRev) && result->tileNode.valid())
            {
                requeueTiles.insert(result->tileNode.get());
                auto item = std::make_shared<ClampWorkItem>();
                item->tileNode = result->tileNode;
                item->sources = result->sources;
                requeueItems.push_back(std::move(item));
            }
        }
        else
        {
            if (result->tileNode.valid())
            {
                std::ostringstream diag;
                diag << "[CesiumClamp] NO_CACHED_ELEV tile=" << result->tileNode.get()
                     << " elevRevSample=" << result->terrainElevationRevAfterJob
                     << " elevRevNow=" << nowElevRev
                     << " sources=" << result->sources.size();
                writeClampDiagnostic(diag.str());

                if (generationMismatch)
                {
                    requeueTiles.insert(result->tileNode.get());
                    auto item = std::make_shared<ClampWorkItem>();
                    item->tileNode = result->tileNode;
                    item->sources = result->sources;
                    requeueItems.push_back(std::move(item));
                }
            }
        }
    }

    if (!requeueTiles.empty())
    {
        bool wake = false;
        {
            std::lock_guard<std::mutex> lock(_clampMutex);
            for (const auto& item : requeueItems)
            {
                osg::Node* n = item ? item->tileNode.get() : nullptr;
                if (!n || _clampInFlight.count(n) != 0u)
                    continue;
                _clampPending.push_back(item);
                _clampInFlight.insert(n);
                wake = true;
            }
        }
        if (wake)
        {
            ensureClampThread();
            _clampCV.notify_one();
        }
    }

    if (applied > 0 || skippedGeom > 0 || !requeueTiles.empty())
    {
        {
            std::ostringstream diag;
            diag << "[CesiumClamp] applyClampResults applied=" << applied
                 << " skippedGeom=" << skippedGeom
                 << " requeuedTiles=" << requeueTiles.size()
                 << " elevRevNow=" << currentElevRev
                 << " totalResults=" << results.size();
            writeClampDiagnostic(diag.str());
        }
        OE_NOTICE << "[CesiumClamp] applyClampResults: applied=" << applied
                  << "  skippedGeom=" << skippedGeom
                  << "  requeuedTiles=" << requeueTiles.size()
                  << "  elevRevNow=" << currentElevRev
                  << "  totalResults=" << results.size() << std::endl;
    }
}

// ---- Terrain change callback -------------------------------------------------

void
CesiumTilesetNode::installTerrainCallback()
{
    if (_terrainCallbackInstalled)
        return;
    auto* mapNode = osgEarth::MapNode::findMapNode(this);
    if (!mapNode || !mapNode->getTerrain())
        return;
    _cachedTerrain = mapNode->getTerrain();
    _terrainCallback = new TerrainCallbackAdapter<CesiumTilesetNode>(this);
    _cachedTerrain->addTerrainCallback(_terrainCallback.get());
    _terrainCallbackInstalled = true;
}

void
CesiumTilesetNode::onTileUpdate(const TileKey& /*key*/, osg::Node* /*tile*/, TerrainCallbackContext& /*context*/)
{
    // Terrain/map elevation changed. Use a generation counter so updates arriving during traversal
    // are not collapsed into a single bool and can schedule one more invalidation pass.
    if (_clampToGround)
        _terrainClampGeneration.fetch_add(1u, std::memory_order_acq_rel);
}

// ---- Traversal ---------------------------------------------------------------

void
CesiumTilesetNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        unsigned terrainClampGenAtStart = 0u;
        if (_clampToGround)
        {
            installTerrainCallback();
            terrainClampGenAtStart = _terrainClampGeneration.load(std::memory_order_acquire);
            if (_processedTerrainClampGeneration != terrainClampGenAtStart)
            {
                invalidateClampedTiles();
                _processedTerrainClampGeneration = terrainClampGenAtStart;
            }
        }

        const osgUtil::CullVisitor* cv = nv.asCullVisitor();
        osg::Vec3d osgEye, osgCenter, osgUp;
        cv->getModelViewMatrix()->getLookAt(osgEye, osgCenter, osgUp);
        osg::Vec3d osgDir = osgCenter - osgEye;
        osgDir.normalize();

        glm::dvec3 pos(osgEye.x(), osgEye.y(), osgEye.z());
        glm::dvec3 dir(osgDir.x(), osgDir.y(), osgDir.z());
        glm::dvec3 up(osgUp.x(), osgUp.y(), osgUp.z());
        glm::dvec2 viewportSize(cv->getViewport()->width(), cv->getViewport()->height());

        double vfov, ar, znear, zfar;
        bool isPerspective = cv->getProjectionMatrix()->getPerspective(vfov, ar, znear, zfar);
        if (!isPerspective)
        {
            double left, right, bottom, top;
            cv->getProjectionMatrix()->getOrtho(left, right, bottom, top, znear, zfar);
            double orthoH = top - bottom;
            double orthoW = right - left;
            ar = (orthoH != 0.0) ? (orthoW / orthoH) : 1.0;
            double dist = std::max(osgEye.length() - 6378137.0, 1.0);
            vfov = 2.0 * osg::RadiansToDegrees(atan2(orthoH * 0.5, dist));
        }
        vfov = osg::DegreesToRadians(vfov);
        double hfov = 2 * atan(tan(vfov / 2) * (ar));

        if (_minimumRenderableLevel >= 0)
        {
            const double viewZoom = estimateViewZoomLevel(osgEye, vfov, cv->getViewport()->height());
            if (viewZoom < static_cast<double>(_minimumRenderableLevel))
            {
                if (_clampToGround)
                {
                    applyClampResults();
                }
                removeChildren(0, getNumChildren());
                osg::Group::traverse(nv);
                return;
            }
        }

        // TODO:  Multiple views
        std::vector<Cesium3DTilesSelection::ViewState> viewStates;
        Cesium3DTilesSelection::ViewState viewState(pos, dir, up, viewportSize, hfov, vfov);
        viewStates.push_back(viewState);
        Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
        auto updates = tileset->updateView(viewStates);

        if (_clampToGround)
            applyClampResults();

        osg::Group* parent = tileParent();

        bool needsClampQueue = false;
        bool hasDeferredUnclampedTile = false;
        std::vector<osg::ref_ptr<osg::Node>> displayNodes;
        std::unordered_set<osg::Node*> displayNodeSet;

        for (auto tile : updates.tilesToRenderThisFrame)
        {
            if (tile->getContent().isRenderContent())
            {
                MainThreadResult* result = reinterpret_cast<MainThreadResult*>(tile->getContent().getRenderContent()->getRenderResources());
                if (result && result->node.valid())
                {
                    if (_clampToGround)
                    {
                        const unsigned elevRev =
                            _cachedTerrain.valid() ? _cachedTerrain->getElevationRevision() : 0u;
                        unsigned appliedRev = 0u;
                        const bool hasRev = getClampElevRevUser(result->node.get(), appliedRev);
                        const bool needsClamp = !hasRev || appliedRev != elevRev;
                        if (needsClamp)
                        {
                            if (!result->clampGeometries.empty())
                            {
                                std::lock_guard<std::mutex> lock(_clampMutex);
                                if (_clampInFlight.count(result->node.get()) == 0u)
                                {
                                    auto item = std::make_shared<ClampWorkItem>();
                                    item->tileNode = result->node;
                                    item->sources = result->clampGeometries;
                                    _clampPending.push_back(std::move(item));
                                    _clampInFlight.insert(result->node.get());
                                    needsClampQueue = true;
                                }
                            }
                        }

                        if (!hasRev)
                        {
                            hasDeferredUnclampedTile = true;
                            continue;
                        }
                    }
                    displayNodes.push_back(result->node.get());
                    displayNodeSet.insert(result->node.get());
                }
            }
        }

        if (_clampToGround && hasDeferredUnclampedTile)
        {
            const unsigned previousCount = parent->getNumChildren();
            for (unsigned i = 0u; i < previousCount; ++i)
            {
                osg::Node* previous = parent->getChild(i);
                unsigned previousRev = 0u;
                if (previous && getClampElevRevUser(previous, previousRev) && displayNodeSet.insert(previous).second)
                    displayNodes.push_back(previous);
            }
        }

        parent->removeChildren(0, parent->getNumChildren());
        for (const auto& node : displayNodes)
        {
            if (node.valid())
                parent->addChild(node.get());
        }

        if (_clampToGround)
        {
            applyClampResults();
            const unsigned terrainClampGenAtEnd = _terrainClampGeneration.load(std::memory_order_acquire);
            if (_processedTerrainClampGeneration != terrainClampGenAtEnd)
            {
                invalidateClampedTiles();
                _processedTerrainClampGeneration = terrainClampGenAtEnd;
            }
        }

        if (needsClampQueue)
        {
            // Lazily resolve the Map* for the worker thread (main-thread only).
            {
                std::lock_guard<std::mutex> lock(_clampMutex);
                if (!_workerMap)
                {
                    auto* mapNode = osgEarth::MapNode::findMapNode(this);
                    if (mapNode)
                        _workerMap = mapNode->getMap();
                    std::ostringstream diag;
                    diag << "[CesiumClamp] resolve workerMap mapNode=" << mapNode
                         << " map=" << _workerMap;
                    writeClampDiagnostic(diag.str());
                }
            }
            ensureClampThread();
            _clampCV.notify_one();
        }
    }
    else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        if (_clampToGround)
        {
            installTerrainCallback();
            const unsigned terrainClampGenAtStart = _terrainClampGeneration.load(std::memory_order_acquire);
            if (_processedTerrainClampGeneration != terrainClampGenAtStart)
            {
                invalidateClampedTiles();
                _processedTerrainClampGeneration = terrainClampGenAtStart;
            }
            applyClampResults();
            const unsigned terrainClampGenAtEnd = _terrainClampGeneration.load(std::memory_order_acquire);
            if (_processedTerrainClampGeneration != terrainClampGenAtEnd)
            {
                invalidateClampedTiles();
                _processedTerrainClampGeneration = terrainClampGenAtEnd;
            }
        }
    }

    osg::Group::traverse(nv);
}

// ---- Bounds ------------------------------------------------------------------

osg::BoundingSphere CesiumTilesetNode::computeBound() const
{
    if (_tileset)
    {
        Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
        auto rootTile = tileset->getRootTile();
        if (rootTile)
        {
            auto bbox = Cesium3DTilesSelection::getOrientedBoundingBoxFromBoundingVolume(rootTile->getBoundingVolume());
            auto& center = bbox.getCenter();
            auto& lengths = bbox.getLengths();
            float radius = std::max(lengths.x, std::max(lengths.y, lengths.z));
            return osg::BoundingSphere(osg::Vec3(center.x, center.y, center.z), radius);
        }
    }
    return osg::BoundingSphere();
}

// ---- Invalidation ------------------------------------------------------------

void
CesiumTilesetNode::invalidateClampedTiles()
{
    _clampGeneration.fetch_add(1u, std::memory_order_acq_rel);
    // Keep the previous elevation revision stamp. Traversal compares it against the
    // current terrain revision, so stale clamped geometry stays visible while the
    // replacement clamp job is running.
}
