/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "CesiumTilesetNode"
#include "Context"
#include "CesiumIon"
#include "PrepareRenderResources"
#include "Settings"

#include <osgEarth/ElevationQuery>
#include <osgEarth/GeoCommon>
#include <osgEarth/MapNode>
#include <osgEarth/Notify>
#include <osgEarth/SpatialReference>
#include <osgUtil/CullVisitor>
#include <Cesium3DTilesSelection/BoundingVolume.h>

#include <osg/Geometry>
#include <osg/MatrixTransform>

#include <algorithm>
#include <cmath>
#include <iterator>

using namespace osgEarth::Cesium;

// ---- Result type (forward-declared in header) --------------------------------

namespace osgEarth { namespace Cesium
{
    struct ClampWorkResult
    {
        osg::ref_ptr<osg::Node> tileNode;
        unsigned generation = 0;
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
    // with clamped positions.  Never modifies the source geometry.
    class AsyncClampVisitor : public osg::NodeVisitor
    {
    public:
        AsyncClampVisitor(const osgEarth::Map* map)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
        {
            _wgs84 = osgEarth::SpatialReference::get("wgs84");
            _ecef = _wgs84 ? _wgs84->getGeocentricSRS() : nullptr;
            _eq.setMap(map);
        }

        bool valid() const { return _wgs84 && _ecef; }

        void apply(osg::MatrixTransform& mt) override
        {
            osg::Matrixd saved = _currentMatrix;
            _currentMatrix.preMult(mt.getMatrix());
            traverse(mt);
            _currentMatrix = saved;
        }

        void apply(osg::Drawable& drawable) override
        {
            auto* geom = drawable.asGeometry();
            if (!geom)
                return;
            auto* verts = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
            if (!verts || verts->empty())
                return;

            const osg::Matrixd& localToEcef = _currentMatrix;
            osg::Matrixd ecefToLocal;
            if (!ecefToLocal.invert(localToEcef))
                return;

            const std::size_t nv = verts->size();

            // Transform all vertices to WGS-84
            std::vector<osg::Vec3d> wgs84Pts;
            wgs84Pts.reserve(nv);
            for (std::size_t i = 0; i < nv; ++i)
            {
                osg::Vec3d ecef = osg::Vec3d((*verts)[i]) * localToEcef;
                osg::Vec3d geo;
                if (_ecef->transform(ecef, _wgs84, geo))
                    wgs84Pts.push_back(geo);
                else
                    wgs84Pts.emplace_back(0.0, 0.0, 0.0);
            }

            // Query terrain elevation in bounded chunks
            for (std::size_t off = 0; off < nv; off += kElevQueryChunkSize)
            {
                const std::size_t end = std::min(off + kElevQueryChunkSize, nv);
                std::vector<osg::Vec3d> chunk(
                    wgs84Pts.begin() + static_cast<std::ptrdiff_t>(off),
                    wgs84Pts.begin() + static_cast<std::ptrdiff_t>(end));
                _eq.getElevations(chunk, _wgs84, true, 0.0);
                for (std::size_t i = 0; i < chunk.size(); ++i)
                    wgs84Pts[off + i].z() = chunk[i].z();
            }

            // Build a NEW vertex array with clamped positions
            osg::ref_ptr<osg::Vec3Array> newVerts = new osg::Vec3Array(nv);
            for (std::size_t i = 0; i < nv; ++i)
            {
                const double ez = wgs84Pts[i].z();
                if (!std::isfinite(ez) || ez < static_cast<double>(NO_DATA_VALUE) + 1.0)
                {
                    (*newVerts)[i] = (*verts)[i];
                    continue;
                }
                osg::Vec3d ecefClamped;
                if (_wgs84->transform(wgs84Pts[i], _ecef, ecefClamped))
                {
                    osg::Vec3d local = ecefClamped * ecefToLocal;
                    (*newVerts)[i].set(
                        static_cast<float>(local.x()),
                        static_cast<float>(local.y()),
                        static_cast<float>(local.z()));
                }
                else
                {
                    (*newVerts)[i] = (*verts)[i];
                }
            }

            ClampWorkResult::GeomUpdate update;
            update.geom = geom;
            update.newVerts = newVerts;
            _updates.push_back(std::move(update));
        }

        std::vector<ClampWorkResult::GeomUpdate> _updates;

    private:
        osg::Matrixd _currentMatrix;
        const osgEarth::SpatialReference* _wgs84 = nullptr;
        const osgEarth::SpatialReference* _ecef = nullptr;
        osgEarth::ElevationQuery _eq;
    };

    class ClearClampFlagVisitor : public osg::NodeVisitor
    {
    public:
        ClearClampFlagVisitor()
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

        void apply(osg::Node& node) override
        {
            node.setUserValue("cpuClamped", false);
            traverse(node);
        }
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
        _clampThread.join();

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
        _clampThread = std::thread(&CesiumTilesetNode::clampWorkerLoop, this);
    }
}

void
CesiumTilesetNode::clampWorkerLoop()
{
    while (_clampRunning.load(std::memory_order_acquire))
    {
        osg::ref_ptr<osg::Node> tileNode;
        const osgEarth::Map* map = nullptr;
        unsigned gen = 0;
        {
            std::unique_lock<std::mutex> lock(_clampMutex);
            _clampCV.wait(lock, [this]() {
                return !_clampPending.empty() || !_clampRunning.load(std::memory_order_acquire);
            });
            if (!_clampRunning.load(std::memory_order_acquire))
                break;
            if (_clampPending.empty())
                continue;

            tileNode = _clampPending.front();
            _clampPending.pop_front();
            map = _workerMap;
            gen = _clampGeneration.load(std::memory_order_acquire);
        }

        if (!tileNode.valid() || !map)
        {
            // Cannot process -- remove from in-flight so it can be re-queued later.
            std::lock_guard<std::mutex> lock(_clampMutex);
            _clampInFlight.erase(tileNode.get());
            continue;
        }

        AsyncClampVisitor visitor(map);
        if (!visitor.valid())
        {
            std::lock_guard<std::mutex> lock(_clampMutex);
            _clampInFlight.erase(tileNode.get());
            continue;
        }

        // Read-only traversal: reads vertices + transforms, queries ElevationPool
        // (ElevationPool has internal locking and is safe to call from any thread).
        tileNode->accept(visitor);

        auto result = std::make_shared<ClampWorkResult>();
        result->tileNode = tileNode;
        result->generation = gen;
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

    const unsigned currentGen = _clampGeneration.load(std::memory_order_acquire);

    for (auto& result : results)
    {
        {
            std::lock_guard<std::mutex> lock(_clampMutex);
            _clampInFlight.erase(result->tileNode.get());
        }

        // Discard results computed against a stale terrain generation.
        if (result->generation < currentGen)
            continue;

        for (auto& update : result->updates)
        {
            osg::ref_ptr<osg::Geometry> geom;
            if (update.geom.lock(geom))
            {
                geom->setVertexArray(update.newVerts.get());
                geom->dirtyBound();
            }
        }
        result->tileNode->setUserValue("cpuClamped", true);
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
    _terrainCallback = new TerrainCallbackAdapter<CesiumTilesetNode>(this);
    mapNode->getTerrain()->addTerrainCallback(_terrainCallback.get());
    _terrainCallbackInstalled = true;
}

void
CesiumTilesetNode::onTileUpdate(const TileKey& /*key*/, osg::Node* /*tile*/, TerrainCallbackContext& /*context*/)
{
    _terrainDirty.store(true, std::memory_order_release);
}

// ---- Traversal ---------------------------------------------------------------

void
CesiumTilesetNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
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

        osg::Group* parent = tileParent();
        parent->removeChildren(0, parent->getNumChildren());

        bool needsClampQueue = false;

        for (auto tile : updates.tilesToRenderThisFrame)
        {
            if (tile->getContent().isRenderContent())
            {
                MainThreadResult* result = reinterpret_cast<MainThreadResult*>(tile->getContent().getRenderContent()->getRenderResources());
                if (result && result->node.valid())
                {
                    if (_clampToGround)
                    {
                        bool alreadyClamped = false;
                        result->node->getUserValue("cpuClamped", alreadyClamped);
                        if (!alreadyClamped)
                        {
                            std::lock_guard<std::mutex> lock(_clampMutex);
                            if (_clampInFlight.find(result->node.get()) == _clampInFlight.end())
                            {
                                _clampPending.push_back(result->node);
                                _clampInFlight.insert(result->node.get());
                                needsClampQueue = true;
                            }
                        }
                    }
                    parent->addChild(result->node.get());
                }
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

            if (_terrainDirty.exchange(false, std::memory_order_acq_rel))
                invalidateClampedTiles();

            applyClampResults();
        }
        osg::Group::traverse(nv);
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
    _clampGeneration.fetch_add(1, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(_clampMutex);
        _clampPending.clear();
        _clampInFlight.clear();
    }
    {
        std::lock_guard<std::mutex> lock(_resultMutex);
        _clampResults.clear();
    }
    ClearClampFlagVisitor visitor;
    accept(visitor);
}
