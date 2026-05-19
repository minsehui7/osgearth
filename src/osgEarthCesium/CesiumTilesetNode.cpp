/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "CesiumTilesetNode"
#include "Context"
#include "CesiumIon"
#include "PrepareRenderResources"
#include "Settings"

#include <osgEarth/GeoCommon>
#include <osgEarth/Notify>
#include <osgEarth/Progress>
#include <osgUtil/CullVisitor>
#include <Cesium3DTilesSelection/BoundingVolume.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

using namespace osgEarth::Cesium;

#define LC "[CesiumTilesetNode] "

// ---- Anonymous helpers -------------------------------------------------------

namespace
{
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

    bool isMapZoomInRange(double viewZoom, int minZoom, int maxZoom)
    {
        if (minZoom < 0 && maxZoom < 0) {
            return true;
        }
        const int z = static_cast<int>(std::floor(viewZoom));
        if (minZoom >= 0 && z < minZoom) {
            return false;
        }
        if (maxZoom >= 0 && z > maxZoom) {
            return false;
        }
        return true;
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

    // #region agent log
    void agentSessionLog(
        const char *hypothesisId,
        const char *location,
        const char *message,
        double viewZoom,
        int minRenderLevel,
        std::size_t tilesToRender,
        uint32_t tilesVisited,
        uint32_t maxDepthVisited,
        int32_t workerLoadQueueLength) {
        try
        {
            std::filesystem::create_directories("logs");
        }
        catch (...)
        {
        }
        FILE *f = std::fopen("logs/cesium-selection-diagnostics.log", "ab");
        if (!f) {
            return;
        }
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        std::fprintf(
            f,
            "{\"sessionId\":\"9cefe4\",\"hypothesisId\":\"%s\",\"location\":\"%s\",\"message\":\"%s\","
            "\"data\":{\"viewZoom\":%.4f,\"minRenderLevel\":%d,\"tilesToRender\":%zu,\"tilesVisited\":%u,\"maxDepthVisited\":%u,\"workerLoadQueueLength\":%d},\"timestamp\":%lld}\n",
            hypothesisId,
            location,
            message,
            viewZoom,
            minRenderLevel,
            tilesToRender,
            tilesVisited,
            maxDepthVisited,
            workerLoadQueueLength,
            static_cast<long long>(ms.count()));
        std::fclose(f);
    }
    // #endregion

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
    options.minimumRenderableLevel = minimumRenderableLevel;
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
    options.minimumRenderableLevel = minimumRenderableLevel;
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
    if (!_tileset)
        return;

    auto* tileset = static_cast<Cesium3DTilesSelection::Tileset*>(_tileset);
    tileset->waitForAllLoadsToComplete(150.0);
    delete tileset;
    _tileset = nullptr;
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
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    if (tileset)
        tileset->getOptions().minimumRenderableLevel = level;
}

void CesiumTilesetNode::setMapZoomRange(int minZoom, int maxZoom)
{
    _mapZoomMin = minZoom;
    _mapZoomMax = maxZoom;
}

int CesiumTilesetNode::getMapZoomMin() const
{
    return _mapZoomMin;
}

int CesiumTilesetNode::getMapZoomMax() const
{
    return _mapZoomMax;
}

osg::Group*
CesiumTilesetNode::tileParent()
{
    if (_clampToGround)
        return ensureDrapeRoot();

    return this;
}

osg::Group*
CesiumTilesetNode::ensureDrapeRoot()
{
    if (!_drapeRoot.valid())
    {
        _drapeRoot = new osgEarth::DrapeableNode();
        _drapeRoot->setName("CesiumTilesetGroundDrapeRoot");
        _drapeRoot->setDrapingEnabled(true);
        _drapeRoot->setIntersectionTraversalEnabled(false);
        addChild(_drapeRoot.get());
        writeClampDiagnostic("[CesiumDrape] created DrapeableNode root");
    }
    return _drapeRoot.get();
}

// ---- Traversal ---------------------------------------------------------------

void
CesiumTilesetNode::traverse(osg::NodeVisitor& nv)
{
    if (isShuttingDown())
    {
        osg::Group::traverse(nv);
        return;
    }

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

        const double viewZoom = estimateViewZoomLevel(osgEye, vfov, cv->getViewport()->height());

        if (!isMapZoomInRange(viewZoom, _mapZoomMin, _mapZoomMax))
        {
            osg::Group* parent = tileParent();
            parent->removeChildren(0, parent->getNumChildren());
            osg::Group::traverse(nv);
            return;
        }

        // TODO:  Multiple views
        std::vector<Cesium3DTilesSelection::ViewState> viewStates;
        Cesium3DTilesSelection::ViewState viewState(pos, dir, up, viewportSize, hfov, vfov);
        viewStates.push_back(viewState);
        Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
        tileset->getAsyncSystem().dispatchMainThreadTasks();
        auto updates = tileset->updateView(viewStates);

        // #region agent log
        {
            static std::chrono::steady_clock::time_point s_lastUpdateLog{};
            const auto now = std::chrono::steady_clock::now();
            const std::size_t nTiles = updates.tilesToRenderThisFrame.size();
            const bool first = (s_lastUpdateLog == std::chrono::steady_clock::time_point{});
            if (first || nTiles > 0 ||
                now - s_lastUpdateLog >= std::chrono::seconds(3)) {
                s_lastUpdateLog = now;
                agentSessionLog(
                    first ? "H4" : "H6",
                    "CesiumTilesetNode.cpp:traverse",
                    first ? "first_update_view" : "update_view",
                    viewZoom,
                    _minimumRenderableLevel,
                    nTiles,
                    updates.tilesVisited,
                    updates.maxDepthVisited,
                    updates.workerThreadTileLoadQueueLength);
            }
        }
        // #endregion

        osg::Group* parent = tileParent();
        std::vector<osg::ref_ptr<osg::Node>> displayNodes;

        for (auto tile : updates.tilesToRenderThisFrame)
        {
            if (tile->getContent().isRenderContent())
            {
                MainThreadResult* result = reinterpret_cast<MainThreadResult*>(tile->getContent().getRenderContent()->getRenderResources());
                if (result && result->node.valid())
                {
                    displayNodes.push_back(result->node.get());
                }
            }
        }

        parent->removeChildren(0, parent->getNumChildren());
        for (const auto& node : displayNodes)
        {
            if (node.valid())
                parent->addChild(node.get());
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
