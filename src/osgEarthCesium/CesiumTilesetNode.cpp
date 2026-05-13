/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "CesiumTilesetNode"
#include "Context"
#include "CesiumIon"
#include "PrepareRenderResources"
#include "Settings"

#include <osgEarth/Notify>
#include <osgUtil/CullVisitor>
#include <Cesium3DTilesSelection/BoundingVolume.h>

#include <algorithm>
#include <cmath>
#include <iterator>

using namespace osgEarth::Cesium;

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
}

CesiumTilesetNode::CesiumTilesetNode(unsigned int assetID, const std::string& server, const std::string& token, float maximumScreenSpaceError, std::vector<int> overlays, const TilesetRenderStyleOptions& renderStyle, int minimumRenderableLevel)
{ 
    _minimumRenderableLevel = minimumRenderableLevel;
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
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    delete tileset;
}

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

unsigned int CesiumTilesetNode::reclampLoadedTiles(const TilesetRenderStyleOptions& renderStyle)
{
    if (!_tileset || !renderStyle.clampToGround)
    {
        return 0u;
    }

    unsigned int clampedGeometryCount = 0u;
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    tileset->forEachLoadedTile([&](const Cesium3DTilesSelection::Tile& tile) {
        if (!tile.getContent().isRenderContent())
        {
            return;
        }

        const auto* renderContent = tile.getContent().getRenderContent();
        if (!renderContent)
        {
            return;
        }

        MainThreadResult* result =
            reinterpret_cast<MainThreadResult*>(renderContent->getRenderResources());
        if (result && result->node.valid())
        {
            clampedGeometryCount += reclampTerrain(result->node.get(), renderStyle);
        }
    });

    if (clampedGeometryCount > 0u)
    {
        dirtyBound();
    }
    return clampedGeometryCount;
}

std::shared_ptr<TilesetTerrainClampTask>
CesiumTilesetNode::snapshotLoadedTilesTerrainClampTask(const TilesetRenderStyleOptions& renderStyle)
{
    auto mergedTask = std::make_shared<TilesetTerrainClampTask>();
    mergedTask->renderStyle = renderStyle;
    if (!_tileset || !renderStyle.clampToGround)
    {
        return mergedTask;
    }

    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    tileset->forEachLoadedTile([&](const Cesium3DTilesSelection::Tile& tile) {
        if (!tile.getContent().isRenderContent())
        {
            return;
        }

        const auto* renderContent = tile.getContent().getRenderContent();
        if (!renderContent)
        {
            return;
        }

        MainThreadResult* result =
            reinterpret_cast<MainThreadResult*>(renderContent->getRenderResources());
        if (result && result->node.valid())
        {
            auto task = snapshotTerrainClampTask(result->node.get(), renderStyle);
            if (task && !task->geometries.empty())
            {
                mergedTask->geometries.insert(
                    mergedTask->geometries.end(),
                    std::make_move_iterator(task->geometries.begin()),
                    std::make_move_iterator(task->geometries.end()));
            }
        }
    });
    return mergedTask;
}

unsigned int
CesiumTilesetNode::applyLoadedTilesTerrainClampTask(TilesetTerrainClampTask& task)
{
    const unsigned int clampedGeometryCount = applyResolvedTerrainClampTask(task);
    if (clampedGeometryCount > 0u)
    {
        dirtyBound();
    }
    return clampedGeometryCount;
}

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
        cv->getProjectionMatrix()->getPerspective(vfov, ar, znear, zfar);
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

        removeChildren(0, getNumChildren());
        for (auto tile : updates.tilesToRenderThisFrame)
        {
            if (tile->getContent().isRenderContent())
            {
                MainThreadResult* result = reinterpret_cast<MainThreadResult*>(tile->getContent().getRenderContent()->getRenderResources());
                if (result && result->node.valid()) {
                    addChild(result->node.get());
                }
            }
        }
    }
    else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {        
        osg::Group::traverse(nv);
    }

    osg::Group::traverse(nv);
}

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