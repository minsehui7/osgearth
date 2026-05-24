/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
 * MIT License
 */
#include "HyperTerrainBridge"
#include "HyperTerrainPrepareRendererResources"
#include "HyperTerrainImageryFactory"
#include "CesiumIon"
#include "Context"
#include "Settings"

#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/Tileset.h>
#include <Cesium3DTilesSelection/TilesetExternals.h>
#include <Cesium3DTilesSelection/TilesetOptions.h>
#include <Cesium3DTilesSelection/ViewState.h>
#include <Cesium3DTilesSelection/ViewUpdateResult.h>
#include <CesiumGeospatial/Ellipsoid.h>

#include <osgEarth/Notify>
#include <osg/Group>

#include <glm/glm.hpp>

#include <optional>
#include <vector>

using namespace osgEarth::Cesium;

namespace {

// Match CesiumLayer default and CesiumTilesetNode server key (CesiumIon::getContext map).
constexpr const char* kDefaultCesiumIonServer = "https://api.cesium.com/";

using Cesium3DTilesSelection::Tile;
using CesiumGeospatial::GlobeRectangle;
using CesiumEllipsoid = CesiumGeospatial::Ellipsoid;

void appendGlobeRectsFromTiles(
    const std::vector<Tile::ConstPointer>& tiles,
    const CesiumEllipsoid& ellipsoid,
    std::vector<GlobeRectangle>& out)
{
    for (const auto& tilePtr : tiles) {
        if (!tilePtr)
            continue;
        const auto bv = Cesium3DTilesSelection::transformBoundingVolume(
            tilePtr->getTransform(), tilePtr->getBoundingVolume());
        if (auto gr = Cesium3DTilesSelection::estimateGlobeRectangle(bv, ellipsoid))
            out.push_back(*gr);
    }
}

bool globeRectOverlapsAny(
    const std::optional<GlobeRectangle>& rect,
    const std::vector<GlobeRectangle>& rects)
{
    if (!rect || rect->isEmpty())
        return false;
    for (const GlobeRectangle& pr : rects) {
        if (pr.isEmpty())
            continue;
        if (auto inter = rect->computeIntersection(pr)) {
            if (!inter->isEmpty())
                return true;
        }
    }
    return false;
}

Cesium3DTilesSelection::ViewState buildViewState(const HyperTerrainViewParams& p)
{
    const glm::dvec3 position(p.eyeX, p.eyeY, p.eyeZ);
    const glm::dvec3 direction(p.dirX, p.dirY, p.dirZ);
    const glm::dvec3 up(p.upX, p.upY, p.upZ);
    double vw = p.viewportWidth;
    double vh = p.viewportHeight;
    if (p.haveViewPixelDims && p.viewPixelWidth > 0.0 && p.viewPixelHeight > 0.0) {
        vw = p.viewPixelWidth;
        vh = p.viewPixelHeight;
    }
    const glm::dvec2 viewportSize(vw, vh);
    const double vFov = p.vFovRad;
    const double hFov = vFov * (vw / vh);

    return Cesium3DTilesSelection::ViewState(
        position, direction, up, viewportSize, hFov, vFov);
}

} // namespace

struct HyperTerrainBridge::Impl {
    osg::ref_ptr<osg::Group> tileGroup;
    std::shared_ptr<HyperTerrainPrepareRendererResources> renderer;
    std::unique_ptr<Cesium3DTilesSelection::Tileset> primaryTileset;
    std::unique_ptr<Cesium3DTilesSelection::Tileset> fallbackTileset;
    Context* context{nullptr};
};

HyperTerrainBridge::HyperTerrainBridge(osg::Group* tileGroup)
    : _impl(std::make_unique<Impl>())
{
    _impl->tileGroup = tileGroup;
}

HyperTerrainBridge::~HyperTerrainBridge()
{
    _impl->primaryTileset.reset();
    _impl->fallbackTileset.reset();
    _impl->renderer.reset();
}

bool HyperTerrainBridge::initialize(const HyperTerrainInitOptions& options)
{
    HyperTerrainImageryFactory::setImageryTileTransportDiagLogging(options.logTileHttpUrls);

    _impl->renderer = std::make_shared<HyperTerrainPrepareRendererResources>(_impl->tileGroup.get());
    _impl->context = CesiumIon::instance().getContext(kDefaultCesiumIonServer);

    if (!_impl->context) {
        OSG_WARN << "[HyperTerrainBridge] Context unavailable." << std::endl;
        return false;
    }

    Cesium3DTilesSelection::TilesetExternals externals{
        _impl->context->assetAccessor,
        _impl->renderer,
        _impl->context->asyncSystem,
        _impl->context->creditSystem,
        HyperTerrainImageryFactory::getOrCreateTilesetLogger(),
        nullptr,
    };

    Cesium3DTilesSelection::TilesetOptions tilesetOptions;
    tilesetOptions.maximumScreenSpaceError = options.maximumScreenSpaceError;

    _impl->primaryTileset.reset();
    _impl->fallbackTileset.reset();

    const bool dualTerrain = !options.terrainEndpoint.empty()
        && !options.ionAccessToken.empty()
        && !options.fallbackIonAssetIdStr.empty();

    try {
        if (dualTerrain) {
            const int64_t fallbackId = std::stoll(options.fallbackIonAssetIdStr);
            _impl->primaryTileset = std::make_unique<Cesium3DTilesSelection::Tileset>(
                externals, options.terrainEndpoint, tilesetOptions);
            _impl->fallbackTileset = std::make_unique<Cesium3DTilesSelection::Tileset>(
                externals, fallbackId, options.ionAccessToken, tilesetOptions);
        } else if (options.ionAssetId > 0 && !options.ionAccessToken.empty()) {
            _impl->primaryTileset = std::make_unique<Cesium3DTilesSelection::Tileset>(
                externals, options.ionAssetId, options.ionAccessToken, tilesetOptions);
        } else if (!options.terrainEndpoint.empty()) {
            _impl->primaryTileset = std::make_unique<Cesium3DTilesSelection::Tileset>(
                externals, options.terrainEndpoint, tilesetOptions);
        } else {
            OSG_WARN << "[HyperTerrainBridge] No terrain_endpoint or ion_asset_id configured."
                     << std::endl;
            return false;
        }
    } catch (...) {
        OSG_WARN << "[HyperTerrainBridge] Tileset init failed." << std::endl;
        return false;
    }

    return _impl->primaryTileset != nullptr;
}

void HyperTerrainBridge::updateFrame(const HyperTerrainViewParams& viewParams, bool tilesetReady)
{
    if (isShuttingDown())
        return;
    if (!_impl->primaryTileset)
        return;
    if (!viewParams.hasCameraData || !tilesetReady)
        return;

    _impl->context->asyncSystem.dispatchMainThreadTasks();

    const auto viewState = buildViewState(viewParams);
    const std::vector<Cesium3DTilesSelection::ViewState> viewStates{viewState};

    const auto primaryResult = _impl->primaryTileset->updateView(viewStates);

    std::optional<Cesium3DTilesSelection::ViewUpdateResult> fallbackResult;
    if (_impl->fallbackTileset) {
        fallbackResult = _impl->fallbackTileset->updateView(viewStates);
    }

    const unsigned numChildren = _impl->tileGroup->getNumChildren();
    for (unsigned i = 0; i < numChildren; ++i) {
        _impl->tileGroup->getChild(i)->setNodeMask(0x0);
    }

    std::vector<GlobeRectangle> primaryRects;
    appendGlobeRectsFromTiles(primaryResult.tilesToRenderThisFrame, CesiumEllipsoid::WGS84, primaryRects);

    auto tryShowTile = [this](const Tile::ConstPointer& tilePtr) {
        if (!tilePtr || !_impl->renderer)
            return false;
        HyperTerrainTileRenderData renderData;
        if (!_impl->renderer->tryResolveTileRenderData(*tilePtr, renderData) || !renderData.xform.valid())
            return false;
        osg::MatrixTransform* xform = renderData.xform.get();
        if (!xform)
            return false;
        bool attachedToTileGroup = false;
        const unsigned parentCount = xform->getNumParents();
        for (unsigned p = 0; p < parentCount; ++p) {
            if (xform->getParent(p) == _impl->tileGroup.get()) {
                attachedToTileGroup = true;
                break;
            }
        }
        if (!attachedToTileGroup)
            return false;
        xform->setNodeMask(~0u);
        return true;
    };

    for (const auto& tilePtr : primaryResult.tilesToRenderThisFrame)
        tryShowTile(tilePtr);

    if (fallbackResult.has_value()) {
        for (const auto& tilePtr : fallbackResult->tilesToRenderThisFrame) {
            if (!tilePtr)
                continue;
            const auto bv = Cesium3DTilesSelection::transformBoundingVolume(
                tilePtr->getTransform(), tilePtr->getBoundingVolume());
            const auto fbGlobe = Cesium3DTilesSelection::estimateGlobeRectangle(bv, CesiumEllipsoid::WGS84);
            if (globeRectOverlapsAny(fbGlobe, primaryRects))
                continue;
            tryShowTile(tilePtr);
        }
    }
}

void HyperTerrainBridge::addImageryOverlay(RasterOverlayPtr overlay)
{
    if (!overlay)
        return;
    if (_impl->primaryTileset)
        _impl->primaryTileset->getOverlays().add(overlay);
    if (_impl->fallbackTileset)
        _impl->fallbackTileset->getOverlays().add(overlay);
}

void HyperTerrainBridge::removeImageryOverlay(RasterOverlayPtr overlay)
{
    if (!overlay)
        return;
    if (_impl->primaryTileset)
        _impl->primaryTileset->getOverlays().remove(overlay);
    if (_impl->fallbackTileset)
        _impl->fallbackTileset->getOverlays().remove(overlay);
}

void HyperTerrainBridge::setImageryOverlayAlpha(RasterOverlayPtr overlay, float alpha)
{
    if (!overlay || !_impl->renderer)
        return;
    _impl->renderer->setRasterOverlayAlpha(overlay.get(), alpha);
}

void HyperTerrainBridge::setTmsImageryExposure(float exposure)
{
    HyperTerrainImageryFactory::setTmsImageryExposure(exposure);
    if (_impl->renderer)
        _impl->renderer->setTmsImageryExposure(exposure);
}

bool HyperTerrainBridge::hasPrimaryTileset() const
{
    return _impl->primaryTileset != nullptr;
}
