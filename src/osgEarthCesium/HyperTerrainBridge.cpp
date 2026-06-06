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
#include "UserTilesetLoadGate"

#include <osgEarth/TerrainStreamingCullPolicy>

#include <Cesium3DTilesSelection/ITileExcluder.h>
#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/Tileset.h>
#include <Cesium3DTilesSelection/TilesetExternals.h>
#include <Cesium3DTilesSelection/TilesetOptions.h>
#include <Cesium3DTilesSelection/ViewState.h>
#include <Cesium3DTilesSelection/ViewUpdateResult.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>

#include <osgEarth/Notify>
#include <osg/Group>
#include <osg/MatrixTransform>

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>
#include <vector>

using namespace osgEarth::Cesium;

namespace {

// Match CesiumLayer default and CesiumTilesetNode server key (CesiumIon::getContext map).
constexpr const char* kDefaultCesiumIonServer = "https://api.cesium.com/";

constexpr size_t kPruneChildMargin = 32;
constexpr int kPruneRateLimitFrames = 30;
constexpr int kDiagLogIntervalFrames = 300;
constexpr uint32_t kStuckOverlayFrameThreshold = 180;
constexpr int kDrainMaxPassesMeshPending = 12;
constexpr int kDrainMaxPassesImageryPending = 3;
constexpr int kDrainStalePassLimit = 2;

using Cesium3DTilesSelection::Tile;
using Cesium3DTilesSelection::TileID;
using CesiumGeometry::QuadtreeTileID;
using CesiumGeometry::UpsampledQuadtreeNode;
using CesiumGeospatial::Cartographic;
using CesiumGeospatial::GlobeRectangle;
using CesiumEllipsoid = CesiumGeospatial::Ellipsoid;

struct PrimaryCoverageTile {
    GlobeRectangle rect;
    uint32_t level;
};

std::optional<uint32_t> quadtreeLevelFromTile(const Tile& tile)
{
    const TileID& id = tile.getTileID();
    if (const auto* quadId = std::get_if<QuadtreeTileID>(&id))
        return quadId->level;
    if (const auto* upsampled = std::get_if<UpsampledQuadtreeNode>(&id))
        return upsampled->tileID.level;
    return std::nullopt;
}

bool isUpsampledPrimaryTile(const Tile& tile)
{
    return std::holds_alternative<UpsampledQuadtreeNode>(tile.getTileID());
}

void appendPrimaryCoverageFromTiles(
    const std::vector<Tile::ConstPointer>& tiles,
    const CesiumEllipsoid& ellipsoid,
    bool excludeUpsampled,
    std::vector<PrimaryCoverageTile>& out)
{
    for (const auto& tilePtr : tiles) {
        if (!tilePtr)
            continue;
        if (excludeUpsampled && isUpsampledPrimaryTile(*tilePtr))
            continue;

        const std::optional<uint32_t> level = quadtreeLevelFromTile(*tilePtr);
        // z=0 blank root covers a hemisphere but is not real local coverage.
        if (!level || *level == 0)
            continue;

        const auto bv = Cesium3DTilesSelection::transformBoundingVolume(
            tilePtr->getTransform(), tilePtr->getBoundingVolume());
        if (auto gr = Cesium3DTilesSelection::estimateGlobeRectangle(bv, ellipsoid))
            out.push_back({*gr, *level});
    }
}

bool globeRectangleFullyContains(
    const GlobeRectangle& outer,
    const GlobeRectangle& inner)
{
    if (inner.isEmpty() || outer.isEmpty())
        return false;

    const auto containsCorner = [&](double longitude, double latitude) {
        return outer.contains(Cartographic(longitude, latitude, 0.0));
    };

    return containsCorner(inner.getWest(), inner.getSouth()) &&
           containsCorner(inner.getEast(), inner.getSouth()) &&
           containsCorner(inner.getWest(), inner.getNorth()) &&
           containsCorner(inner.getEast(), inner.getNorth());
}

bool primaryCoversFallbackTile(
    const std::optional<GlobeRectangle>& fallbackGlobe,
    uint32_t fallbackLevel,
    const std::vector<PrimaryCoverageTile>& primaryTiles)
{
    if (!fallbackGlobe || fallbackGlobe->isEmpty())
        return false;

    for (const PrimaryCoverageTile& primary : primaryTiles) {
        if (primary.level < fallbackLevel)
            continue;
        if (globeRectangleFullyContains(primary.rect, *fallbackGlobe))
            return true;
    }
    return false;
}

class PrimaryCoverageFallbackExcluder
    : public Cesium3DTilesSelection::ITileExcluder {
public:
    void setCoverage(
        std::vector<PrimaryCoverageTile> coverage,
        int32_t ionAlwaysOnLevelMax)
    {
        _coverage = std::move(coverage);
        _ionAlwaysOnLevelMax = ionAlwaysOnLevelMax;
    }

    void clearCoverage()
    {
        _coverage.clear();
        _ionAlwaysOnLevelMax = -1;
    }

    bool shouldExclude(const Tile& tile) const noexcept override
    {
        if (_ionAlwaysOnLevelMax < 0 || _coverage.empty())
            return false;

        const std::optional<uint32_t> level = quadtreeLevelFromTile(tile);
        if (!level || *level <= static_cast<uint32_t>(_ionAlwaysOnLevelMax))
            return false;

        const auto bv = Cesium3DTilesSelection::transformBoundingVolume(
            tile.getTransform(), tile.getBoundingVolume());
        const auto fbGlobe =
            Cesium3DTilesSelection::estimateGlobeRectangle(bv, CesiumEllipsoid::WGS84);
        return primaryCoversFallbackTile(fbGlobe, *level, _coverage);
    }

private:
    std::vector<PrimaryCoverageTile> _coverage;
    int32_t _ionAlwaysOnLevelMax{-1};
};

void destroyTilesetQuietly(std::unique_ptr<Cesium3DTilesSelection::Tileset>& tileset)
{
    if (!tileset) {
        return;
    }
    tileset->waitForAllLoadsToComplete(150.0);
    tileset.reset();
}

Cesium3DTilesSelection::ViewState buildViewState(const HyperTerrainViewParams& p)
{
    const glm::dvec3 position(p.eyeX, p.eyeY, p.eyeZ);
    const glm::dvec3 direction(p.dirX, p.dirY, p.dirZ);
    const glm::dvec3 up(p.upX, p.upY, p.upZ);
    double vw = p.viewportWidth;
    double vh = p.viewportHeight;
    if ((vw < 1.0 || vh < 1.0) && p.haveViewPixelDims
        && p.viewPixelWidth > 0.0 && p.viewPixelHeight > 0.0) {
        vw = p.viewPixelWidth;
        vh = p.viewPixelHeight;
    }
    const glm::dvec2 viewportSize(vw, vh);
    const double vFov = p.vFovRad;
    const double hFov =
        (p.hFovRad > 0.0) ? p.hFovRad : (vFov * (vw / vh));

    return Cesium3DTilesSelection::ViewState(
        position, direction, up, viewportSize, hFov, vFov);
}

struct TileSelectionStats {
    size_t selectedCount = 0;
    size_t renderContentCount = 0;
    size_t renderReadyCount = 0;
    size_t pendingImageryCount = 0;
};

TileSelectionStats countTerrainTileSelection(
    const std::vector<Tile::ConstPointer>& tiles,
    const HyperTerrainPrepareRendererResources* renderer,
    bool requireImageryForDisplay)
{
    TileSelectionStats stats;
    stats.selectedCount = tiles.size();
    if (!renderer) {
        return stats;
    }
    for (const auto& tilePtr : tiles) {
        if (!tilePtr || !tilePtr->getContent().isRenderContent()) {
            continue;
        }
        ++stats.renderContentCount;
        HyperTerrainTileRenderData renderData;
        if (!renderer->tryResolveTileRenderData(*tilePtr, renderData) || !renderData.xform.valid()) {
            continue;
        }
        const bool imageryReady =
            !requireImageryForDisplay || renderer->tileImageryReadyForDisplay(*tilePtr);
        if (!imageryReady) {
            ++stats.pendingImageryCount;
        }
        if (requireImageryForDisplay && !imageryReady) {
            continue;
        }
        ++stats.renderReadyCount;
    }
    return stats;
}

void applyMainThreadRebuildBudget(
    Cesium3DTilesSelection::Tileset* tileset,
    double& rebuildBudgetCredit)
{
    if (!tileset) {
        return;
    }

    Cesium3DTilesSelection::TilesetOptions& options = tileset->getOptions();
    const double rate = terrainRebuildsPerFrameRate();
    if (rate <= 0.0) {
        options.mainThreadLoadingTimeLimit = 0.0;
        options.maximumMainThreadTilesPerLoadPass = 0;
        options.enforceMainThreadTilesPerLoadPass = false;
        return;
    }

    options.mainThreadLoadingTimeLimit = 86400000.0;
    options.enforceMainThreadTilesPerLoadPass = true;
    rebuildBudgetCredit += rate;
    const int budget = static_cast<int>(std::floor(rebuildBudgetCredit));
    rebuildBudgetCredit -= static_cast<double>(budget);
    options.maximumMainThreadTilesPerLoadPass =
        static_cast<uint32_t>(std::max(4, budget));
}

void maybeRelaxImplicitRootStuckSse(
    Cesium3DTilesSelection::Tileset* tileset,
    const std::vector<Tile::ConstPointer>& tilesToRender,
    size_t renderContentCount,
    int& stuckImplicitRootFrames)
{
    if (!tileset) {
        return;
    }

    const size_t selectedCount = tilesToRender.size();
    if (selectedCount != 1 || renderContentCount != 0) {
        return;
    }

    const Tile::ConstPointer& onlyTile = tilesToRender.front();
    if (!onlyTile) {
        return;
    }

    const TileID& tileId = onlyTile->getTileID();
    if (const auto* quadId = std::get_if<QuadtreeTileID>(&tileId)) {
        if (quadId->level == 0) {
            if (++stuckImplicitRootFrames >= 6) {
                Cesium3DTilesSelection::TilesetOptions& opts = tileset->getOptions();
                const float prev = opts.maximumScreenSpaceError;
                opts.maximumScreenSpaceError = std::max(1.0f, prev * 0.5f);
                stuckImplicitRootFrames = 0;
            }
        } else {
            stuckImplicitRootFrames = 0;
        }
    }
}

void drainMainThreadUntilDrawable(
    Cesium3DTilesSelection::Tileset* tileset,
    const std::vector<Tile::ConstPointer>& tilesToRender,
    const HyperTerrainPrepareRendererResources* renderer,
    bool requireImageryForDisplay,
    TileSelectionStats& stats,
    int& outPassesUsed)
{
    outPassesUsed = 0;
    if (!tileset) {
        return;
    }

    int stalePasses = 0;
    size_t prevContent = stats.renderContentCount;
    size_t prevReady = stats.renderReadyCount;

    const int maxPasses =
        (requireImageryForDisplay && stats.renderContentCount > 0 && stats.renderReadyCount == 0)
            ? kDrainMaxPassesImageryPending
            : kDrainMaxPassesMeshPending;

    for (int pass = 0; pass < maxPasses && stats.selectedCount > 0 && stats.renderReadyCount == 0;
         ++pass) {
        ++outPassesUsed;
        tileset->getAsyncSystem().dispatchMainThreadTasks();
        stats = countTerrainTileSelection(tilesToRender, renderer, requireImageryForDisplay);
        if (stats.renderContentCount == 0 || stats.renderReadyCount > 0) {
            break;
        }
        if (requireImageryForDisplay && stats.pendingImageryCount >= stats.renderContentCount) {
            break;
        }
        if (stats.renderContentCount == prevContent && stats.renderReadyCount == prevReady) {
            if (++stalePasses >= kDrainStalePassLimit) {
                break;
            }
        } else {
            stalePasses = 0;
            prevContent = stats.renderContentCount;
            prevReady = stats.renderReadyCount;
        }
    }
}

} // namespace

struct HyperTerrainBridge::Impl {
    osg::ref_ptr<osg::Group> tileGroup;
    std::shared_ptr<HyperTerrainPrepareRendererResources> renderer;
    std::unique_ptr<Cesium3DTilesSelection::Tileset> primaryTileset;
    std::unique_ptr<Cesium3DTilesSelection::Tileset> fallbackTileset;
    std::shared_ptr<PrimaryCoverageFallbackExcluder> fallbackExcluder;
    Context* context{nullptr};
    int32_t cesiumIonTerrainAlwaysOnLevelMax{-1};
    double primaryRebuildBudgetCredit = 0.0;
    double fallbackRebuildBudgetCredit = 0.0;
    int primaryStuckImplicitRootFrames = 0;
    int fallbackStuckImplicitRootFrames = 0;
    int pruneCooldownFrames = 0;
    int diagFrameCounter = 0;
    int primaryDrainPassesLastFrame = 0;
    int fallbackDrainPassesLastFrame = 0;
    /// App-registered TMS overlays (Mapbox/VWorld/Naver/OWM); when zero, show mesh without imagery gate.
    int registeredImageryOverlayCount = 0;
    /// While true, user 3D Tiles layers defer cull/update and drape work for terrain priority.
    bool deferUserTilesets = true;
    int terrainStableFrames = 0;
};

HyperTerrainBridge::HyperTerrainBridge(osg::Group* tileGroup)
    : _impl(std::make_unique<Impl>())
{
    _impl->tileGroup = tileGroup;
}

HyperTerrainBridge::~HyperTerrainBridge()
{
    destroyTilesetQuietly(_impl->fallbackTileset);
    destroyTilesetQuietly(_impl->primaryTileset);
    _impl->renderer.reset();
}

bool HyperTerrainBridge::initialize(const HyperTerrainInitOptions& options)
{
    clearTerrainLoadingExclusions();

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
    _impl->cesiumIonTerrainAlwaysOnLevelMax = options.cesiumIonTerrainAlwaysOnLevelMax;

    const bool dualTerrain = !options.terrainEndpoint.empty()
        && !options.ionAccessToken.empty()
        && !options.fallbackIonAssetIdStr.empty();

    Cesium3DTilesSelection::TilesetOptions primaryOptions = tilesetOptions;
    if (dualTerrain && options.cesiumIonTerrainAlwaysOnLevelMax >= 0) {
        primaryOptions.minimumRenderableLevel = options.cesiumIonTerrainAlwaysOnLevelMax + 1;
    }

    try {
        if (dualTerrain) {
            const int64_t fallbackId = std::stoll(options.fallbackIonAssetIdStr);
            _impl->primaryTileset = std::make_unique<Cesium3DTilesSelection::Tileset>(
                externals, options.terrainEndpoint, primaryOptions);

            Cesium3DTilesSelection::TilesetOptions fallbackOptions = tilesetOptions;
            if (options.cesiumIonTerrainAlwaysOnLevelMax >= 0) {
                _impl->fallbackExcluder =
                    std::make_shared<PrimaryCoverageFallbackExcluder>();
                fallbackOptions.excluders.push_back(_impl->fallbackExcluder);
            }
            _impl->fallbackTileset = std::make_unique<Cesium3DTilesSelection::Tileset>(
                externals, fallbackId, options.ionAccessToken, fallbackOptions);
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

    if (options.ionAssetId > 0) {
        registerTerrainLoadingExclusionIonAsset(options.ionAssetId);
    }
    if (dualTerrain && !options.fallbackIonAssetIdStr.empty()) {
        try {
            registerTerrainLoadingExclusionIonAsset(
                std::stoll(options.fallbackIonAssetIdStr));
        } catch (...) {
        }
    }
    if (!options.terrainEndpoint.empty()) {
        registerTerrainLoadingExclusionUrl(options.terrainEndpoint);
    }

    osgEarth::setTerrainStreamingPrioritizeTerrainCullFn(
        []() { return isHyperTerrainLoadingActive(); });

    return _impl->primaryTileset != nullptr;
}

namespace {

// Short cooldown after first display-ready terrain tile (not 45; signature churn prevented release).
constexpr int kTerrainStableFramesBeforeUserTilesets = 10;

void updateDeferUserTilesetsState(
    bool activelyLoading,
    bool& deferUserTilesets,
    int& terrainStableFrames)
{
    if (activelyLoading) {
        deferUserTilesets = true;
        terrainStableFrames = 0;
        return;
    }
    if (terrainStableFrames < kTerrainStableFramesBeforeUserTilesets) {
        ++terrainStableFrames;
        deferUserTilesets = true;
        return;
    }
    deferUserTilesets = false;
}

} // namespace

void HyperTerrainBridge::updateFrame(const HyperTerrainViewParams& viewParams, bool tilesetReady)
{
    if (isShuttingDown())
        return;
    if (!_impl->primaryTileset) {
        _impl->deferUserTilesets = false;
        applyUserTilesetLoadDeferPolicy(false);
        setHyperTerrainLoadingActive(false);
        return;
    }
    if (!viewParams.hasCameraData || !tilesetReady) {
        _impl->deferUserTilesets = true;
        _impl->terrainStableFrames = 0;
        applyUserTilesetLoadDeferPolicy(true);
        setHyperTerrainLoadingActive(true);
        return;
    }

    applyMainThreadRebuildBudget(_impl->primaryTileset.get(), _impl->primaryRebuildBudgetCredit);
    if (_impl->fallbackTileset) {
        applyMainThreadRebuildBudget(_impl->fallbackTileset.get(), _impl->fallbackRebuildBudgetCredit);
    }

    _impl->context->asyncSystem.dispatchMainThreadTasks();

    const auto viewState = buildViewState(viewParams);
    const std::vector<Cesium3DTilesSelection::ViewState> viewStates{viewState};

    const bool dualTerrain = _impl->fallbackTileset != nullptr;
    const int32_t ionAlwaysOnMax = _impl->cesiumIonTerrainAlwaysOnLevelMax;
    const bool ionLevelGate = dualTerrain && ionAlwaysOnMax >= 0;

    Cesium3DTilesSelection::ViewUpdateResult primaryResult;
    std::optional<Cesium3DTilesSelection::ViewUpdateResult> fallbackResult;

    if (ionLevelGate) {
        if (_impl->fallbackExcluder)
            _impl->fallbackExcluder->clearCoverage();

        const auto coarseFallbackResult =
            _impl->fallbackTileset->updateView(viewStates);

        bool skipPrimaryUpdate = true;
        for (const auto& tilePtr : coarseFallbackResult.tilesToRenderThisFrame) {
            if (!tilePtr)
                continue;
            const uint32_t fbLevel = quadtreeLevelFromTile(*tilePtr).value_or(0u);
            if (fbLevel > static_cast<uint32_t>(ionAlwaysOnMax)) {
                skipPrimaryUpdate = false;
                break;
            }
        }

        if (skipPrimaryUpdate) {
            fallbackResult = std::move(coarseFallbackResult);
        } else {
            primaryResult = _impl->primaryTileset->updateView(viewStates);

            std::vector<PrimaryCoverageTile> primaryCoverageForExcluder;
            appendPrimaryCoverageFromTiles(
                primaryResult.tilesToRenderThisFrame,
                CesiumEllipsoid::WGS84,
                dualTerrain,
                primaryCoverageForExcluder);

            if (_impl->fallbackExcluder) {
                _impl->fallbackExcluder->setCoverage(
                    std::move(primaryCoverageForExcluder),
                    ionAlwaysOnMax);
            }

            fallbackResult = _impl->fallbackTileset->updateView(viewStates);
        }
    } else {
        primaryResult = _impl->primaryTileset->updateView(viewStates);
        if (_impl->fallbackTileset) {
            fallbackResult = _impl->fallbackTileset->updateView(viewStates);
        }
    }

    const HyperTerrainPrepareRendererResources* renderer = _impl->renderer.get();
    const bool requireImageryForDisplay = _impl->registeredImageryOverlayCount > 0;

    TileSelectionStats primaryStats = countTerrainTileSelection(
        primaryResult.tilesToRenderThisFrame, renderer, requireImageryForDisplay);
    maybeRelaxImplicitRootStuckSse(
        _impl->primaryTileset.get(),
        primaryResult.tilesToRenderThisFrame,
        primaryStats.renderContentCount,
        _impl->primaryStuckImplicitRootFrames);
    drainMainThreadUntilDrawable(
        _impl->primaryTileset.get(),
        primaryResult.tilesToRenderThisFrame,
        renderer,
        requireImageryForDisplay,
        primaryStats,
        _impl->primaryDrainPassesLastFrame);

    TileSelectionStats fallbackStats;
    _impl->fallbackDrainPassesLastFrame = 0;
    if (fallbackResult.has_value()) {
        fallbackStats = countTerrainTileSelection(
            fallbackResult->tilesToRenderThisFrame, renderer, requireImageryForDisplay);
        maybeRelaxImplicitRootStuckSse(
            _impl->fallbackTileset.get(),
            fallbackResult->tilesToRenderThisFrame,
            fallbackStats.renderContentCount,
            _impl->fallbackStuckImplicitRootFrames);
        drainMainThreadUntilDrawable(
            _impl->fallbackTileset.get(),
            fallbackResult->tilesToRenderThisFrame,
            renderer,
            requireImageryForDisplay,
            fallbackStats,
            _impl->fallbackDrainPassesLastFrame);
    }

    if (_impl->renderer) {
        _impl->renderer->recoverStuckOverlayUniforms(kStuckOverlayFrameThreshold);

        const size_t selectedCountForPrune =
            primaryStats.selectedCount + fallbackStats.selectedCount;
        if (osg::Group* const tileGroup = _impl->tileGroup.get()) {
            const unsigned sceneChildren = tileGroup->getNumChildren();
            if (sceneChildren > selectedCountForPrune + kPruneChildMargin) {
                if (_impl->pruneCooldownFrames <= 0) {
                    _impl->renderer->pruneSceneNodesWithoutHandles();
                    _impl->pruneCooldownFrames = kPruneRateLimitFrames;
                } else {
                    --_impl->pruneCooldownFrames;
                }
            } else if (_impl->pruneCooldownFrames > 0) {
                --_impl->pruneCooldownFrames;
            }
        }
    }

    const size_t selectedCount = primaryStats.selectedCount + fallbackStats.selectedCount;
    const size_t renderContentCount =
        primaryStats.renderContentCount + fallbackStats.renderContentCount;
    const size_t renderReadyCount = primaryStats.renderReadyCount + fallbackStats.renderReadyCount;

    // Defer user 3D Tiles until at least one selected tile has mesh content on the main thread.
    // Imagery readiness is enforced only for OSG display (tryShowTile); do not block the whole
    // app on TMS/imagery or profile elevation sampling stalls when mesh already exists.
    const bool activelyLoading = renderContentCount == 0 && selectedCount > 0;

    updateDeferUserTilesetsState(
        activelyLoading, _impl->deferUserTilesets, _impl->terrainStableFrames);
    applyUserTilesetLoadDeferPolicy(_impl->deferUserTilesets);
    setHyperTerrainLoadingActive(_impl->deferUserTilesets);

    if (_impl->renderer &&
        (getLogTerrainRequest() || getLogTmsRequest()) &&
        ++_impl->diagFrameCounter >= kDiagLogIntervalFrames) {
        _impl->diagFrameCounter = 0;
        const unsigned sceneChildren = _impl->tileGroup.valid()
            ? _impl->tileGroup->getNumChildren()
            : 0u;
        std::ostringstream oss;
        oss << "[HyperTerrain] diag sceneChildren=" << sceneChildren
            << " aliveHandles=" << _impl->renderer->aliveHandleCount()
            << " rasterAttachments=" << _impl->renderer->rasterAttachmentCount()
            << " selected=" << selectedCount
            << " renderContent=" << (primaryStats.renderContentCount + fallbackStats.renderContentCount)
            << " pendingImagery="
            << (primaryStats.pendingImageryCount + fallbackStats.pendingImageryCount)
            << " renderReady=" << renderReadyCount
            << " drainPasses="
            << (_impl->primaryDrainPassesLastFrame + _impl->fallbackDrainPassesLastFrame);
        OSG_NOTICE << oss.str() << std::endl;
    }

    // Hide every OSG tile under _tileGroup; tryShowTile re-enables the current selection only.
    // Full sweep (not lastVisible diff) so tiles skipped by ion/dual/imagery gates cannot zombie-draw.
    if (osg::Group* const tileGroup = _impl->tileGroup.get()) {
        const unsigned numChildren = tileGroup->getNumChildren();
        for (unsigned i = 0; i < numChildren; ++i) {
            tileGroup->getChild(i)->setNodeMask(0x0);
        }
    }

    std::vector<PrimaryCoverageTile> primaryCoverage;
    appendPrimaryCoverageFromTiles(
        primaryResult.tilesToRenderThisFrame,
        CesiumEllipsoid::WGS84,
        dualTerrain,
        primaryCoverage);

    auto tryShowTile = [this, requireImageryForDisplay](const Tile::ConstPointer& tilePtr) {
        if (!tilePtr || !_impl->renderer)
            return false;
        osg::MatrixTransform* xform = nullptr;
        if (!_impl->renderer->tryResolveTileAttachedXform(*tilePtr, xform) || !xform)
            return false;
        // Mesh is created before TMS attachRasterInMainThread; keep NodeMask off until imagery is live.
        if (requireImageryForDisplay &&
            !_impl->renderer->tileImageryReadyForDisplay(*tilePtr)) {
            return false;
        }
        xform->setNodeMask(~0u);
        return true;
    };

    for (const auto& tilePtr : primaryResult.tilesToRenderThisFrame) {
        if (dualTerrain && tilePtr && isUpsampledPrimaryTile(*tilePtr))
            continue;
        if (ionLevelGate && tilePtr) {
            const uint32_t level = quadtreeLevelFromTile(*tilePtr).value_or(0u);
            if (level <= static_cast<uint32_t>(ionAlwaysOnMax))
                continue;
        }
        tryShowTile(tilePtr);
    }

    if (fallbackResult.has_value()) {
        for (const auto& tilePtr : fallbackResult->tilesToRenderThisFrame) {
            if (!tilePtr)
                continue;
            const auto bv = Cesium3DTilesSelection::transformBoundingVolume(
                tilePtr->getTransform(), tilePtr->getBoundingVolume());
            const auto fbGlobe = Cesium3DTilesSelection::estimateGlobeRectangle(bv, CesiumEllipsoid::WGS84);
            const uint32_t fbLevel = quadtreeLevelFromTile(*tilePtr).value_or(0u);
            if (ionLevelGate && fbLevel <= static_cast<uint32_t>(ionAlwaysOnMax)) {
                tryShowTile(tilePtr);
                continue;
            }
            if (primaryCoversFallbackTile(fbGlobe, fbLevel, primaryCoverage))
                continue;
            tryShowTile(tilePtr);
        }
    }
}

void HyperTerrainBridge::addImageryOverlay(RasterOverlayPtr overlay)
{
    if (!overlay)
        return;
    ++_impl->registeredImageryOverlayCount;
    if (_impl->primaryTileset)
        _impl->primaryTileset->getOverlays().add(overlay);
    if (_impl->fallbackTileset)
        _impl->fallbackTileset->getOverlays().add(overlay);
}

void HyperTerrainBridge::removeImageryOverlay(RasterOverlayPtr overlay)
{
    if (!overlay)
        return;
    _impl->registeredImageryOverlayCount =
        std::max(0, _impl->registeredImageryOverlayCount - 1);
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
