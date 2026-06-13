/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
 * MIT License
 */
#include "HyperTerrainBridge"
#include "AssetAccessor"
#include "HyperTerrainPrepareRendererResources"
#include "HyperTerrainImageryFactory"
#include "HyperTerrainLitShader"
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
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <sstream>
#include <unordered_set>
#include <vector>

using namespace osgEarth::Cesium;

namespace {

// Match CesiumLayer default and CesiumTilesetNode server key (CesiumIon::getContext map).
constexpr const char* kDefaultCesiumIonServer = "https://api.cesium.com/";

constexpr size_t kPruneChildMargin = 32;
constexpr size_t kLruExtraKeepMargin = 64;
constexpr int kPruneRateLimitFrames = 30;
constexpr int kDiagLogIntervalFrames = 300;
constexpr uint32_t kStuckOverlayFrameThreshold = 1;
constexpr int kDrainMaxPassesMeshPending = 12;
constexpr int kDrainMaxPassesImageryPending = 3;
constexpr int kDrainStalePassLimit = 2;
constexpr int kImplicitRootStuckSseRelaxFrames = 6;
constexpr double kImplicitRootStuckSseRelaxScale = 0.5;
constexpr const char* kHyperTerrainPerfLogPath = "logs/hyperterrain-perf.log";
constexpr double kRadiansToDegrees = 57.295779513082320876;
constexpr size_t kTileLoadStateBucketCount = 8;

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

struct ViewportDimensions {
    double width = 1280.0;
    double height = 720.0;
};

ViewportDimensions resolveViewStateViewport(const HyperTerrainViewParams& p)
{
    if (p.haveViewPixelDims && p.viewPixelWidth >= 1.0 && p.viewPixelHeight >= 1.0) {
        return {p.viewPixelWidth, p.viewPixelHeight};
    }
    if (p.viewportWidth >= 1.0 && p.viewportHeight >= 1.0) {
        return {p.viewportWidth, p.viewportHeight};
    }
    return {};
}

Cesium3DTilesSelection::ViewState buildViewState(const HyperTerrainViewParams& p)
{
    const glm::dvec3 position(p.eyeX, p.eyeY, p.eyeZ);
    const glm::dvec3 direction(p.dirX, p.dirY, p.dirZ);
    const glm::dvec3 up(p.upX, p.upY, p.upZ);
    const ViewportDimensions viewport = resolveViewStateViewport(p);
    const glm::dvec2 viewportSize(viewport.width, viewport.height);
    const double vFov = p.vFovRad;
    const double hFov =
        (p.hFovRad > 0.0) ? p.hFovRad : (vFov * (viewport.width / viewport.height));

    return Cesium3DTilesSelection::ViewState(
        position, direction, up, viewportSize, hFov, vFov);
}

struct TileSelectionStats {
    size_t selectedCount = 0;
    size_t renderContentCount = 0;
    size_t renderReadyCount = 0;
    size_t pendingImageryCount = 0;
    std::array<size_t, kTileLoadStateBucketCount> tileLoadStateBuckets{};
    size_t tileLoadStateOtherCount = 0;
};

TileSelectionStats countTerrainTileSelection(
    const std::vector<Tile::ConstPointer>& tiles,
    const HyperTerrainPrepareRendererResources* renderer,
    bool requireImageryForDisplay)
{
    TileSelectionStats stats;
    stats.selectedCount = tiles.size();
    for (const auto& tilePtr : tiles) {
        if (!tilePtr) {
            continue;
        }
        const int state = static_cast<int>(tilePtr->getState());
        if (state >= 0 && static_cast<size_t>(state) < stats.tileLoadStateBuckets.size()) {
            ++stats.tileLoadStateBuckets[static_cast<size_t>(state)];
        } else {
            ++stats.tileLoadStateOtherCount;
        }
        if (!renderer) {
            continue;
        }
        if (!tilePtr->getContent().isRenderContent()) {
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

std::string tileLoadStateBucketsToString(const TileSelectionStats& stats)
{
    std::ostringstream oss;
    for (size_t i = 0; i < stats.tileLoadStateBuckets.size(); ++i) {
        if (i > 0) {
            oss << ',';
        }
        oss << i << ':' << stats.tileLoadStateBuckets[i];
    }
    oss << ",other:" << stats.tileLoadStateOtherCount;
    return oss.str();
}

void applyMainThreadRebuildBudget(
    Cesium3DTilesSelection::Tileset* tileset,
    double& rebuildBudgetCredit)
{
    if (!tileset) {
        return;
    }

    Cesium3DTilesSelection::TilesetOptions& options = tileset->getOptions();
    // Re-applied per frame so runtime settings changes take effect; eviction work itself is
    // bounded by cesium-native's tileCacheUnloadTimeLimit and skips tiles selected this frame.
    options.maximumCachedBytes = getTerrainMaximumCachedBytes();
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
    double configuredMaximumScreenSpaceError,
    int& stuckImplicitRootFrames,
    const char* tilesetLabel)
{
    if (!tileset) {
        return;
    }

    bool stuckOnBlankRoot = false;
    if (tilesToRender.size() == 1 && renderContentCount == 0 && tilesToRender.front()) {
        const TileID& tileId = tilesToRender.front()->getTileID();
        if (const auto* quadId = std::get_if<QuadtreeTileID>(&tileId)) {
            stuckOnBlankRoot = (quadId->level == 0);
        }
    }

    Cesium3DTilesSelection::TilesetOptions& opts = tileset->getOptions();
    const char* const label = tilesetLabel ? tilesetLabel : "tileset";

    if (!stuckOnBlankRoot) {
        stuckImplicitRootFrames = 0;
        // Keep emergency SSE relaxation temporary; leaving it active makes later
        // views over-select deep LOD and can starve new tile loads.
        if (opts.maximumScreenSpaceError != configuredMaximumScreenSpaceError) {
            if (getLogHyperTerrainDiag()) {
                OSG_NOTICE << "[HyperTerrain] restored " << label
                           << " maxSse=" << configuredMaximumScreenSpaceError
                           << " from " << opts.maximumScreenSpaceError
                           << std::endl;
            }
            opts.maximumScreenSpaceError = configuredMaximumScreenSpaceError;
        }
        return;
    }

    if (++stuckImplicitRootFrames >= kImplicitRootStuckSseRelaxFrames) {
        const double relaxedMaxSse = std::min(
            configuredMaximumScreenSpaceError,
            std::max(1.0, configuredMaximumScreenSpaceError * kImplicitRootStuckSseRelaxScale));
        if (opts.maximumScreenSpaceError != relaxedMaxSse) {
            if (getLogHyperTerrainDiag()) {
                OSG_WARN << "[HyperTerrain] relaxing " << label
                         << " maxSse=" << opts.maximumScreenSpaceError
                         << " -> " << relaxedMaxSse
                         << " after implicit root stuck"
                         << std::endl;
            }
            opts.maximumScreenSpaceError = relaxedMaxSse;
        }
        stuckImplicitRootFrames = 0;
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

    for (int pass = 0; pass < maxPasses && stats.selectedCount > stats.renderReadyCount;
         ++pass) {
        ++outPassesUsed;
        tileset->getAsyncSystem().dispatchMainThreadTasks();
        stats = countTerrainTileSelection(tilesToRender, renderer, requireImageryForDisplay);
        if (stats.renderContentCount == 0) {
            break;
        }
        if (requireImageryForDisplay && stats.pendingImageryCount >= stats.renderContentCount &&
            stats.renderReadyCount == 0) {
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

void collectSelectionXforms(
    const std::vector<Tile::ConstPointer>& tiles,
    const HyperTerrainPrepareRendererResources* renderer,
    std::unordered_set<osg::MatrixTransform*>& outXforms)
{
    if (!renderer) {
        return;
    }
    for (const auto& tilePtr : tiles) {
        if (!tilePtr) {
            continue;
        }
        osg::MatrixTransform* xform = nullptr;
        if (renderer->tryResolveTileAttachedXform(*tilePtr, xform) && xform) {
            outXforms.insert(xform);
        }
    }
}

void collectSelectionStateSets(
    const std::vector<Tile::ConstPointer>& tiles,
    const HyperTerrainPrepareRendererResources* renderer,
    std::vector<osg::StateSet*>& outStateSets)
{
    if (!renderer) {
        return;
    }
    for (const auto& tilePtr : tiles) {
        if (!tilePtr) {
            continue;
        }
        HyperTerrainTileRenderData renderData;
        if (!renderer->tryResolveTileRenderData(*tilePtr, renderData) || !renderData.geom.valid()) {
            continue;
        }
        osg::StateSet* ss = renderData.geom->getStateSet();
        if (ss) {
            outStateSets.push_back(ss);
        }
    }
}

size_t finalizeFailedRasterOverlaysForSelection(
    HyperTerrainPrepareRendererResources* renderer,
    const std::vector<Tile::ConstPointer>& tiles)
{
    if (!renderer) {
        return 0;
    }
    size_t finalized = 0;
    for (const auto& tilePtr : tiles) {
        if (!tilePtr) {
            continue;
        }
        finalized += renderer->finalizeFailedRasterOverlaysForTile(
            const_cast<Tile&>(*tilePtr.get()));
    }
    return finalized;
}

size_t sceneChildTargetMax(const size_t selectedCount)
{
    return selectedCount + kPruneChildMargin + kLruExtraKeepMargin;
}

void appendHyperTerrainPerfDiagFile(const std::string& line)
{
    static std::mutex s_fileMutex;
    std::lock_guard<std::mutex> lock(s_fileMutex);

    std::error_code ec;
    std::filesystem::create_directories("logs", ec);

    std::ofstream out(kHyperTerrainPerfLogPath, std::ios::out | std::ios::app);
    if (!out) {
        return;
    }
    out << line << '\n';
}

void emitHyperTerrainPerfDiag(const std::string& line)
{
    const std::string stampedLine = elapsedLogStamp() + " " + line;
    // stderr: visible in VS Output / console regardless of Qt Log dock or osg notify level.
    std::fputs(stampedLine.c_str(), stderr);
    std::fputc('\n', stderr);
    std::fflush(stderr);
    appendHyperTerrainPerfDiagFile(stampedLine);
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
    /// Init-time SSE; maybeRelaxImplicitRootStuckSse restores to this once the root unsticks.
    double configuredMaximumScreenSpaceError = 16.0;
    double primaryRebuildBudgetCredit = 0.0;
    double fallbackRebuildBudgetCredit = 0.0;
    int primaryStuckImplicitRootFrames = 0;
    int fallbackStuckImplicitRootFrames = 0;
    int pruneCooldownFrames = 0;
    int diagFrameCounter = 0;
    int diagPerfFrameCounter = 0;
    int primaryDrainPassesLastFrame = 0;
    int fallbackDrainPassesLastFrame = 0;
    /// App-registered TMS overlays (Mapbox/VWorld/Naver/OWM); when zero, show mesh without imagery gate.
    int registeredImageryOverlayCount = 0;
    std::unordered_set<const CesiumRasterOverlays::RasterOverlay*> activeOverlays;
    /// When false, show mesh as soon as selected (dual-terrain NodeMask compositing unchanged).
    bool waitForTmsImageryBeforeDisplay = true;
    /// While true, user 3D Tiles layers defer cull/update and drape work for terrain priority.
    bool deferUserTilesets = true;
    int terrainStableFrames = 0;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> lastVisibleXforms;
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
    tilesetOptions.maximumCachedBytes = getTerrainMaximumCachedBytes();
    _impl->configuredMaximumScreenSpaceError = options.maximumScreenSpaceError;

    _impl->primaryTileset.reset();
    _impl->fallbackTileset.reset();
    _impl->cesiumIonTerrainAlwaysOnLevelMax = options.cesiumIonTerrainAlwaysOnLevelMax;
    _impl->waitForTmsImageryBeforeDisplay = options.waitForTmsImageryBeforeDisplay;

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
    const bool requireImageryForDisplay =
        _impl->waitForTmsImageryBeforeDisplay && _impl->registeredImageryOverlayCount > 0;

    TileSelectionStats primaryStats = countTerrainTileSelection(
        primaryResult.tilesToRenderThisFrame, renderer, requireImageryForDisplay);
    maybeRelaxImplicitRootStuckSse(
        _impl->primaryTileset.get(),
        primaryResult.tilesToRenderThisFrame,
        primaryStats.renderContentCount,
        _impl->configuredMaximumScreenSpaceError,
        _impl->primaryStuckImplicitRootFrames,
        "primary");
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
            _impl->configuredMaximumScreenSpaceError,
            _impl->fallbackStuckImplicitRootFrames,
            "fallback");
        drainMainThreadUntilDrawable(
            _impl->fallbackTileset.get(),
            fallbackResult->tilesToRenderThisFrame,
            renderer,
            requireImageryForDisplay,
            fallbackStats,
            _impl->fallbackDrainPassesLastFrame);
    }

    const size_t selectedCount = primaryStats.selectedCount + fallbackStats.selectedCount;
    const size_t renderContentCount =
        primaryStats.renderContentCount + fallbackStats.renderContentCount;
    const size_t renderReadyCount = primaryStats.renderReadyCount + fallbackStats.renderReadyCount;
    const bool activelyLoading = selectedCount > renderReadyCount;

    size_t prunedOrphans = 0;
    size_t detachedStale = 0;
    size_t overlayRecovered = 0;
    size_t finalizedOverlays = 0;

    if (_impl->renderer) {
        if (requireImageryForDisplay) {
            finalizedOverlays += finalizeFailedRasterOverlaysForSelection(
                _impl->renderer.get(), primaryResult.tilesToRenderThisFrame);
            if (fallbackResult.has_value()) {
                finalizedOverlays += finalizeFailedRasterOverlaysForSelection(
                    _impl->renderer.get(), fallbackResult->tilesToRenderThisFrame);
            }
        }

        std::vector<osg::StateSet*> selectionStateSets;
        selectionStateSets.reserve(selectedCount);
        collectSelectionStateSets(primaryResult.tilesToRenderThisFrame, renderer, selectionStateSets);
        if (fallbackResult.has_value()) {
            collectSelectionStateSets(
                fallbackResult->tilesToRenderThisFrame, renderer, selectionStateSets);
        }

        overlayRecovered = selectionStateSets.empty()
            ? 0
            : _impl->renderer->recoverStuckOverlayUniformsForStateSets(
                  selectionStateSets, kStuckOverlayFrameThreshold);

        const size_t targetMaxChildren = sceneChildTargetMax(selectedCount);

        std::unordered_set<osg::MatrixTransform*> protectedXforms;
        protectedXforms.reserve(selectedCount);
        collectSelectionXforms(primaryResult.tilesToRenderThisFrame, renderer, protectedXforms);
        if (fallbackResult.has_value()) {
            collectSelectionXforms(
                fallbackResult->tilesToRenderThisFrame, renderer, protectedXforms);
        }

        if (osg::Group* const tileGroup = _impl->tileGroup.get()) {
            const unsigned sceneChildren = tileGroup->getNumChildren();
            if (sceneChildren > targetMaxChildren) {
                if (_impl->pruneCooldownFrames <= 0) {
                    prunedOrphans = _impl->renderer->pruneSceneNodesWithoutHandles();
                    detachedStale = _impl->renderer->detachStaleSceneNodesFromRoot(
                        protectedXforms, targetMaxChildren);
                    _impl->pruneCooldownFrames = kPruneRateLimitFrames;
                } else {
                    --_impl->pruneCooldownFrames;
                }
            } else if (_impl->pruneCooldownFrames > 0) {
                --_impl->pruneCooldownFrames;
            }
        }
    }

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
            << " renderContent=" << renderContentCount
            << " pendingImagery="
            << (primaryStats.pendingImageryCount + fallbackStats.pendingImageryCount)
            << " renderReady=" << renderReadyCount
            << " drainPasses="
            << (_impl->primaryDrainPassesLastFrame + _impl->fallbackDrainPassesLastFrame);
        OSG_NOTICE << oss.str() << std::endl;
    }

    for (const osg::ref_ptr<osg::MatrixTransform>& xformRef : _impl->lastVisibleXforms) {
        osg::MatrixTransform* xform = xformRef.get();
        if (xform) {
            xform->setNodeMask(0x0);
        }
    }
    _impl->lastVisibleXforms.clear();

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
            xform = nullptr;
        if (!xform) {
            HyperTerrainTileRenderData renderData;
            if (!_impl->renderer->tryResolveTileRenderData(*tilePtr, renderData) || !renderData.xform.valid())
                return false;
            xform = renderData.xform.get();
        }
        if (!xform || !_impl->renderer->ensureTileXformAttached(xform))
            return false;
        // Mesh is created before TMS attachRasterInMainThread; keep NodeMask off until imagery is live.
        // Failed TMS overlays are finalized earlier; proceed with any imagery that is ready.
        if (requireImageryForDisplay &&
            !_impl->renderer->tileImageryReadyForDisplay(*tilePtr)) {
            return false;
        }
        xform->setNodeMask(~0u);
        _impl->lastVisibleXforms.emplace_back(xform);
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

    if (_impl->renderer && getLogHyperTerrainDiag() &&
        ++_impl->diagPerfFrameCounter >= kDiagLogIntervalFrames) {
        _impl->diagPerfFrameCounter = 0;
        const unsigned sceneChildren = _impl->tileGroup.valid()
            ? _impl->tileGroup->getNumChildren()
            : 0u;
        const uint32_t mainThreadBudget = _impl->primaryTileset
            ? _impl->primaryTileset->getOptions().maximumMainThreadTilesPerLoadPass
            : 0u;
        const uint32_t fallbackMainThreadBudget = _impl->fallbackTileset
            ? _impl->fallbackTileset->getOptions().maximumMainThreadTilesPerLoadPass
            : 0u;
        const double primaryMaxSse = _impl->primaryTileset
            ? _impl->primaryTileset->getOptions().maximumScreenSpaceError
            : 0.0;
        const double fallbackMaxSse = _impl->fallbackTileset
            ? _impl->fallbackTileset->getOptions().maximumScreenSpaceError
            : 0.0;
        const ViewportDimensions viewStateViewport = resolveViewStateViewport(viewParams);
        const HyperTerrainAssetRequestStats requestStats = hyperTerrainAssetRequestStats();
        std::ostringstream oss;
        oss << "[HyperTerrain] perf sceneChildren=" << sceneChildren
            << " targetMax=" << sceneChildTargetMax(selectedCount)
            << " aliveHandles=" << _impl->renderer->aliveHandleCount()
            << " rasterAttachments=" << _impl->renderer->rasterAttachmentCount()
            << " selected=" << selectedCount
            << " renderContent=" << renderContentCount
            << " pendingImagery="
            << (primaryStats.pendingImageryCount + fallbackStats.pendingImageryCount)
            << " renderReady=" << renderReadyCount
            << " visible=" << _impl->lastVisibleXforms.size()
            << " drainPasses="
            << (_impl->primaryDrainPassesLastFrame + _impl->fallbackDrainPassesLastFrame)
            << " primarySel=" << primaryStats.selectedCount
            << " primaryContent=" << primaryStats.renderContentCount
            << " primaryPendingImagery=" << primaryStats.pendingImageryCount
            << " primaryReady=" << primaryStats.renderReadyCount
            << " primaryTileStates=" << tileLoadStateBucketsToString(primaryStats)
            << " fallbackSel=" << fallbackStats.selectedCount
            << " fallbackContent=" << fallbackStats.renderContentCount
            << " fallbackPendingImagery=" << fallbackStats.pendingImageryCount
            << " fallbackReady=" << fallbackStats.renderReadyCount
            << " fallbackTileStates=" << tileLoadStateBucketsToString(fallbackStats)
            << " terrainReq=" << requestStats.terrainStarted
            << "/" << requestStats.terrainCompleted
            << "/" << requestStats.terrainOk
            << "/" << requestStats.terrainNotFound
            << "/" << requestStats.terrainError
            << "/" << requestStats.terrainCancelled
            << " terrainReqActive="
            << (requestStats.terrainStarted >= requestStats.terrainCompleted
                    ? requestStats.terrainStarted - requestStats.terrainCompleted
                    : 0)
            << " terrainReqBytes=" << requestStats.terrainBytes
            << " terrainReqLast=" << requestStats.lastTerrainStatus
            << "/" << requestStats.lastTerrainBytes
            << " imageryReq=" << requestStats.imageryStarted
            << "/" << requestStats.imageryCompleted
            << "/" << requestStats.imageryOk
            << "/" << requestStats.imageryNotFound
            << "/" << requestStats.imageryError
            << "/" << requestStats.imageryCancelled
            << " imageryReqActive="
            << (requestStats.imageryStarted >= requestStats.imageryCompleted
                    ? requestStats.imageryStarted - requestStats.imageryCompleted
                    : 0)
            << " imageryReqBytes=" << requestStats.imageryBytes
            << " imageryReqLast=" << requestStats.lastImageryStatus
            << "/" << requestStats.lastImageryBytes
            << " terrainRebuildRate=" << terrainRebuildsPerFrameRate()
            << " mainThreadBudget=" << mainThreadBudget << "/" << fallbackMainThreadBudget
            << " budgetCredit=" << _impl->primaryRebuildBudgetCredit
            << "/" << _impl->fallbackRebuildBudgetCredit
            << " maxSse=" << primaryMaxSse << "/" << fallbackMaxSse
            << " configuredMaxSse=" << _impl->configuredMaximumScreenSpaceError
            << " stuckRootFrames=" << _impl->primaryStuckImplicitRootFrames
            << "/" << _impl->fallbackStuckImplicitRootFrames
            << " cullViewport=" << viewParams.viewportWidth << "x" << viewParams.viewportHeight
            << " appViewport="
            << (viewParams.haveViewPixelDims ? viewParams.viewPixelWidth : 0.0)
            << "x"
            << (viewParams.haveViewPixelDims ? viewParams.viewPixelHeight : 0.0)
            << " viewStateViewport=" << viewStateViewport.width << "x" << viewStateViewport.height
            << " fovDeg=" << (viewParams.hFovRad * kRadiansToDegrees)
            << "/" << (viewParams.vFovRad * kRadiansToDegrees)
            << " waitForTmsImagery=" << (_impl->waitForTmsImageryBeforeDisplay ? 1 : 0)
            << " requireImagery=" << (requireImageryForDisplay ? 1 : 0)
            << " activeOverlays=" << _impl->registeredImageryOverlayCount
            << " deferUserTilesets=" << (_impl->deferUserTilesets ? 1 : 0)
            << " activelyLoading=" << (activelyLoading ? 1 : 0)
            << " prunedOrphans=" << prunedOrphans
            << " detachedStale=" << detachedStale
            << " overlayRecovered=" << overlayRecovered
            << " finalizedOverlays=" << finalizedOverlays;
        emitHyperTerrainPerfDiag(oss.str());
    }
}

void HyperTerrainBridge::addImageryOverlay(RasterOverlayPtr overlay, int compositingSlot)
{
    if (!overlay)
        return;
    const bool newlyActive = _impl->activeOverlays.insert(overlay.get()).second;
    int slot = compositingSlot;
    if (slot < 0) {
        slot = _impl->registeredImageryOverlayCount;
        ++_impl->registeredImageryOverlayCount;
    } else {
        slot = std::max(0, std::min(slot, 3));
    }
    if (_impl->renderer)
        _impl->renderer->registerImageryOverlaySlot(overlay.get(), slot);
    if (!newlyActive)
        return;
    if (_impl->primaryTileset)
        _impl->primaryTileset->getOverlays().add(overlay);
    if (_impl->fallbackTileset)
        _impl->fallbackTileset->getOverlays().add(overlay);
    _impl->registeredImageryOverlayCount =
        static_cast<int>(_impl->activeOverlays.size());
}

void HyperTerrainBridge::removeImageryOverlay(RasterOverlayPtr overlay)
{
    if (!overlay)
        return;
    if (!_impl->activeOverlays.erase(overlay.get()))
        return;
    if (_impl->renderer)
        _impl->renderer->unregisterImageryOverlaySlot(overlay.get());
    if (_impl->primaryTileset)
        _impl->primaryTileset->getOverlays().remove(overlay);
    if (_impl->fallbackTileset)
        _impl->fallbackTileset->getOverlays().remove(overlay);
    _impl->registeredImageryOverlayCount =
        static_cast<int>(_impl->activeOverlays.size());
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

void HyperTerrainBridge::setTerrainBaseColor(const osg::Vec3f& rgb)
{
    if (_impl->renderer)
        _impl->renderer->setTerrainBaseColor(rgb);
    else
        setHyperTerrainBaseColor(rgb);
}

void HyperTerrainBridge::refreshLoadedTileRasterOverlays()
{
    auto refreshTileset = [](Cesium3DTilesSelection::Tileset* tileset) {
        if (!tileset)
            return;
        const Cesium3DTilesSelection::TilesetOptions& opts = tileset->getOptions();
        Cesium3DTilesSelection::RasterOverlayCollection& overlays = tileset->getOverlays();
        tileset->forEachLoadedTile([&](const Cesium3DTilesSelection::Tile& tile) {
            if (tile.getState() != Cesium3DTilesSelection::TileLoadState::Done)
                return;
            if (!tile.getContent().isRenderContent())
                return;
            Cesium3DTilesSelection::Tile& mutableTile =
                const_cast<Cesium3DTilesSelection::Tile&>(tile);
            overlays.updateTileOverlays(mutableTile, opts);
        });
    };

    refreshTileset(_impl->primaryTileset.get());
    refreshTileset(_impl->fallbackTileset.get());
}

bool HyperTerrainBridge::hasPrimaryTileset() const
{
    return _impl->primaryTileset != nullptr;
}
