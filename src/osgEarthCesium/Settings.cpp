/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/

// This translation unit implements osgEarthCesium exports. Define the library
// macro before including Settings so declarations match definitions (MSVC C4273).
#if !defined(OSGEARTHCESIUM_LIBRARY_STATIC) && !defined(OSGEARTHCESIUM_LIBRARY)
#define OSGEARTHCESIUM_LIBRARY
#endif

#include "Settings"
#include "CesiumIon"
#include "HyperTerrainImageryFactory"

#include <osgEarth/CesiumIon>

#include <atomic>
#include <algorithm>

static std::string CESIUM_KEY = "";
static std::atomic<bool> s_log3DTilesRequest{false};
static std::atomic<bool> s_logTerrainRequest{false};
static std::atomic<bool> s_logTmsRequest{false};
static std::atomic<bool> s_logHyperTerrainDiag{false};
static std::atomic<bool> s_cesiumShuttingDown{false};
static osgEarth::Cesium::TileRebuildsPerFrameFn s_tileRebuildsPerFrameProvider;
static osgEarth::Cesium::TileRebuildsPerFrameFn s_terrainRebuildsPerFrameProvider;
static std::atomic<bool> s_hyperTerrainLoadingActive{true};
static std::atomic<double> s_userTilesetRebuildScaleWhileTerrainLoads{0.05};
static std::atomic<uint32_t> s_maximumSimultaneousTmsLoads{20};

namespace
{
    class ReadKey
    {
    public:
        ReadKey()
        {
            // Get the key from an environment variable
            const char* key = ::getenv("OSGEARTH_CESIUMION_KEY");
            if (key)
            {
                osgEarth::Cesium::setCesiumIonKey(std::string(key));
            }
        }
    };
}

static ReadKey s_READKEY;

std::string  osgEarth::Cesium::getCesiumIonKey()
{
    return CESIUM_KEY;
}

void osgEarth::Cesium::setCesiumIonKey(const std::string& key)
{
    CESIUM_KEY = key;
}

bool osgEarth::Cesium::getLog3DTilesRequest()
{
    return s_log3DTilesRequest.load(std::memory_order_relaxed);
}

void osgEarth::Cesium::setLog3DTilesRequest(bool enabled)
{
    s_log3DTilesRequest.store(enabled, std::memory_order_relaxed);
}

bool osgEarth::Cesium::getLogTerrainRequest()
{
    return s_logTerrainRequest.load(std::memory_order_relaxed);
}

void osgEarth::Cesium::setLogTerrainRequest(bool enabled)
{
    s_logTerrainRequest.store(enabled, std::memory_order_relaxed);
    CesiumIonTerrainMeshLayer::setLogTerrainHttpResponses(enabled);
    HyperTerrainImageryFactory::refreshTilesetLoggerLevel();
}

bool osgEarth::Cesium::getLogTmsRequest()
{
    return s_logTmsRequest.load(std::memory_order_relaxed);
}

void osgEarth::Cesium::setLogTmsRequest(bool enabled)
{
    s_logTmsRequest.store(enabled, std::memory_order_relaxed);
    HyperTerrainImageryFactory::refreshTilesetLoggerLevel();
}

bool osgEarth::Cesium::getLogHyperTerrainDiag()
{
    return s_logHyperTerrainDiag.load(std::memory_order_relaxed);
}

void osgEarth::Cesium::setLogHyperTerrainDiag(bool enabled)
{
    s_logHyperTerrainDiag.store(enabled, std::memory_order_relaxed);
}

void osgEarth::Cesium::setMaximumSimultaneousTmsLoads(uint32_t value)
{
    s_maximumSimultaneousTmsLoads.store(std::max(1u, value), std::memory_order_relaxed);
}

uint32_t osgEarth::Cesium::getMaximumSimultaneousTmsLoads()
{
    return s_maximumSimultaneousTmsLoads.load(std::memory_order_relaxed);
}

void osgEarth::Cesium::requestShutdown()
{
    s_cesiumShuttingDown.store(true, std::memory_order_release);
}

bool osgEarth::Cesium::isShuttingDown()
{
    return s_cesiumShuttingDown.load(std::memory_order_acquire);
}

void osgEarth::Cesium::shutdown()
{
    requestShutdown();
    CesiumIon::instance().shutdown();
}

void osgEarth::Cesium::setTileRebuildsPerFrameProvider(TileRebuildsPerFrameFn fn)
{
    s_tileRebuildsPerFrameProvider = std::move(fn);
}

double osgEarth::Cesium::tileRebuildsPerFrameRate()
{
    if (s_tileRebuildsPerFrameProvider) {
        return std::max(0.0, s_tileRebuildsPerFrameProvider());
    }
    return 0.25;
}

void osgEarth::Cesium::setTerrainRebuildsPerFrameProvider(TileRebuildsPerFrameFn fn)
{
    s_terrainRebuildsPerFrameProvider = std::move(fn);
}

double osgEarth::Cesium::terrainRebuildsPerFrameRate()
{
    if (s_terrainRebuildsPerFrameProvider) {
        return std::max(0.0, s_terrainRebuildsPerFrameProvider());
    }
    return 0.25;
}

void osgEarth::Cesium::setHyperTerrainLoadingActive(bool active)
{
    s_hyperTerrainLoadingActive.store(active, std::memory_order_relaxed);
}

bool osgEarth::Cesium::isHyperTerrainLoadingActive()
{
    return s_hyperTerrainLoadingActive.load(std::memory_order_relaxed);
}

void osgEarth::Cesium::setUserTilesetRebuildScaleWhileTerrainLoads(double scale)
{
    s_userTilesetRebuildScaleWhileTerrainLoads.store(
        std::clamp(scale, 0.0, 1.0), std::memory_order_relaxed);
}

double osgEarth::Cesium::getUserTilesetRebuildScaleWhileTerrainLoads()
{
    return s_userTilesetRebuildScaleWhileTerrainLoads.load(std::memory_order_relaxed);
}

double osgEarth::Cesium::userTilesetRebuildsPerFrameRate()
{
    if (isHyperTerrainLoadingActive()) {
        return 0.0;
    }
    return tileRebuildsPerFrameRate();
}

uint32_t osgEarth::Cesium::userTilesetMinMainThreadTilesPerPass()
{
    return isHyperTerrainLoadingActive() ? 0u : 4u;
}

int osgEarth::Cesium::userTilesetMaxDrainPassesWhileTerrainLoads()
{
    return isHyperTerrainLoadingActive() ? 2 : 12;
}