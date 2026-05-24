/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/

#include "Settings"
#include "CesiumIon"

#include <atomic>
#include <algorithm>

static std::string CESIUM_KEY = "";
static std::atomic<bool> s_log3DTilesHttpUrls{false};
static std::atomic<bool> s_cesiumShuttingDown{false};
static osgEarth::Cesium::TileRebuildsPerFrameFn s_tileRebuildsPerFrameProvider;

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

bool osgEarth::Cesium::getLog3DTilesHttpUrls()
{
    return s_log3DTilesHttpUrls.load(std::memory_order_relaxed);
}

void osgEarth::Cesium::setLog3DTilesHttpUrls(bool enabled)
{
    s_log3DTilesHttpUrls.store(enabled, std::memory_order_relaxed);
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