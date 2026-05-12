/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/

#include "Settings"
#include "CesiumIon"

#include <atomic>

static std::string CESIUM_KEY = "";
static std::atomic<bool> s_log3DTilesHttpUrls{false};

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

void osgEarth::Cesium::shutdown()
{
    CesiumIon::instance().shutdown();
}