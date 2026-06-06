// HyperTerrainImageryFactory.cpp
// VWorld / Naver / Mapbox / OWM → cesium-native RasterOverlay 팩토리
//
// UrlTemplateRasterOverlayOptions::minimumLevel / maximumLevel 은 osgEarth TMS와
// 같이 QuadtreeRasterOverlayTileProvider 에서 적용된다. minimum 미만·ideal 이
// maximum+2 초과면 해당 타일만 투명, 그 안에서는 sticky 로 maximum 근처 LOD 를 맞춘다.

#include "HyperTerrainImageryFactory"
#include "Settings"

#include <CesiumRasterOverlays/UrlTemplateRasterOverlay.h>
#include <CesiumRasterOverlays/TileMapServiceRasterOverlay.h>
#include <CesiumRasterOverlays/IonRasterOverlay.h>
#include <CesiumGeometry/Rectangle.h>
// RasterOverlayOptions is defined inside RasterOverlay.h
#include <CesiumRasterOverlays/RasterOverlay.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <mutex>
#include <string_view>
#include <vector>

namespace {

static std::mutex s_loggerMutex;
static std::shared_ptr<spdlog::logger> s_tilesetLogger;

bool containsIgnoreCase(std::string_view haystack, std::string_view needle)
{
    if (needle.empty())
        return true;
    auto it = std::search(
        haystack.begin(),
        haystack.end(),
        needle.begin(),
        needle.end(),
        [](char a, char b) {
            return std::tolower(static_cast<unsigned char>(a))
                == std::tolower(static_cast<unsigned char>(b));
        });
    return it != haystack.end();
}

bool isImageryTilesetLogMessage(std::string_view text)
{
    if (text.empty())
        return false;

    if (containsIgnoreCase(text, "image for tile")
        || containsIgnoreCase(text, "failed to load image")
        || containsIgnoreCase(text, "loading image")
        || containsIgnoreCase(text, "raster overlay")
        || containsIgnoreCase(text, " for image "))
    {
        return true;
    }

    if (containsIgnoreCase(text, "response code")
        && (containsIgnoreCase(text, ".png")
            || containsIgnoreCase(text, ".jpg")
            || containsIgnoreCase(text, ".jpeg")
            || containsIgnoreCase(text, ".webp")))
    {
        return true;
    }

    return false;
}

bool isTerrainTilesetLogMessage(std::string_view text)
{
    if (text.empty())
        return false;

    return containsIgnoreCase(text, "quantized mesh")
        || containsIgnoreCase(text, "layer.json")
        || containsIgnoreCase(text, ".terrain")
        || containsIgnoreCase(text, "terrain tile")
        || containsIgnoreCase(text, "upsampled");
}

bool shouldEmitCesiumTilesetLog(std::string_view text)
{
    const bool imagery = isImageryTilesetLogMessage(text);
    const bool terrain = isTerrainTilesetLogMessage(text);

    if (imagery)
        return osgEarth::Cesium::getLogTmsRequest();
    if (terrain)
        return osgEarth::Cesium::getLogTerrainRequest();

    // Unknown cesium_tileset messages must not leak through the other flag.
    return false;
}

class CategoryGatedLogger final : public spdlog::logger {
public:
    template <typename SinkIt>
    CategoryGatedLogger(std::string name, SinkIt begin, SinkIt end)
        : spdlog::logger(std::move(name), begin, end)
    {
    }

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        const std::string_view text(msg.payload.data(), msg.payload.size());
        if (!shouldEmitCesiumTilesetLog(text))
            return;
        spdlog::logger::sink_it_(msg);
    }
};

std::shared_ptr<spdlog::logger> createTilesetLoggerLocked()
{
    auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    std::vector<spdlog::sink_ptr> sinks{sink};

    auto logger = std::make_shared<CategoryGatedLogger>(
        "cesium_tileset",
        sinks.begin(),
        sinks.end());

    if (const auto& defaultLogger = spdlog::default_logger())
        logger->set_level(defaultLogger->level());

    return logger;
}

CesiumRasterOverlays::RasterOverlayOptions currentRasterOverlayOpts()
{
    CesiumRasterOverlays::RasterOverlayOptions opts;
    opts.maximumSimultaneousTileLoads =
        osgEarth::Cesium::getMaximumSimultaneousTmsLoads();
    return opts;
}

} // namespace

static std::atomic<float> s_tmsImageryExposure{
    osgEarth::Cesium::HyperTerrainImageryFactory::kDefaultTmsImageryExposure};

namespace osgEarth { namespace Cesium {

using CesiumRasterOverlays::UrlTemplateRasterOverlay;
using CesiumRasterOverlays::TileMapServiceRasterOverlay;
using CesiumRasterOverlays::IonRasterOverlay;
using CesiumRasterOverlays::UrlTemplateRasterOverlayOptions;
using CesiumUtility::IntrusivePointer;
using CesiumRasterOverlays::RasterOverlay;

void HyperTerrainImageryFactory::setTmsImageryExposure(float exposure)
{
    s_tmsImageryExposure.store(std::max(0.0f, exposure), std::memory_order_relaxed);
}

float HyperTerrainImageryFactory::tmsImageryExposure()
{
    return s_tmsImageryExposure.load(std::memory_order_relaxed);
}

void HyperTerrainImageryFactory::setImageryTileTransportDiagLogging(bool /*enabled*/)
{
    refreshTilesetLoggerLevel();
}

void HyperTerrainImageryFactory::refreshTilesetLoggerLevel()
{
    std::lock_guard<std::mutex> lk(s_loggerMutex);
    if (s_tilesetLogger)
        s_tilesetLogger->set_level(spdlog::default_logger()->level());
}

std::shared_ptr<spdlog::logger> HyperTerrainImageryFactory::getOrCreateTilesetLogger()
{
    std::lock_guard<std::mutex> lk(s_loggerMutex);
    if (!s_tilesetLogger)
        s_tilesetLogger = createTilesetLoggerLocked();
    return s_tilesetLogger;
}

// ---- VWorld ----

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldStreet(const std::string& apiKey)
{
    // VWorld TMS - Base (도로) 타일
    // URL 패턴: https://api.vworld.kr/req/wmts/1.0.0/{apiKey}/Base/{z}/{y}/{x}.png
    std::string url =
        "https://api.vworld.kr/req/wmts/1.0.0/" + apiKey +
        "/Base/{z}/{reverseY}/{x}.png";

    UrlTemplateRasterOverlayOptions opts;
    opts.maximumLevel = 19;

    return new UrlTemplateRasterOverlay("VWorldStreet", url, {}, opts, currentRasterOverlayOpts());
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldSatellite(const std::string& apiKey)
{
    std::string url =
        "https://api.vworld.kr/req/wmts/1.0.0/" + apiKey +
        "/Satellite/{z}/{reverseY}/{x}.jpeg";

    UrlTemplateRasterOverlayOptions opts;
    opts.maximumLevel = 19;

    return new UrlTemplateRasterOverlay("VWorldSatellite", url, {}, opts, currentRasterOverlayOpts());
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldHybridOverlay(const std::string& apiKey)
{
    std::string url =
        "https://api.vworld.kr/req/wmts/1.0.0/" + apiKey +
        "/Hybrid/{z}/{reverseY}/{x}.png";

    UrlTemplateRasterOverlayOptions opts;
    opts.maximumLevel = 19;

    return new UrlTemplateRasterOverlay("VWorldHybrid", url, {}, opts, currentRasterOverlayOpts());
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldLabel()
{
    // QGIS TMS for Korea / VWorld open 2D — Hybrid(라벨·도로명) 오버레이 타일.
    // https://xdworld.vworld.kr/2d/Hybrid/service/{z}/{x}/{y}.png
    const std::string url =
        "https://xdworld.vworld.kr/2d/Hybrid/service/{z}/{x}/{reverseY}.png";

    UrlTemplateRasterOverlayOptions opts;
    opts.minimumLevel = 7;
    opts.maximumLevel = 18;
    // Service is Korea-only; avoid pointless HTTP outside this bounds.
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    opts.coverageRectangle = CesiumGeometry::Rectangle(
        124.6 * kDegToRad,
        33.1 * kDegToRad,
        131.9 * kDegToRad,
        38.5 * kDegToRad);

    return new UrlTemplateRasterOverlay("VWorldLabel", url, {}, opts, currentRasterOverlayOpts());
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldGray(const std::string& apiKey)
{
    std::string url =
        "https://api.vworld.kr/req/wmts/1.0.0/" + apiKey +
        "/gray/{z}/{reverseY}/{x}.png";

    UrlTemplateRasterOverlayOptions opts;
    opts.maximumLevel = 18;

    return new UrlTemplateRasterOverlay("VWorldGray", url, {}, opts, currentRasterOverlayOpts());
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldMidnight(const std::string& apiKey)
{
    std::string url =
        "https://api.vworld.kr/req/wmts/1.0.0/" + apiKey +
        "/midnight/{z}/{reverseY}/{x}.png";

    UrlTemplateRasterOverlayOptions opts;
    opts.maximumLevel = 18;

    return new UrlTemplateRasterOverlay("VWorldMidnight", url, {}, opts, currentRasterOverlayOpts());
}

// ---- Bing Maps via Ion ----

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createBingMapsAerial(const std::string& ionAccessToken)
{
    return new IonRasterOverlay("BingMapsAerial", 2, ionAccessToken, currentRasterOverlayOpts());
}

// ---- Mapbox Satellite v9 ----
// Web Mercator (EPSG:3857), 레벨 0~22, x/y 표준 XYZ 순서

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createMapboxSatellite(const std::string& accessToken)
{
    if (accessToken.empty()) {
        return nullptr;
    }

    const std::string url =
        "https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{reverseY}"
        "?access_token=" + accessToken;

    UrlTemplateRasterOverlayOptions opts;
    opts.minimumLevel = 0;
    opts.maximumLevel = 22;

    return new UrlTemplateRasterOverlay("MapboxSatellite", url, {}, opts, currentRasterOverlayOpts());
}

// ---- Naver ----

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createNaverSatellite()
{
    // Naver 위성 슬라이드 타일
    // TODO: 실제 URL 및 인증 방식 확인 필요
    std::string url =
        "https://simg.pstatic.net/onetile/bl3/694/{z}/{reverseY}/{x}";

    UrlTemplateRasterOverlayOptions opts;
    opts.maximumLevel = 14;

    return new UrlTemplateRasterOverlay("NaverSatellite", url, {}, opts, currentRasterOverlayOpts());
}

// ---- OpenWeatherMap ----

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createOwmClouds(const std::string& apiKey)
{
    std::string url =
        "https://tile.openweathermap.org/map/clouds_new/{z}/{x}/{reverseY}.png"
        "?appid=" + apiKey;

    UrlTemplateRasterOverlayOptions opts;
    opts.maximumLevel = 5;
    opts.minimumLevel = 0;

    return new UrlTemplateRasterOverlay("OwmClouds", url, {}, opts, currentRasterOverlayOpts());
}

} // namespace Cesium
} // namespace osgEarth
