// HyperTerrainImageryFactory.cpp
// VWorld / Naver / Mapbox / OWM → cesium-native RasterOverlay 팩토리
//
// UrlTemplateRasterOverlayOptions::minimumLevel / maximumLevel 은 osgEarth TMS와
// 같이 QuadtreeRasterOverlayTileProvider 에서 적용된다. minimum 미만·ideal 이
// maximum+2 초과면 해당 타일만 투명, 그 안에서는 sticky 로 maximum 근처 LOD 를 맞춘다.
//
// VWorld는 전역 EPSG:3857 z/x/y 타일(한국 z=6~19)이다. coverageRectangle 만 주면
// cesium-native 가 한국 bbox 로 지역 쿼드트리를 만들어 URL 이 0/0/0 에 고정되므로
// 반드시 전역 WebMercator tilingScheme 을 명시한다.
// coverageRectangle 은 한국 밖 타일에 경계 픽셀을 늘려 붙이는(stretch) 글리치를 유발하므로
// 사용하지 않는다 — 한국 밖은 타일 HTTP 실패로 투명 처리.

#include "HyperTerrainImageryFactory"
#include "Settings"

#include <CesiumRasterOverlays/UrlTemplateRasterOverlay.h>
#include <CesiumRasterOverlays/TileMapServiceRasterOverlay.h>
#include <CesiumRasterOverlays/IonRasterOverlay.h>
#include <CesiumGeometry/Rectangle.h>
#include <CesiumGeometry/QuadtreeTilingScheme.h>
#include <CesiumGeospatial/GlobeRectangle.h>
#include <CesiumGeospatial/Projection.h>
#include <CesiumGeospatial/WebMercatorProjection.h>
// RasterOverlayOptions is defined inside RasterOverlay.h
#include <CesiumRasterOverlays/RasterOverlay.h>

#include <osgEarth/URI>

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

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr uint32_t kVWorldMinimumLevel = 6;
constexpr uint32_t kVWorldMaximumLevel = 19;

CesiumGeospatial::GlobeRectangle vworldKoreaGlobeRectangle()
{
    return CesiumGeospatial::GlobeRectangle(
        124.6 * kDegToRad,
        33.1 * kDegToRad,
        131.9 * kDegToRad,
        38.5 * kDegToRad);
}

CesiumGeometry::Rectangle vworldKoreaProjectedCoverageRectangle()
{
    const CesiumGeospatial::WebMercatorProjection projection;
    return CesiumGeospatial::projectRectangleSimple(
        projection,
        vworldKoreaGlobeRectangle());
}

CesiumGeometry::QuadtreeTilingScheme vworldGlobalWebMercatorTilingScheme()
{
    const CesiumGeospatial::WebMercatorProjection projection;
    const CesiumGeometry::Rectangle globeProjected =
        CesiumGeospatial::projectRectangleSimple(
            projection,
            CesiumGeospatial::WebMercatorProjection::MAXIMUM_GLOBE_RECTANGLE);
    return CesiumGeometry::QuadtreeTilingScheme(globeProjected, 1, 1);
}

void applyVworldRegionalOverlayOptions(
    CesiumRasterOverlays::UrlTemplateRasterOverlayOptions& opts)
{
    opts.minimumLevel = kVWorldMinimumLevel;
    opts.maximumLevel = kVWorldMaximumLevel;
    opts.tilingScheme = vworldGlobalWebMercatorTilingScheme();
}

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

    if (containsIgnoreCase(text, "req/wmts")
        || containsIgnoreCase(text, "api.vworld")
        || containsIgnoreCase(text, "xdworld.vworld")
        || containsIgnoreCase(text, "mapbox.com")
        || containsIgnoreCase(text, "map.pstatic")
        || containsIgnoreCase(text, "openweathermap"))
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

constexpr const char* kNaverSatelliteStyleJsonUrl =
    "https://map.pstatic.net/nrb/styles/satellite.json?fmt=png&mt=bg";
constexpr const char* kNaverSatelliteFallbackVersion = "1781263792";

std::string fetchNaverSatelliteTileVersion()
{
    osgEarth::URIContext uriContext;
    uriContext.addHeader("Referer", "https://map.naver.com/");
    uriContext.addHeader(
        "User-Agent",
        "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");

    const osgEarth::URI uri(kNaverSatelliteStyleJsonUrl, uriContext);
    const osgEarth::ReadResult result = uri.readString();
    if (result.code() != osgEarth::ReadResult::RESULT_OK) {
        spdlog::warn(
            "Naver satellite style JSON fetch failed (code={}); using fallback version {}",
            static_cast<int>(result.code()),
            kNaverSatelliteFallbackVersion);
        return kNaverSatelliteFallbackVersion;
    }

    const std::string& body = result.getString();
    static constexpr std::string_view kVersionKey = "\"version\":\"";
    const std::size_t pos = body.find(kVersionKey);
    if (pos == std::string::npos) {
        spdlog::warn(
            "Naver satellite style JSON missing version; using fallback {}",
            kNaverSatelliteFallbackVersion);
        return kNaverSatelliteFallbackVersion;
    }

    const std::size_t start = pos + kVersionKey.size();
    const std::size_t end = body.find('"', start);
    if (end == std::string::npos || end <= start) {
        spdlog::warn(
            "Naver satellite style JSON version parse failed; using fallback {}",
            kNaverSatelliteFallbackVersion);
        return kNaverSatelliteFallbackVersion;
    }

    return body.substr(start, end - start);
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

std::string HyperTerrainImageryFactory::normalizeVWorldApiKey(std::string apiKey)
{
    for (char& c : apiKey) {
        if (c == '%')
            c = '-';
    }
    const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    apiKey.erase(apiKey.begin(), std::find_if(apiKey.begin(), apiKey.end(), notSpace));
    apiKey.erase(std::find_if(apiKey.rbegin(), apiKey.rend(), notSpace).base(), apiKey.end());
    return apiKey;
}

// ---- VWorld ----

IntrusivePointer<RasterOverlay> makeVWorldWmtsOverlay(
    const char* overlayName,
    const std::string& apiKey,
    std::string url)
{
    if (apiKey.empty()) {
        spdlog::warn(
            "{}: empty API key (ExternalService/vworldApiKey or VWORLD_API_KEY)",
            overlayName);
        return nullptr;
    }

    UrlTemplateRasterOverlayOptions opts;
    applyVworldRegionalOverlayOptions(opts);

    return new UrlTemplateRasterOverlay(
        overlayName,
        std::move(url),
        {},
        opts,
        currentRasterOverlayOpts());
}

IntrusivePointer<RasterOverlay> makeVWorldXdworldOverlay(
    const char* overlayName,
    std::string url)
{
    UrlTemplateRasterOverlayOptions opts;
    applyVworldRegionalOverlayOptions(opts);

    return new UrlTemplateRasterOverlay(
        overlayName,
        std::move(url),
        {},
        opts,
        currentRasterOverlayOpts());
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldStreet(const std::string& /*apiKey*/)
{
    // xdworld open 2D Base — no API key (WMTS Base needs a registered vworld.kr key).
    const std::string url =
        "https://xdworld.vworld.kr/2d/Base/service/{z}/{x}/{reverseY}.png";
    return makeVWorldXdworldOverlay("VWorldStreet", url);
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldSatellite(const std::string& /*apiKey*/)
{
    const std::string url =
        "https://xdworld.vworld.kr/2d/Satellite/service/{z}/{x}/{reverseY}.jpeg";
    return makeVWorldXdworldOverlay("VWorldSatellite", url);
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldHybridOverlay(const std::string& apiKey)
{
    const std::string key = normalizeVWorldApiKey(apiKey);
    const std::string url =
        "https://api.vworld.kr/req/wmts/1.0.0/" + key
        + "/Hybrid/{z}/{reverseY}/{x}.png";
    return makeVWorldWmtsOverlay("VWorldHybrid", key, url);
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldLabel()
{
    // xdworld open 2D Hybrid (라벨·도로명). Slippy-map Y = cesium-native {reverseY}.
    // https://xdworld.vworld.kr/2d/Hybrid/service/{z}/{x}/{y}.png
    const std::string url =
        "https://xdworld.vworld.kr/2d/Hybrid/service/{z}/{x}/{reverseY}.png";

    UrlTemplateRasterOverlayOptions opts;
    applyVworldRegionalOverlayOptions(opts);

    return new UrlTemplateRasterOverlay("VWorldLabel", url, {}, opts, currentRasterOverlayOpts());
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldGray(const std::string& apiKey)
{
    const std::string key = normalizeVWorldApiKey(apiKey);
    const std::string url =
        "https://api.vworld.kr/req/wmts/1.0.0/" + key
        + "/gray/{z}/{reverseY}/{x}.png";
    return makeVWorldWmtsOverlay("VWorldGray", key, url);
}

IntrusivePointer<RasterOverlay>
HyperTerrainImageryFactory::createVWorldMidnight(const std::string& apiKey)
{
    const std::string key = normalizeVWorldApiKey(apiKey);
    const std::string url =
        "https://api.vworld.kr/req/wmts/1.0.0/" + key
        + "/midnight/{z}/{reverseY}/{x}.png";
    return makeVWorldWmtsOverlay("VWorldMidnight", key, url);
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
    // Naver Maps NRB satellite (Web Mercator XYZ). Version in the path changes periodically;
    // fetch satellite.json at layer creation time (same source as map.naver.com).
    const std::string version = fetchNaverSatelliteTileVersion();
    const std::string url =
        "https://map.pstatic.net/nrb/styles/satellite/" + version
        + "/{z}/{x}/{reverseY}.png?mt=bg";

    UrlTemplateRasterOverlayOptions opts;
    opts.minimumLevel = 0;
    opts.maximumLevel = 21;

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
