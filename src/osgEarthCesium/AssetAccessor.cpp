/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "AssetAccessor"
#include "Settings"
#include "UserTilesetLoadGate"

#include <CesiumAsync/AsyncSystem.h>
#include <osgEarth/LocalTerrainFileStore>
#include <osgEarth/LocalTerrainUri>
#include <osgEarth/URI>
#include <osgEarth/Registry>
#include <osgEarth/Notify>

#include <osg/Notify>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#endif

#undef LC
#define LC "[HyperTerrain] "

using namespace osgEarth;
using namespace osgEarth::Cesium;

namespace {

enum class RequestKind {
    None,
    Terrain,
    Imagery,
};

struct RequestCounterSet {
    std::atomic<uint64_t> started{0};
    std::atomic<uint64_t> completed{0};
    std::atomic<uint64_t> ok{0};
    std::atomic<uint64_t> notFound{0};
    std::atomic<uint64_t> error{0};
    std::atomic<uint64_t> cancelled{0};
    std::atomic<uint64_t> bytes{0};
    std::atomic<uint64_t> lastBytes{0};
    std::atomic<uint16_t> lastStatus{0};
};

RequestCounterSet s_terrainRequestCounters;
RequestCounterSet s_imageryRequestCounters;
constexpr const char* kHyperTerrainRequestLogPath = "logs/hyperterrain-requests.log";

std::chrono::steady_clock::time_point resolveAppLogOrigin()
{
#ifdef _WIN32
    FILETIME creationTime;
    FILETIME exitTime;
    FILETIME kernelTime;
    FILETIME userTime;
    if (GetProcessTimes(GetCurrentProcess(), &creationTime, &exitTime, &kernelTime, &userTime)) {
        FILETIME nowFileTime;
        GetSystemTimeAsFileTime(&nowFileTime);

        ULARGE_INTEGER created;
        created.LowPart = creationTime.dwLowDateTime;
        created.HighPart = creationTime.dwHighDateTime;

        ULARGE_INTEGER now;
        now.LowPart = nowFileTime.dwLowDateTime;
        now.HighPart = nowFileTime.dwHighDateTime;

        if (now.QuadPart >= created.QuadPart) {
            using HundredNanoseconds = std::chrono::duration<long long, std::ratio<1, 10000000>>;
            const auto processAge = HundredNanoseconds(
                static_cast<long long>(now.QuadPart - created.QuadPart));
            return std::chrono::steady_clock::now()
                - std::chrono::duration_cast<std::chrono::steady_clock::duration>(processAge);
        }
    }
#endif
    return std::chrono::steady_clock::now();
}

const std::chrono::steady_clock::time_point s_appLogOrigin = resolveAppLogOrigin();

RequestCounterSet* countersForKind(RequestKind kind)
{
    switch (kind) {
    case RequestKind::Terrain:
        return &s_terrainRequestCounters;
    case RequestKind::Imagery:
        return &s_imageryRequestCounters;
    case RequestKind::None:
    default:
        return nullptr;
    }
}

void recordRequestStarted(RequestKind kind)
{
    if (RequestCounterSet* counters = countersForKind(kind)) {
        counters->started.fetch_add(1, std::memory_order_relaxed);
    }
}

void recordRequestCompleted(RequestKind kind, uint16_t statusCode, std::size_t bytes)
{
    RequestCounterSet* counters = countersForKind(kind);
    if (!counters) {
        return;
    }
    counters->completed.fetch_add(1, std::memory_order_relaxed);
    counters->bytes.fetch_add(static_cast<uint64_t>(bytes), std::memory_order_relaxed);
    counters->lastBytes.store(static_cast<uint64_t>(bytes), std::memory_order_relaxed);
    counters->lastStatus.store(statusCode, std::memory_order_relaxed);
    if (statusCode == 200) {
        counters->ok.fetch_add(1, std::memory_order_relaxed);
    } else if (statusCode == 404) {
        counters->notFound.fetch_add(1, std::memory_order_relaxed);
    } else if (statusCode == 499) {
        counters->cancelled.fetch_add(1, std::memory_order_relaxed);
    } else {
        counters->error.fetch_add(1, std::memory_order_relaxed);
    }
}

void copyCounterSet(const RequestCounterSet& src,
    uint64_t& started,
    uint64_t& completed,
    uint64_t& ok,
    uint64_t& notFound,
    uint64_t& error,
    uint64_t& cancelled,
    uint64_t& bytes,
    uint16_t& lastStatus,
    uint64_t& lastBytes)
{
    started = src.started.load(std::memory_order_relaxed);
    completed = src.completed.load(std::memory_order_relaxed);
    ok = src.ok.load(std::memory_order_relaxed);
    notFound = src.notFound.load(std::memory_order_relaxed);
    error = src.error.load(std::memory_order_relaxed);
    cancelled = src.cancelled.load(std::memory_order_relaxed);
    bytes = src.bytes.load(std::memory_order_relaxed);
    lastStatus = src.lastStatus.load(std::memory_order_relaxed);
    lastBytes = src.lastBytes.load(std::memory_order_relaxed);
}

std::string localPathForFileUrl(const std::string& url) {
    return LocalTerrainUri::fileUrlToNativePath(url);
}

bool tryReadLocalTerrainTile(
    const std::string& url,
    std::vector<std::byte>& outBytes)
{
    const auto bytes = LocalTerrainFileStore::readTerrainTileFromRequestUrl(url);
    if (!bytes || bytes->empty())
        return false;

    outBytes.assign(bytes->begin(), bytes->end());
    return true;
}

void warnTerrainTileMiss(
    const std::string& url,
    const std::filesystem::path& nativePath)
{
    if (!osgEarth::Cesium::getLogTerrainRequest())
        return;

    std::error_code ec;
    const bool exists = std::filesystem::is_regular_file(nativePath, ec);
    if (!exists)
        return;

    OE_WARN << osgEarth::Cesium::elapsedLogStamp()
            << " " << LC << "terrain tile read failed but file exists"
            << " url=" << url
            << " native=" << nativePath.generic_string()
            << std::endl;
}

bool isTerrainRequestUrl(const std::string& url)
{
    if (LocalTerrainUri::isLocalTerrainTileRequest(url))
        return true;
    const std::string pathOnly = LocalTerrainUri::stripQueryAndFragment(url);
    if (pathOnly.size() >= 8
        && pathOnly.compare(pathOnly.size() - 8, 8, ".terrain") == 0)
    {
        return true;
    }
    return pathOnly.find("layer.json") != std::string::npos;
}

bool isLikelyRasterImageryUrl(const std::string& url)
{
    const std::string path = LocalTerrainUri::stripQueryAndFragment(url);
    std::string lower = path;
    std::transform(
        lower.begin(),
        lower.end(),
        lower.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    return lower.find(".png") != std::string::npos
        || lower.find(".jpg") != std::string::npos
        || lower.find(".jpeg") != std::string::npos
        || lower.find(".webp") != std::string::npos
        || lower.find("/wmts/") != std::string::npos
        || lower.find("req/wmts") != std::string::npos
        || lower.find("xdworld.vworld") != std::string::npos;
}

void appendHyperTerrainRequestDiagFile(const std::string& line)
{
    static std::mutex s_fileMutex;
    std::lock_guard<std::mutex> lock(s_fileMutex);

    std::error_code ec;
    std::filesystem::create_directories("logs", ec);

    std::ofstream out(kHyperTerrainRequestLogPath, std::ios::out | std::ios::app);
    if (!out) {
        return;
    }
    out << line << '\n';
}

bool extractVWorldOwsException(const std::string& content, std::string& outMessage)
{
    if (content.size() < 20
        || content.compare(0, 5, "<?xml") != 0
        || content.find("ExceptionReport") == std::string::npos)
    {
        return false;
    }

    constexpr const char* kTag = "<ExceptionText><![CDATA[";
    const auto pos = content.find(kTag);
    if (pos != std::string::npos) {
        const auto start = pos + std::strlen(kTag);
        const auto end = content.find("]]>", start);
        if (end != std::string::npos)
            outMessage = content.substr(start, end - start);
    }
    return true;
}

bool is3DTilesRequestUrl(const std::string& url)
{
    if (isTerrainRequestUrl(url) || isLikelyRasterImageryUrl(url))
        return false;

    const std::string path = LocalTerrainUri::stripQueryAndFragment(url);
    std::string lower = path;
    std::transform(
        lower.begin(),
        lower.end(),
        lower.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (lower.find("tileset.json") != std::string::npos)
        return true;

    static const char* kContentExtensions[] = {
        ".b3dm", ".glb", ".gltf", ".i3dm", ".pnts", ".cmpt"};
    for (const char* ext : kContentExtensions)
    {
        if (lower.size() >= std::strlen(ext)
            && lower.compare(lower.size() - std::strlen(ext), std::strlen(ext), ext) == 0)
        {
            return true;
        }
    }

    if (lower.find("api.cesium.com") != std::string::npos
        || lower.find("assets.cesium.com") != std::string::npos)
    {
        return true;
    }

    return false;
}

RequestKind classifyAssetRequestUrl(const std::string& url)
{
    if (isTerrainRequestUrl(url)) {
        return RequestKind::Terrain;
    }
    if (isLikelyRasterImageryUrl(url)) {
        return RequestKind::Imagery;
    }
    return RequestKind::None;
}

uint16_t parseHttpStatusCodeString(const std::string& value)
{
    if (value.empty()) {
        return 0;
    }

    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || parsed < 100 || parsed > 599) {
        return 0;
    }
    return static_cast<uint16_t>(parsed);
}

uint16_t responseStatusFromReadResult(const ReadResult& result)
{
    if (const Config* httpStatus = result.metadata().find("HTTP Response Code")) {
        if (const uint16_t status = parseHttpStatusCodeString(httpStatus->value())) {
            return status;
        }
    }

    switch (result.code()) {
    case ReadResult::RESULT_OK:
        return 200;
    case ReadResult::RESULT_CANCELED:
        return 499;
    case ReadResult::RESULT_UNAUTHORIZED:
        return 403;
    case ReadResult::RESULT_NOT_FOUND:
        return 404;
    case ReadResult::RESULT_SERVER_ERROR:
        return 500;
    case ReadResult::RESULT_TIMEOUT:
        return 408;
    case ReadResult::RESULT_NOT_MODIFIED:
        return 304;
    default:
        return 500;
    }
}

void logTerrainRequestRead(
    const std::string& url,
    const std::string& readPath,
    bool isRemote,
    int readCode,
    uint16_t responseStatus,
    std::size_t contentBytes)
{
    if (!isTerrainRequestUrl(url))
        return;

    const bool blacklisted =
        osgEarth::Registry::instance()->isBlacklisted(readPath);

    if (osgEarth::Cesium::getLogTerrainRequest()) {
        OE_DEBUG << osgEarth::Cesium::elapsedLogStamp()
                 << " " << LC << "terrain GET"
                 << " url=" << url
                 << " readPath=" << readPath
                 << " remote=" << (isRemote ? "true" : "false")
                 << " readCode=" << readCode
                 << " status=" << responseStatus
                 << " bytes=" << contentBytes
                 << " blacklisted=" << (blacklisted ? "true" : "false")
                 << std::endl;
    }

    if (responseStatus == 200 || !osgEarth::Cesium::getLogHyperTerrainDiag()) {
        return;
    }

    std::ostringstream oss;
    oss << osgEarth::Cesium::elapsedLogStamp()
        << " " << LC << "terrain request failed"
        << " url=" << url
        << " readPath=" << readPath
        << " remote=" << (isRemote ? "true" : "false")
        << " readCode=" << readCode
        << " status=" << responseStatus
        << " bytes=" << contentBytes
        << " blacklisted=" << (blacklisted ? "true" : "false");
    appendHyperTerrainRequestDiagFile(oss.str());
}

void log3DTilesHttpLine(const char *verb, int statusCode, const std::string &url) {
    const std::string stamp = osgEarth::Cesium::elapsedLogStamp();
    if (verb) {
        std::fprintf(stderr, "%s [3DTiles] %s: %s\n", stamp.c_str(), verb, url.c_str());
    } else {
        std::fprintf(stderr, "%s [3DTiles] %d %s\n", stamp.c_str(), statusCode, url.c_str());
    }
}

void logTmsImageryHttpLine(int statusCode, const std::string& url, std::size_t contentBytes)
{
    if (!osgEarth::Cesium::getLogTmsRequest() || !isLikelyRasterImageryUrl(url))
        return;
    const std::string stamp = osgEarth::Cesium::elapsedLogStamp();
    std::fprintf(
        stderr,
        "%s [TMS] %d %s (bytes=%zu)\n",
        stamp.c_str(),
        statusCode,
        url.c_str(),
        contentBytes);
    std::fflush(stderr);
}

void maybeLog3DTilesHttpLine(const char* verb, int statusCode, const std::string& url)
{
    if (!is3DTilesRequestUrl(url))
        return;
    if (!osgEarth::Cesium::getLog3DTilesRequest())
        return;
    if (osg::getNotifyLevel() < osg::DEBUG_INFO)
        return;
    log3DTilesHttpLine(verb, statusCode, url);
}

template <typename Promise>
void resolveCancelledGet(
    const Promise& promise,
    const std::shared_ptr<AssetRequest>& request,
    RequestKind requestKind)
{
    auto response = std::make_unique<AssetResponse>();
    response->_statusCode = 499;
    recordRequestCompleted(requestKind, response->_statusCode, 0);
    request->setResponse(std::move(response));
    promise.resolve(request);
}

} // namespace

HyperTerrainAssetRequestStats osgEarth::Cesium::hyperTerrainAssetRequestStats()
{
    HyperTerrainAssetRequestStats out;
    copyCounterSet(s_terrainRequestCounters,
        out.terrainStarted,
        out.terrainCompleted,
        out.terrainOk,
        out.terrainNotFound,
        out.terrainError,
        out.terrainCancelled,
        out.terrainBytes,
        out.lastTerrainStatus,
        out.lastTerrainBytes);
    copyCounterSet(s_imageryRequestCounters,
        out.imageryStarted,
        out.imageryCompleted,
        out.imageryOk,
        out.imageryNotFound,
        out.imageryError,
        out.imageryCancelled,
        out.imageryBytes,
        out.lastImageryStatus,
        out.lastImageryBytes);
    return out;
}

std::string osgEarth::Cesium::elapsedLogStamp()
{
    using namespace std::chrono;
    const auto secCount = duration_cast<seconds>(steady_clock::now() - s_appLogOrigin).count();
    const unsigned long long t =
        secCount >= 0 ? static_cast<unsigned long long>(secCount) : 0ULL;
    const unsigned long long hh = t / 3600ULL;
    const unsigned mm = static_cast<unsigned>((t % 3600ULL) / 60ULL);
    const unsigned ss = static_cast<unsigned>(t % 60ULL);

    char buf[48];
    std::snprintf(buf, sizeof(buf), "[+%llu:%02u:%02u]", hh, mm, ss);
    return std::string(buf);
}

/**********************************************/
AssetRequest::AssetRequest(const std::string& method, const std::string& url, const std::vector<CesiumAsync::IAssetAccessor::THeader>& headers) :
    _method(method),
    _url(url)
{
    _headers.insert(headers.begin(), headers.end());
}

const std::string& AssetRequest::method() const
{
    return _method;
}

const std::string& AssetRequest::url() const
{
    return _url;
}

const CesiumAsync::HttpHeaders& AssetRequest::headers() const
{
    return _headers;
}

const CesiumAsync::IAssetResponse* AssetRequest::response() const
{
    return _response.get();
}

void AssetRequest::setResponse(std::unique_ptr< AssetResponse > response)
{
    _response = std::move(response);
}

/**********************************************/

uint16_t AssetResponse::statusCode() const
{
    return _statusCode;
}

std::string AssetResponse::contentType() const
{
    return _contentType;
}


const CesiumAsync::HttpHeaders& AssetResponse::headers() const
{
    return _headers;
}

std::span<const std::byte> AssetResponse::data() const
{
    return std::span<const std::byte>(_result.data(), _result.size());
}

/**********************************************/

AssetAccessor::AssetAccessor()
{
    _options = new osgDB::Options;
    _cacheSettings = new CacheSettings;
    _cacheSettings->setCache(Registry::instance()->getDefaultCache());
    _cacheSettings->store(_options.get());
}

CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>>
AssetAccessor::get(const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& url,
    const std::vector<CesiumAsync::IAssetAccessor::THeader>& headers)
{
    const RequestKind requestKind =
        getLogHyperTerrainDiag() ? classifyAssetRequestUrl(url) : RequestKind::None;
    if (isShuttingDown())
    {
        auto request = std::make_shared<AssetRequest>("GET", url, headers);
        auto response = std::make_unique<AssetResponse>();
        response->_statusCode = 499;
        recordRequestStarted(requestKind);
        recordRequestCompleted(requestKind, response->_statusCode, 0);
        request->setResponse(std::move(response));
        return asyncSystem.createResolvedFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(request);
    }

    if (isHyperTerrainLoadingActive() && shouldDeferUserTilesetNetworkRequest(url))
    {
        auto request = std::make_shared<AssetRequest>("GET", url, headers);
        auto response = std::make_unique<AssetResponse>();
        response->_statusCode = 499;
        request->setResponse(std::move(response));
        return asyncSystem.createResolvedFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(request);
    }

    osg::ref_ptr<osgDB::Options> options = _options.get();
    auto request = std::make_shared<AssetRequest>("GET", url, headers);
    recordRequestStarted(requestKind);
    return asyncSystem.createFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(
        [&](const auto& promise)
        {
            asyncSystem.runInWorkerThread([promise, request, url, headers, options, requestKind]() {
                if (isShuttingDown())
                {
                    resolveCancelledGet(promise, request, requestKind);
                    return;
                }

                maybeLog3DTilesHttpLine("GET", 0, url);

                URIContext uriContext;
                for (auto header : headers)
                {
                    uriContext.addHeader(header.first, header.second);
                }

                // Local disk tiles: read natively (query strip, gunzip, blank root).
                if (LocalTerrainUri::isLocalTerrainTileRequest(url))
                {
                    const std::filesystem::path nativePath =
                        LocalTerrainUri::terrainTileNativePathFromUrl(url);
                    const std::string readPath = nativePath.generic_string();
                    std::unique_ptr< AssetResponse > response = std::make_unique< AssetResponse >();

                    if (tryReadLocalTerrainTile(url, response->_result))
                    {
                        response->_statusCode = 200;
                        response->_contentType = "application/octet-stream";
                    }
                    else
                    {
                        warnTerrainTileMiss(url, nativePath);
                        response->_statusCode = 404;
                    }

                    logTerrainRequestRead(
                        url,
                        readPath,
                        false,
                        static_cast<int>(response->_statusCode),
                        response->_statusCode,
                        response->_result.size());
                    recordRequestCompleted(
                        requestKind,
                        response->_statusCode,
                        response->_result.size());

                    request->setResponse(std::move(response));

                    maybeLog3DTilesHttpLine(
                        nullptr,
                        static_cast<int>(request->_response->_statusCode),
                        url);

                    promise.resolve(request);
                    return;
                }

                const std::string readPath = localPathForFileUrl(url);
                URI uri(readPath, uriContext);

                auto httpResponse = uri.readString(options.get());
                if (isShuttingDown())
                {
                    resolveCancelledGet(promise, request, requestKind);
                    return;
                }

                std::unique_ptr< AssetResponse > response = std::make_unique< AssetResponse >();

                std::string content;
                response->_statusCode = responseStatusFromReadResult(httpResponse);
                if (response->_statusCode >= 200 && response->_statusCode < 300)
                {
                    content = httpResponse.getString();
                }
                else
                {
                    content = httpResponse.getString();
                }

                if (response->_statusCode == 200 && response->_result.empty())
                {
                    response->_contentType = httpResponse.metadata().value(IOMetadata::CONTENT_TYPE);
                    for (auto& i : httpResponse.metadata().children())
                    {
                        response->_headers[i.key()] = i.value();
                    }
                }

                logTerrainRequestRead(
                    url,
                    readPath,
                    uri.isRemote(),
                    static_cast<int>(httpResponse.code()),
                    response->_statusCode,
                    response->_result.empty() ? content.size() : response->_result.size());

                if (response->_result.empty())
                {
                    std::vector<std::byte> result(content.size());
                    for (unsigned int i = 0; i < content.size(); ++i)
                    {
                        result[i] = (std::byte)content[i];
                    }
                    response->_result = std::move(result);
                }

                std::string owsError;
                if (response->_statusCode == 200
                    && extractVWorldOwsException(content, owsError))
                {
                    response->_statusCode = 403;
                    OE_WARN << LC << "VWorld WMTS rejected tile"
                            << " url=" << url;
                    if (!owsError.empty())
                        OE_WARN << " message=" << owsError;
                    OE_WARN << std::endl;
                }

                const uint16_t imageryStatusCode = response->_statusCode;
                const std::size_t imageryBytes = response->_result.size();
                const bool logImagery = isLikelyRasterImageryUrl(url);
                recordRequestCompleted(
                    requestKind,
                    response->_statusCode,
                    response->_result.empty() ? content.size() : response->_result.size());

                request->setResponse(std::move(response));

                maybeLog3DTilesHttpLine(
                    nullptr,
                    static_cast<int>(request->_response->_statusCode),
                    url);

                if (logImagery) {
                    logTmsImageryHttpLine(
                        static_cast<int>(imageryStatusCode),
                        url,
                        imageryBytes);
                }

                promise.resolve(request);
                });
        }
    );
}

CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>>
AssetAccessor::request(
    const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& verb,
    const std::string& url,
    const std::vector<CesiumAsync::IAssetAccessor::THeader>& headers,
    const std::span<const std::byte>& contentPayload)
{
    if (isHyperTerrainLoadingActive() && shouldDeferUserTilesetNetworkRequest(url))
    {
        auto request = std::make_shared<AssetRequest>(verb, url, headers);
        auto response = std::make_unique<AssetResponse>();
        response->_statusCode = 499;
        request->setResponse(std::move(response));
        return asyncSystem.createResolvedFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(request);
    }

    auto request = std::make_shared<AssetRequest>(verb, url, headers);
    return asyncSystem.createFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(
        [&](const auto& promise)
        {
        }
    );
}

void AssetAccessor::tick() noexcept
{
}