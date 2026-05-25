/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "AssetAccessor"
#include "Settings"

#include <CesiumAsync/AsyncSystem.h>
#include <osgEarth/LocalTerrainFileStore>
#include <osgEarth/LocalTerrainUri>
#include <osgEarth/URI>
#include <osgEarth/Registry>
#include <osgEarth/Notify>

#include <osg/Notify>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstring>

#undef LC
#define LC "[HyperTerrain] "

using namespace osgEarth;
using namespace osgEarth::Cesium;

namespace {

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

    OE_WARN << LC << "terrain tile read failed but file exists"
            << " url=" << url
            << " native=" << nativePath.generic_string()
            << std::endl;
}

// Elapsed since first 3D Tiles HTTP log (same "[+h:mm:ss]" as HyperIndexer / h3dt-build).
std::chrono::steady_clock::time_point logSessionOrigin() {
    static const std::chrono::steady_clock::time_point origin = std::chrono::steady_clock::now();
    return origin;
}

void writeElapsedStamp(char *buf, std::size_t bufSize) {
    using namespace std::chrono;
    const auto secCount = duration_cast<seconds>(steady_clock::now() - logSessionOrigin()).count();
    const unsigned long long t =
        secCount >= 0 ? static_cast<unsigned long long>(secCount) : 0ULL;
    const unsigned long long hh = t / 3600ULL;
    const unsigned mm = static_cast<unsigned>((t % 3600ULL) / 60ULL);
    const unsigned ss = static_cast<unsigned>(t % 60ULL);
    std::snprintf(buf, bufSize, "[+%llu:%02u:%02u]", hh, mm, ss);
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
        || lower.find("req/wmts") != std::string::npos;
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

void logTerrainRequestRead(
    const std::string& url,
    const std::string& readPath,
    bool isRemote,
    int readCode,
    std::size_t contentBytes)
{
    if (!osgEarth::Cesium::getLogTerrainRequest() || !isTerrainRequestUrl(url))
        return;

    const bool blacklisted =
        osgEarth::Registry::instance()->isBlacklisted(readPath);

    OE_DEBUG << LC << "terrain GET"
             << " url=" << url
             << " readPath=" << readPath
             << " remote=" << (isRemote ? "true" : "false")
             << " code=" << readCode
             << " bytes=" << contentBytes
             << " blacklisted=" << (blacklisted ? "true" : "false")
             << std::endl;
}

void log3DTilesHttpLine(const char *verb, int statusCode, const std::string &url) {
    char stamp[48];
    writeElapsedStamp(stamp, sizeof(stamp));
    if (verb) {
        std::fprintf(stderr, "%s [3DTiles] %s: %s\n", stamp, verb, url.c_str());
    } else {
        std::fprintf(stderr, "%s [3DTiles] %d %s\n", stamp, statusCode, url.c_str());
    }
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
    const std::shared_ptr<AssetRequest>& request)
{
    auto response = std::make_unique<AssetResponse>();
    response->_statusCode = 499;
    request->setResponse(std::move(response));
    promise.resolve(request);
}

} // namespace

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
    if (isShuttingDown())
    {
        auto request = std::make_shared<AssetRequest>("GET", url, headers);
        auto response = std::make_unique<AssetResponse>();
        response->_statusCode = 499;
        request->setResponse(std::move(response));
        return asyncSystem.createResolvedFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(request);
    }

    osg::ref_ptr<osgDB::Options> options = _options.get();
    auto request = std::make_shared<AssetRequest>("GET", url, headers);
    return asyncSystem.createFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(
        [&](const auto& promise)
        {
            asyncSystem.runInWorkerThread([promise, request, url, headers, options]() {
                if (isShuttingDown())
                {
                    resolveCancelledGet(promise, request);
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
                    resolveCancelledGet(promise, request);
                    return;
                }

                std::unique_ptr< AssetResponse > response = std::make_unique< AssetResponse >();

                std::string content;
                if (httpResponse.code() == ReadResult::RESULT_OK)
                {
                    response->_statusCode = 200;
                    content = httpResponse.getString();
                }
                else
                {
                    content = httpResponse.getString();
                    response->_statusCode =
                        httpResponse.code() == ReadResult::RESULT_NOT_FOUND ? 404 : 500;
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
                request->setResponse(std::move(response));

                maybeLog3DTilesHttpLine(
                    nullptr,
                    static_cast<int>(request->_response->_statusCode),
                    url);

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