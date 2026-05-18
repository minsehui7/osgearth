/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "AssetAccessor"
#include "Settings"

#include <CesiumAsync/AsyncSystem.h>
#include <CesiumUtility/Uri.h>
#include <osgEarth/URI>
#include <osgEarth/Registry>

#include <chrono>
#include <cstdio>
#include <cstring>

using namespace osgEarth;
using namespace osgEarth::Cesium;

namespace {

/// file:/// URLs must be converted before osgEarth local read (ifstream cannot open the URI string).
std::string localPathForFileUrl(const std::string& url) {
    const std::size_t schemeEnd = url.find("://");
    if (schemeEnd == std::string::npos) {
        return url;
    }
    if (url.compare(0, schemeEnd, "file") != 0) {
        return url;
    }
    const CesiumUtility::Uri parsed(url);
    if (!parsed.isValid()) {
        return url;
    }
    return CesiumUtility::Uri::uriPathToNativePath(std::string(parsed.getPath()));
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

// #region agent log
void agentLogAssetRead(
    const char *hypothesisId,
    const char *message,
    const std::string &url,
    const std::string &readPath,
    bool isRemote,
    int readCode,
    std::size_t contentBytes,
    bool blacklisted) {
    FILE *f = std::fopen("G:/dev/HyperLiDAR/debug-d0b09c.log", "ab");
    if (!f) {
        return;
    }
    const auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    std::fprintf(
        f,
        "{\"sessionId\":\"d0b09c\",\"hypothesisId\":\"%s\",\"location\":\"AssetAccessor.cpp:get\","
        "\"message\":\"%s\",\"data\":{\"url\":\"%s\",\"readPath\":\"%s\",\"isRemote\":%s,"
        "\"readCode\":%d,\"contentBytes\":%zu,\"blacklisted\":%s},\"timestamp\":%lld}\n",
        hypothesisId,
        message,
        url.c_str(),
        readPath.c_str(),
        isRemote ? "true" : "false",
        readCode,
        contentBytes,
        blacklisted ? "true" : "false",
        static_cast<long long>(ts.count()));
    std::fclose(f);
}
// #endregion

void log3DTilesHttpLine(const char *verb, int statusCode, const std::string &url) {
    char stamp[48];
    writeElapsedStamp(stamp, sizeof(stamp));
    if (verb) {
        std::fprintf(stderr, "%s [3DTiles] %s: %s\n", stamp, verb, url.c_str());
    } else {
        std::fprintf(stderr, "%s [3DTiles] %d %s\n", stamp, statusCode, url.c_str());
    }
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
    osg::ref_ptr<osgDB::Options> options = _options.get();
    auto request = std::make_shared<AssetRequest>("GET", url, headers);
    return asyncSystem.createFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(
        [&](const auto& promise)
        {
            asyncSystem.runInWorkerThread([promise, request, url, headers, options]() {
                const bool logHttp = osgEarth::Cesium::getLog3DTilesHttpUrls();
                if (logHttp) {
                    log3DTilesHttpLine("GET", 0, url);
                }

                URIContext uriContext;
                for (auto header : headers)
                {
                    uriContext.addHeader(header.first, header.second);
                }
                
                const std::string readPath = localPathForFileUrl(url);
                URI uri(readPath, uriContext);

                const bool blacklisted =
                    osgEarth::Registry::instance()->isBlacklisted(uri.full());
                auto httpResponse = uri.readString(options.get());
                std::unique_ptr< AssetResponse > response = std::make_unique< AssetResponse >();

                if (httpResponse.code() == ReadResult::RESULT_OK)
                {
                    response->_statusCode = 200;
                }
                response->_contentType = httpResponse.metadata().value(IOMetadata::CONTENT_TYPE);
                for (auto& i : httpResponse.metadata().children())
                {
                    response->_headers[i.key()] = i.value();
                }
                std::string content = httpResponse.getString();

                // #region agent log
                agentLogAssetRead(
                    "H1",
                    "asset_get_after_readString",
                    url,
                    readPath,
                    uri.isRemote(),
                    static_cast<int>(httpResponse.code()),
                    content.size(),
                    blacklisted);
                // #endregion

                std::vector<std::byte> result(content.size());
                for (unsigned int i = 0; i < content.size(); ++i)
                {
                    result[i] = (std::byte)content[i];
                }

                response->_result = result;
                request->setResponse(std::move(response));

                if (logHttp) {
                    log3DTilesHttpLine(
                        nullptr,
                        static_cast<int>(request->_response->_statusCode),
                        url);
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