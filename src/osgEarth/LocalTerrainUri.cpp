/* osgEarth

 * Copyright 2025 Pelican Mapping

 * MIT License

 */

#include <osgEarth/LocalTerrainUri>



#include <algorithm>

#include <cctype>

#include <cstdlib>



using namespace osgEarth;



namespace {



std::string trimCopy(const std::string& value)

{

    const auto begin = value.find_first_not_of(" \t\r\n");

    if (begin == std::string::npos)

        return {};

    const auto end = value.find_last_not_of(" \t\r\n");

    return value.substr(begin, end - begin + 1);

}



std::string toLowerAscii(std::string value)

{

    for (char& ch : value)

        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

    return value;

}



bool isFileSchemeUrl(const std::string& url)

{

    if (url.size() < 7)

        return false;

    return toLowerAscii(url.substr(0, 7)) == "file://";

}



std::string percentDecode(const std::string& value)

{

    std::string out;

    out.reserve(value.size());

    for (std::size_t i = 0; i < value.size(); ++i) {

        if (value[i] == '%' && i + 2 < value.size()) {

            const auto hex = value.substr(i + 1, 2);

            char* end = nullptr;

            const long code = std::strtol(hex.c_str(), &end, 16);

            if (end == hex.c_str() + 2) {

                out.push_back(static_cast<char>(code));

                i += 2;

                continue;

            }

        }

        out.push_back(value[i]);

    }

    return out;

}



bool endsWithTerrainExtension(const std::string& pathOnly)

{

    if (pathOnly.size() < 8)

        return false;

    return toLowerAscii(pathOnly.substr(pathOnly.size() - 8)) == ".terrain";

}



bool isWindowsAbsolutePath(const std::string& pathOnly)

{

    return pathOnly.size() >= 3 && std::isalpha(static_cast<unsigned char>(pathOnly[0]))

        && pathOnly[1] == ':'

        && (pathOnly[2] == '/' || pathOnly[2] == '\\');

}



std::string stripFileScheme(std::string path)

{

    const std::string lower = toLowerAscii(path);

    if (lower.rfind("file:///", 0) == 0)

        return path.substr(8);

    if (lower.rfind("file://", 0) == 0)

        return path.substr(7);

    return path;

}



std::string normalizeNativePathString(std::string path)

{

    path = stripFileScheme(std::move(path));

    if (path.size() >= 3 && path[0] == '/'

        && std::isalpha(static_cast<unsigned char>(path[1])) && path[2] == ':')

    {

        path.erase(0, 1);

    }

    return path;

}



} // namespace



std::string LocalTerrainUri::stripQueryAndFragment(std::string url)

{

    url = percentDecode(std::move(url));

    const auto query = url.find('?');

    if (query != std::string::npos)

        url.resize(query);

    const auto fragment = url.find('#');

    if (fragment != std::string::npos)

        url.resize(fragment);

    return url;

}



bool LocalTerrainUri::isTerrainTileFileUrl(const std::string& url)

{

    if (!isFileSchemeUrl(url))

        return false;

    return endsWithTerrainExtension(stripQueryAndFragment(url));

}



bool LocalTerrainUri::isLocalTerrainTileRequest(const std::string& url)

{

    const std::string pathOnly = stripQueryAndFragment(url);

    if (!endsWithTerrainExtension(pathOnly))

        return false;

    return isFileSchemeUrl(url) || isWindowsAbsolutePath(pathOnly);

}



std::filesystem::path LocalTerrainUri::terrainTileNativePathFromUrl(const std::string& url)

{

    return std::filesystem::path(normalizeNativePathString(stripQueryAndFragment(url)));

}



std::string LocalTerrainUri::fileUrlToNativePath(const std::string& fileUrl)

{

    if (isFileSchemeUrl(fileUrl))

        return terrainTileNativePathFromUrl(fileUrl).generic_string();

    // Remote http(s) URLs must keep ?query (access_token, v=, extensions=, etc.).

    return fileUrl;

}



std::filesystem::path LocalTerrainUri::normalizeRootPath(const std::string& root)

{

    const std::string trimmed = trimCopy(root);

    if (trimmed.empty())

        return {};



    if (isFileSchemeUrl(trimmed))

        return terrainTileNativePathFromUrl(trimmed);



    std::filesystem::path native(trimmed);

    if (!native.is_absolute())

        native = std::filesystem::absolute(native);

    return native;

}



std::string LocalTerrainUri::nativePathToFileUrl(const std::filesystem::path& path)

{

    const std::string generic = path.generic_string();

    if (generic.size() >= 2 && generic[1] == ':')

        return "file:///" + generic;

    return "file://" + generic;

}



std::string LocalTerrainUri::ensureTrailingSlash(std::string url)

{

    if (url.empty())

        return url;

    if (url.back() != '/')

        url.push_back('/');

    return url;

}



std::optional<LocalTerrainEndpoints> LocalTerrainUri::resolveLocalTerrain(

    const std::string& root,

    const std::string& tileset)

{

    const std::string trimmedTileset = trimCopy(tileset);

    if (trimmedTileset.empty())

        return std::nullopt;



    const std::filesystem::path nativeRoot = normalizeRootPath(root);

    if (nativeRoot.empty())

        return std::nullopt;



    LocalTerrainLocation location{nativeRoot, trimmedTileset};

    if (!LocalTerrainFileStore::tilesetExists(location))

        return std::nullopt;



    const std::filesystem::path tilesetDir = nativeRoot / trimmedTileset;

    const std::filesystem::path layerJsonPath = LocalTerrainFileStore::layerPath(location);



    LocalTerrainEndpoints endpoints;

    endpoints.location = location;

    endpoints.layerJsonUrl = nativePathToFileUrl(layerJsonPath);

    endpoints.tileBaseUrl = ensureTrailingSlash(nativePathToFileUrl(tilesetDir));

    return endpoints;

}



std::optional<LocalTerrainLocation> LocalTerrainUri::parseTerrainTileFileUrl(

    const std::string& fileUrl)

{

    if (!isLocalTerrainTileRequest(fileUrl))

        return std::nullopt;



    const std::filesystem::path nativePath = terrainTileNativePathFromUrl(fileUrl);

    if (nativePath.empty())

        return std::nullopt;



    uint64_t z = 0;

    uint64_t x = 0;

    uint64_t y = 0;

    if (!tryParseTerrainTileCoords(nativePath, z, x, y))

        return std::nullopt;



    const std::filesystem::path tilesetDir = nativePath.parent_path().parent_path().parent_path();

    const std::filesystem::path rootDir = tilesetDir.parent_path();

    if (tilesetDir.empty() || rootDir.empty())

        return std::nullopt;



    LocalTerrainLocation location{rootDir, tilesetDir.filename().string()};

    if (!LocalTerrainFileStore::tilesetExists(location))

        return std::nullopt;

    return location;

}



bool LocalTerrainUri::tryParseTerrainTileCoords(

    const std::filesystem::path& nativePath,

    uint64_t& outZ,

    uint64_t& outX,

    uint64_t& outY)

{

    const std::string filename = nativePath.filename().string();

    if (!endsWithTerrainExtension(filename))

        return false;



    const std::filesystem::path xDir = nativePath.parent_path();
    const std::filesystem::path zDir = xDir.parent_path();
    if (xDir.empty() || zDir.empty())
        return false;

    auto parseU64 = [](const std::string& text, uint64_t& out) -> bool {

        if (text.empty())

            return false;

        for (char ch : text) {

            if (!std::isdigit(static_cast<unsigned char>(ch)))

                return false;

        }

        try {

            out = std::stoull(text);

            return true;

        } catch (...) {

            return false;

        }

    };



    const std::string yText = filename.substr(0, filename.size() - 8);

    if (!parseU64(yText, outY) || !parseU64(xDir.filename().string(), outX)

        || !parseU64(zDir.filename().string(), outZ))

    {

        return false;

    }

    return true;

}

