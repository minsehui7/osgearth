/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/LocalTerrainFileStore>
#include <osgEarth/LocalTerrainUri>

#include <fstream>
#include <cstddef>
#include <zlib.h>

using namespace osgEarth;

namespace {

#include "LocalTerrainBlankRootData.inc"

bool isGzip(const std::byte* data, std::size_t size)
{
    return size >= 2
        && static_cast<unsigned char>(data[0]) == 0x1f
        && static_cast<unsigned char>(data[1]) == 0x8b;
}

bool gunzipBytes(const std::byte* data, std::size_t size, std::vector<std::byte>& out)
{
    z_stream stream{};
    if (inflateInit2(&stream, 16 + MAX_WBITS) != Z_OK)
        return false;

    stream.avail_in = static_cast<uInt>(size);
    stream.next_in = reinterpret_cast<Bytef*>(const_cast<std::byte*>(data));

    constexpr uInt chunkSize = 65536;
    int ret = Z_OK;
    do {
        const std::size_t oldSize = out.size();
        out.resize(oldSize + chunkSize);
        stream.avail_out = chunkSize;
        stream.next_out = reinterpret_cast<Bytef*>(out.data() + oldSize);
        ret = inflate(&stream, Z_NO_FLUSH);
        if (ret == Z_NEED_DICT || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
            inflateEnd(&stream);
            return false;
        }
        out.resize(oldSize + chunkSize - stream.avail_out);
    } while (ret != Z_STREAM_END);

    inflateEnd(&stream);
    return ret == Z_STREAM_END;
}

std::optional<std::vector<std::byte>> gunzipIfNeeded(std::vector<std::byte> data)
{
    if (data.empty())
        return std::nullopt;

    if (!isGzip(data.data(), data.size()))
        return data;

    std::vector<std::byte> out;
    if (!gunzipBytes(data.data(), data.size(), out))
        return std::nullopt;
    return out;
}

} // namespace

std::filesystem::path LocalTerrainFileStore::layerPath(const LocalTerrainLocation& location)
{
    return location.root / location.tileset / "layer.json";
}

std::filesystem::path LocalTerrainFileStore::tilePath(
    const LocalTerrainLocation& location,
    uint64_t z,
    uint64_t x,
    uint64_t y)
{
    return location.root / location.tileset / std::to_string(z) / std::to_string(x)
        / (std::to_string(y) + ".terrain");
}

bool LocalTerrainFileStore::tilesetExists(const LocalTerrainLocation& location)
{
    if (location.tileset.empty())
        return false;
    std::error_code ec;
    return std::filesystem::is_directory(location.root / location.tileset, ec);
}

bool LocalTerrainFileStore::isRootTile(uint64_t z, uint64_t x, uint64_t y)
{
    return z == 0 && y == 0 && (x == 0 || x == 1);
}

std::optional<std::vector<std::byte>> LocalTerrainFileStore::readFile(
    const std::filesystem::path& path)
{
    auto readStream = [](std::ifstream& stream) -> std::optional<std::vector<std::byte>> {
        stream.seekg(0, std::ios::end);
        const std::streamoff size = stream.tellg();
        if (size < 0)
            return std::nullopt;

        stream.seekg(0, std::ios::beg);
        std::vector<std::byte> data(static_cast<std::size_t>(size));
        if (!stream.read(reinterpret_cast<char*>(data.data()), size))
            return std::nullopt;
        return data;
    };

    {
        std::ifstream stream(path, std::ios::binary);
        if (stream) {
            if (auto data = readStream(stream))
                return data;
        }
    }

#ifdef _WIN32
    {
        std::ifstream stream(path.wstring(), std::ios::binary);
        if (stream) {
            if (auto data = readStream(stream))
                return data;
        }
    }

    {
        std::ifstream stream(path.native(), std::ios::binary);
        if (stream) {
            if (auto data = readStream(stream))
                return data;
        }
    }
#endif

    return std::nullopt;
}

std::optional<std::vector<std::byte>> LocalTerrainFileStore::readTerrainTileFromRequestUrl(
    const std::string& requestUrl)
{
    if (!LocalTerrainUri::isLocalTerrainTileRequest(requestUrl))
        return std::nullopt;

    const std::filesystem::path nativePath =
        LocalTerrainUri::terrainTileNativePathFromUrl(requestUrl);
    if (nativePath.empty())
        return std::nullopt;

    uint64_t z = 0;
    uint64_t x = 0;
    uint64_t y = 0;
    if (!LocalTerrainUri::tryParseTerrainTileCoords(nativePath, z, x, y))
        return std::nullopt;

    return readTerrainTileAtNativePath(nativePath, z, x, y);
}

std::optional<std::vector<std::byte>> LocalTerrainFileStore::readTerrainTileAtNativePath(
    const std::filesystem::path& nativeTilePath,
    uint64_t z,
    uint64_t x,
    uint64_t y)
{
    auto data = readFile(nativeTilePath);
    if (!data && isRootTile(z, x, y))
        return readBlankRootTileUncompressed(x);
    if (!data)
        return std::nullopt;
    return gunzipIfNeeded(std::move(*data));
}

std::optional<std::vector<std::byte>> LocalTerrainFileStore::readTerrainTile(
    const LocalTerrainLocation& location,
    uint64_t z,
    uint64_t x,
    uint64_t y)
{
    return readTerrainTileAtNativePath(tilePath(location, z, x, y), z, x, y);
}

std::optional<std::vector<std::byte>> LocalTerrainFileStore::readLayerJson(
    const LocalTerrainLocation& location)
{
    return readFile(layerPath(location));
}

std::optional<std::string> LocalTerrainFileStore::gunzipTerrainBytes(const std::string& raw)
{
    if (raw.empty())
        return std::nullopt;

    std::vector<std::byte> bytes(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i)
        bytes[i] = static_cast<std::byte>(static_cast<unsigned char>(raw[i]));

    const auto out = gunzipIfNeeded(std::move(bytes));
    if (!out)
        return std::nullopt;

    std::string result;
    result.resize(out->size());
    for (std::size_t i = 0; i < out->size(); ++i)
        result[i] = static_cast<char>((*out)[i]);
    return result;
}

const std::byte* LocalTerrainFileStore::blankRootTileGzipData(uint64_t rootX)
{
    if (rootX == 0)
        return reinterpret_cast<const std::byte*>(kBlankRootTileGzipX0);
    if (rootX == 1)
        return reinterpret_cast<const std::byte*>(kBlankRootTileGzipX1);
    return nullptr;
}

std::size_t LocalTerrainFileStore::blankRootTileGzipSize(uint64_t rootX)
{
    if (rootX == 0)
        return kBlankRootTileGzipX0_size;
    if (rootX == 1)
        return kBlankRootTileGzipX1_size;
    return 0;
}

std::optional<std::vector<std::byte>> LocalTerrainFileStore::readBlankRootTileUncompressed(
    uint64_t rootX)
{
    const std::byte* gzipData = blankRootTileGzipData(rootX);
    const std::size_t gzipSize = blankRootTileGzipSize(rootX);
    if (!gzipData || gzipSize == 0)
        return std::nullopt;

    std::vector<std::byte> out;
    if (!gunzipBytes(gzipData, gzipSize, out))
        return std::nullopt;
    return out;
}
