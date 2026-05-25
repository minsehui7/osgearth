#include <osgEarth/LocalTerrainFileStore>
#include <osgEarth/LocalTerrainUri>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

int gFailures = 0;

void expectTrue(bool value, const char* message)
{
    if (!value) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++gFailures;
    }
}

void testTilePathMapping()
{
    const osgEarth::LocalTerrainLocation location{
        std::filesystem::path("C:/data/terrain"),
        "ALOS_Korea_DSM"};
    const auto path = osgEarth::LocalTerrainFileStore::tilePath(location, 12, 3456, 789);
    expectTrue(
        path.generic_string() == "C:/data/terrain/ALOS_Korea_DSM/12/3456/789.terrain",
        "tile path matches cesium-terrain-server fs.go layout");
}

void testRootTileDetection()
{
    expectTrue(osgEarth::LocalTerrainFileStore::isRootTile(0, 0, 0), "0/0/0 is root");
    expectTrue(osgEarth::LocalTerrainFileStore::isRootTile(0, 1, 0), "0/1/0 is root");
    expectTrue(!osgEarth::LocalTerrainFileStore::isRootTile(1, 0, 0), "z=1 is not root");
}

void testResolveLocalTerrainUrl()
{
    const auto endpoints =
        osgEarth::LocalTerrainUri::resolveLocalTerrain("C:/data/terrain", "ALOS_Korea_DSM");
    if (!endpoints) {
        std::fprintf(stderr, "SKIP: tileset C:/data/terrain/ALOS_Korea_DSM not present on disk\n");
        return;
    }
    expectTrue(
        endpoints->layerJsonUrl.find("file:///") == 0,
        "layer.json URL uses file scheme");
    expectTrue(
        endpoints->layerJsonUrl.find("layer.json") != std::string::npos,
        "layer.json URL ends with layer.json");
    expectTrue(
        endpoints->tileBaseUrl.find("ALOS_Korea_DSM/") != std::string::npos,
        "tile base URL includes tileset directory");
}

constexpr std::size_t kHeightmapBlankUncompressedSize = 8452u;
constexpr std::size_t kQuantizedMeshHeaderSize = 92u;

uint32_t readVertexCount(const std::vector<std::byte>& bytes)
{
    if (bytes.size() < kQuantizedMeshHeaderSize)
        return 0;
    uint32_t vertexCount = 0;
    std::memcpy(&vertexCount, bytes.data() + 88, sizeof(vertexCount));
    return vertexCount;
}

void testBlankRootTileGunzip()
{
    for (uint64_t rootX : {0u, 1u}) {
        const auto uncompressed =
            osgEarth::LocalTerrainFileStore::readBlankRootTileUncompressed(rootX);
        expectTrue(uncompressed.has_value(), "blank root tile gunzips");
        expectTrue(uncompressed.has_value() && !uncompressed->empty(), "blank root tile has payload");
        if (!uncompressed)
            continue;
        expectTrue(
            uncompressed->size() != kHeightmapBlankUncompressedSize,
            "blank root tile is quantized-mesh, not heightmap-1.0 (8452 bytes)");
        expectTrue(
            uncompressed->size() > kQuantizedMeshHeaderSize,
            "blank root tile has quantized-mesh header");
        const uint32_t vertexCount = readVertexCount(*uncompressed);
        expectTrue(vertexCount > 0u && vertexCount < 100000u, "blank root vertexCount plausible");

        const std::size_t gzipSize =
            osgEarth::LocalTerrainFileStore::blankRootTileGzipSize(rootX);
        expectTrue(gzipSize > 0, "embedded blank root gzip size");
        expectTrue(
            osgEarth::LocalTerrainFileStore::blankRootTileGzipData(rootX) != nullptr,
            "embedded blank root gzip pointer");
    }
}

void testBlankRootMatchesReferenceFiles()
{
    const std::filesystem::path refRoot =
        std::filesystem::path("data") / "quantized-mesh-blank-root";
    for (uint64_t rootX : {0u, 1u}) {
        const std::filesystem::path refPath =
            refRoot / (std::string("0-") + std::to_string(rootX) + "-0.terrain");
        if (!std::filesystem::is_regular_file(refPath))
            continue;

        const auto refBytes = osgEarth::LocalTerrainFileStore::readFile(refPath);
        expectTrue(refBytes.has_value(), "reference blank root file readable");
        if (!refBytes)
            continue;

        const std::size_t embedSize =
            osgEarth::LocalTerrainFileStore::blankRootTileGzipSize(rootX);
        const auto* embedData =
            osgEarth::LocalTerrainFileStore::blankRootTileGzipData(rootX);
        expectTrue(embedData != nullptr && embedSize == refBytes->size(), "embed gzip size matches ref");
        if (embedData && embedSize == refBytes->size()) {
            expectTrue(
                std::memcmp(embedData, refBytes->data(), embedSize) == 0,
                "embedded blank root gzip bytes match reference file");
        }
    }
}

void testTerrainTileUrlQueryStrip()
{
    const std::string url =
        "file:///C:/data/terrain/ALOS_Korea_DSM/2/6/3.terrain?v=1.1.0&extensions=octvertexnormals";
    expectTrue(
        osgEarth::LocalTerrainUri::isTerrainTileFileUrl(url),
        "terrain tile file URL recognized with query suffix");
    expectTrue(
        osgEarth::LocalTerrainUri::fileUrlToNativePath(url)
            == "C:/data/terrain/ALOS_Korea_DSM/2/6/3.terrain",
        "query stripped from native terrain path");

    uint64_t z = 0;
    uint64_t x = 0;
    uint64_t y = 0;
    expectTrue(
        osgEarth::LocalTerrainUri::tryParseTerrainTileCoords(
            std::filesystem::path(osgEarth::LocalTerrainUri::fileUrlToNativePath(url)),
            z,
            x,
            y),
        "tile coords parse after query strip");
    expectTrue(z == 2 && x == 6 && y == 3, "tile coords z/x/y");
}

void testRemoteUrlQueryPreserved()
{
    const std::string ionUrl =
        "https://api.cesium.com/v1/assets/1/endpoint?access_token=test-token";
    expectTrue(
        osgEarth::LocalTerrainUri::fileUrlToNativePath(ionUrl) == ionUrl,
        "https URL query preserved for AssetAccessor");

    const std::string mapboxUrl =
        "https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/0/0/0?access_token=pk.test";
    expectTrue(
        osgEarth::LocalTerrainUri::fileUrlToNativePath(mapboxUrl) == mapboxUrl,
        "mapbox URL query preserved");
}

void testReadRealTerrainTileFromDisk()
{
    const auto endpoints =
        osgEarth::LocalTerrainUri::resolveLocalTerrain("C:/data/terrain", "ALOS_Korea_DSM");
    if (!endpoints) {
        std::fprintf(stderr, "SKIP: tileset C:/data/terrain/ALOS_Korea_DSM not present on disk\n");
        return;
    }

    const auto bytes = osgEarth::LocalTerrainFileStore::readTerrainTile(
        endpoints->location, 2, 6, 3);
    expectTrue(bytes.has_value() && !bytes->empty(), "readTerrainTile returns decompressed payload");
    expectTrue(bytes->size() > 92, "decompressed tile has quantized-mesh header");
}

void testLocalTerrainTileRequestVariants()
{
    const std::string fileUrl =
        "file:///C:/data/terrain/ALOS_Korea_DSM/12/6985/2902.terrain?v=1.1.0&extensions=octvertexnormals";
    const std::string bareUrl =
        "C:/data/terrain/ALOS_Korea_DSM/12/6985/2902.terrain?v=1.1.0&extensions=octvertexnormals";
    const std::string upperSchemeUrl =
        "FILE:///C:/data/terrain/ALOS_Korea_DSM/12/6985/2902.terrain?v=1.1.0";

    expectTrue(
        osgEarth::LocalTerrainUri::isLocalTerrainTileRequest(fileUrl),
        "file URL is local terrain tile request");
    expectTrue(
        osgEarth::LocalTerrainUri::isLocalTerrainTileRequest(bareUrl),
        "bare Windows path is local terrain tile request");
    expectTrue(
        osgEarth::LocalTerrainUri::isLocalTerrainTileRequest(upperSchemeUrl),
        "uppercase FILE scheme is local terrain tile request");
    expectTrue(
        osgEarth::LocalTerrainUri::terrainTileNativePathFromUrl(fileUrl).generic_string()
            == "C:/data/terrain/ALOS_Korea_DSM/12/6985/2902.terrain",
        "native path from file URL");
    expectTrue(
        osgEarth::LocalTerrainUri::terrainTileNativePathFromUrl(upperSchemeUrl).generic_string()
            == "C:/data/terrain/ALOS_Korea_DSM/12/6985/2902.terrain",
        "native path from uppercase FILE URL");
}

void testParseAndReadDetailTileFromDisk()
{
    const std::string url =
        "file:///C:/data/terrain/ALOS_Korea_DSM/12/6985/2902.terrain?v=1.1.0&extensions=octvertexnormals";
    const auto location = osgEarth::LocalTerrainUri::parseTerrainTileFileUrl(url);
    expectTrue(location.has_value(), "parseTerrainTileFileUrl for L12 detail tile");

    const std::filesystem::path nativePath =
        osgEarth::LocalTerrainUri::fileUrlToNativePath(url);
    expectTrue(std::filesystem::is_regular_file(nativePath), "L12 detail tile exists on disk");

    uint64_t z = 0;
    uint64_t x = 0;
    uint64_t y = 0;
    expectTrue(
        osgEarth::LocalTerrainUri::tryParseTerrainTileCoords(nativePath, z, x, y),
        "L12 detail tile coords parse");
    expectTrue(z == 12 && x == 6985 && y == 2902, "L12 detail tile z/x/y");

    if (!location)
        return;

    const auto bytes = osgEarth::LocalTerrainFileStore::readTerrainTileFromRequestUrl(url);
    expectTrue(bytes.has_value() && !bytes->empty(), "readTerrainTileFromRequestUrl L12 detail tile");
    expectTrue(bytes->size() > 92, "L12 detail tile has quantized-mesh header");
}

void testRootTileCoordParsing()
{
    uint64_t z = 0;
    uint64_t x = 0;
    uint64_t y = 0;
    expectTrue(
        osgEarth::LocalTerrainUri::tryParseTerrainTileCoords(
            std::filesystem::path("C:/data/terrain/ALOS_Korea_DSM/0/0/0.terrain"),
            z,
            x,
            y),
        "root 0/0/0 coords parse");
    expectTrue(z == 0 && x == 0 && y == 0, "root 0/0/0 z/x/y");

    expectTrue(
        osgEarth::LocalTerrainUri::tryParseTerrainTileCoords(
            std::filesystem::path("C:/data/terrain/ALOS_Korea_DSM/0/1/0.terrain"),
            z,
            x,
            y),
        "root 0/1/0 coords parse");
    expectTrue(z == 0 && x == 1 && y == 0, "root 0/1/0 z/x/y");
}

} // namespace

int main()
{
    testTilePathMapping();
    testRootTileDetection();
    testResolveLocalTerrainUrl();
    testBlankRootTileGunzip();
    testBlankRootMatchesReferenceFiles();
    testTerrainTileUrlQueryStrip();
    testRemoteUrlQueryPreserved();
    testReadRealTerrainTileFromDisk();
    testLocalTerrainTileRequestVariants();
    testParseAndReadDetailTileFromDisk();
    testRootTileCoordParsing();
    if (gFailures != 0) {
        std::fprintf(stderr, "%d test(s) failed\n", gFailures);
        return EXIT_FAILURE;
    }
    std::fprintf(stderr, "LocalTerrainFileStoreTest: all passed\n");
    return EXIT_SUCCESS;
}
