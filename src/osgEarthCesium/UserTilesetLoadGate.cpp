/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
 * MIT License
 */
#include "UserTilesetLoadGate"

#include <Cesium3DTilesSelection/Tileset.h>
#include <Cesium3DTilesSelection/TilesetOptions.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace osgEarth::Cesium {
namespace {

std::mutex g_mutex;

struct UserTilesetRegistration {
    Cesium3DTilesSelection::Tileset* tileset = nullptr;
    int64_t ionAssetId = 0;
    std::string urlPrefixLower;

    bool optionsSaved = false;
    uint32_t savedMaximumSimultaneousTileLoads = 20;
    uint32_t savedLoadingDescendantLimit = 20;
    bool savedPreloadAncestors = true;
    bool savedPreloadSiblings = true;
    uint32_t savedMaximumMainThreadTilesPerLoadPass = 0;
    bool savedEnforceMainThreadTilesPerLoadPass = false;
};

std::vector<UserTilesetRegistration> g_userTilesets;
std::vector<int64_t> g_terrainIonAssetIds;
std::vector<std::string> g_terrainUrlPrefixLower;

std::string toLower(std::string s)
{
    std::transform(
        s.begin(),
        s.end(),
        s.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string normalizeUrlPrefix(std::string url)
{
    url = toLower(std::move(url));
    const std::string marker = "tileset.json";
    const size_t pos = url.find(marker);
    if (pos != std::string::npos) {
        url.resize(pos);
    }
    while (!url.empty() && (url.back() == '/' || url.back() == '\\')) {
        url.pop_back();
    }
    return url;
}

bool urlContainsIonAssetId(const std::string& urlLower, int64_t assetId)
{
    if (assetId <= 0) {
        return false;
    }
    char pattern[64];
    const int n = std::snprintf(
        pattern,
        sizeof(pattern),
        "/assets/%lld/",
        static_cast<long long>(assetId));
    if (n <= 0) {
        return false;
    }
    return urlLower.find(pattern) != std::string::npos;
}

bool urlMatchesPrefix(const std::string& urlLower, const std::string& prefixLower)
{
    return !prefixLower.empty() && urlLower.find(prefixLower) == 0;
}

bool isTerrainExcludedUrl(const std::string& urlLower)
{
    for (int64_t id : g_terrainIonAssetIds) {
        if (urlContainsIonAssetId(urlLower, id)) {
            return true;
        }
    }
    for (const std::string& prefix : g_terrainUrlPrefixLower) {
        if (urlMatchesPrefix(urlLower, prefix)) {
            return true;
        }
    }
    return false;
}

bool isRegisteredUserTilesetUrl(const std::string& urlLower)
{
    for (const UserTilesetRegistration& reg : g_userTilesets) {
        if (urlContainsIonAssetId(urlLower, reg.ionAssetId)) {
            return true;
        }
        if (urlMatchesPrefix(urlLower, reg.urlPrefixLower)) {
            return true;
        }
    }
    return false;
}

void applyDeferToTileset(UserTilesetRegistration& reg, bool defer)
{
    if (!reg.tileset) {
        return;
    }

    Cesium3DTilesSelection::TilesetOptions& opts = reg.tileset->getOptions();
    if (defer) {
        if (!reg.optionsSaved) {
            reg.savedMaximumSimultaneousTileLoads = opts.maximumSimultaneousTileLoads;
            reg.savedLoadingDescendantLimit = opts.loadingDescendantLimit;
            reg.savedPreloadAncestors = opts.preloadAncestors;
            reg.savedPreloadSiblings = opts.preloadSiblings;
            reg.savedMaximumMainThreadTilesPerLoadPass =
                opts.maximumMainThreadTilesPerLoadPass;
            reg.savedEnforceMainThreadTilesPerLoadPass =
                opts.enforceMainThreadTilesPerLoadPass;
            reg.optionsSaved = true;
        }
        opts.maximumSimultaneousTileLoads = 0;
        opts.loadingDescendantLimit = 0;
        opts.preloadAncestors = false;
        opts.preloadSiblings = false;
        opts.maximumMainThreadTilesPerLoadPass = 0;
        opts.enforceMainThreadTilesPerLoadPass = true;
    } else if (reg.optionsSaved) {
        opts.maximumSimultaneousTileLoads = reg.savedMaximumSimultaneousTileLoads;
        opts.loadingDescendantLimit = reg.savedLoadingDescendantLimit;
        opts.preloadAncestors = reg.savedPreloadAncestors;
        opts.preloadSiblings = reg.savedPreloadSiblings;
        opts.maximumMainThreadTilesPerLoadPass =
            reg.savedMaximumMainThreadTilesPerLoadPass;
        opts.enforceMainThreadTilesPerLoadPass =
            reg.savedEnforceMainThreadTilesPerLoadPass;
        reg.optionsSaved = false;
    }
}

} // namespace

void clearTerrainLoadingExclusions()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_terrainIonAssetIds.clear();
    g_terrainUrlPrefixLower.clear();
}

void registerTerrainLoadingExclusionIonAsset(int64_t ionAssetId)
{
    if (ionAssetId <= 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    if (std::find(g_terrainIonAssetIds.begin(), g_terrainIonAssetIds.end(), ionAssetId)
        == g_terrainIonAssetIds.end()) {
        g_terrainIonAssetIds.push_back(ionAssetId);
    }
}

void registerTerrainLoadingExclusionUrl(const std::string& rootUrl)
{
    const std::string prefix = normalizeUrlPrefix(rootUrl);
    if (prefix.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    if (std::find(g_terrainUrlPrefixLower.begin(), g_terrainUrlPrefixLower.end(), prefix)
        == g_terrainUrlPrefixLower.end()) {
        g_terrainUrlPrefixLower.push_back(prefix);
    }
}

void registerUserTileset(
    Cesium3DTilesSelection::Tileset* tileset,
    int64_t ionAssetId,
    const std::string& rootUrl)
{
    if (!tileset) {
        return;
    }

    UserTilesetRegistration reg;
    reg.tileset = tileset;
    reg.ionAssetId = ionAssetId;
    reg.urlPrefixLower = normalizeUrlPrefix(rootUrl);

    std::lock_guard<std::mutex> lock(g_mutex);
    g_userTilesets.push_back(std::move(reg));
}

void unregisterUserTileset(Cesium3DTilesSelection::Tileset* tileset)
{
    if (!tileset) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    for (auto it = g_userTilesets.begin(); it != g_userTilesets.end();) {
        if (it->tileset != tileset) {
            ++it;
            continue;
        }
        applyDeferToTileset(*it, false);
        it = g_userTilesets.erase(it);
    }
}

bool shouldDeferUserTilesetNetworkRequest(const std::string& url)
{
    const std::string urlLower = toLower(url);
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!isRegisteredUserTilesetUrl(urlLower)) {
        return false;
    }
    return !isTerrainExcludedUrl(urlLower);
}

void applyUserTilesetLoadDeferPolicy(bool defer)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    for (UserTilesetRegistration& reg : g_userTilesets) {
        applyDeferToTileset(reg, defer);
    }
    // Newly registered tilesets after apply(true) may have missed defer; catch up next frame.
    if (defer) {
        for (UserTilesetRegistration& reg : g_userTilesets) {
            if (!reg.optionsSaved && reg.tileset) {
                applyDeferToTileset(reg, true);
            }
        }
    }
}

} // namespace osgEarth::Cesium
