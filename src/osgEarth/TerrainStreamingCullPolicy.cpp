/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/TerrainStreamingCullPolicy>

namespace osgEarth
{
namespace
{
    TerrainStreamingPrioritizeTerrainCullFn g_prioritizeTerrainCull;
}

void setTerrainStreamingPrioritizeTerrainCullFn(TerrainStreamingPrioritizeTerrainCullFn fn)
{
    g_prioritizeTerrainCull = std::move(fn);
}

bool terrainStreamingPrioritizeTerrainCull()
{
    return g_prioritizeTerrainCull ? g_prioritizeTerrainCull() : false;
}

} // namespace osgEarth
