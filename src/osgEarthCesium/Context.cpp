/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/

#include "Context"
#include "Settings"

#include <CesiumIonClient/Connection.h>

using namespace osgEarth::Cesium;

Context::Context():
    taskProcessor(std::make_shared<TaskProcessor>()),
    asyncSystem(taskProcessor)
{
    Cesium3DTilesContent::registerAllTileContentTypes();
    assetAccessor = std::make_shared<AssetAccessor>();
    prepareRenderResources = std::make_shared< PrepareRendererResources >();
    logger = spdlog::default_logger();
    creditSystem = std::make_shared<CesiumUtility::CreditSystem>();
}

Context::~Context()
{
    shutdown();
}

void Context::shutdown()
{
    if (taskProcessor)
        taskProcessor->shutdown();

    if (assetAccessor)
        assetAccessor->tick();

    // Drain a few main-thread continuations without blocking on worker pools.
    for (int i = 0; i < 8; ++i)
    {
        asyncSystem.dispatchMainThreadTasks();
        if (assetAccessor)
            assetAccessor->tick();
    }
}

