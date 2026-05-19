/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "TaskProcessor"
#include "Settings"

#include <osgEarth/Threading>
#include <osgEarth/Notify>

#include <chrono>
#include <thread>

using namespace osgEarth;
using namespace osgEarth::Threading;
using namespace osgEarth::Cesium;

const std::string CESIUM_ARENA_NAME = "cesium";

namespace {

constexpr int kShutdownWaitMs = 500;

} // namespace

TaskProcessor::TaskProcessor()
{
    jobs::get_pool(CESIUM_ARENA_NAME)->set_concurrency(8);
}

TaskProcessor::~TaskProcessor()
{
}

void TaskProcessor::shutdown()
{
    auto* pool = jobs::get_pool(CESIUM_ARENA_NAME);
    if (!pool)
        return;

    pool->cancel_all();

    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(kShutdownWaitMs);
    for (;;)
    {
        auto* metrics = pool->metrics();
        const unsigned int active = metrics->pending + metrics->running;
        if (active == 0)
            break;
        if (std::chrono::steady_clock::now() >= deadline)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void TaskProcessor::startTask(std::function<void()> f)
{
    if (isShuttingDown())
        return;

    auto task = [f = std::move(f)]() {
        f();
        return true;
    };
    jobs::context cx;
    cx.pool = jobs::get_pool(CESIUM_ARENA_NAME);
    jobs::dispatch(task, cx);
}
