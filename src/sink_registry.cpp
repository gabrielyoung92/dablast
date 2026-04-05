#include "sink_registry.h"
#include <mutex>
#include <queue>

static std::mutex          g_mutex;
static std::queue<SubchannelSink*> g_queue;

void dablast_push_sink(SubchannelSink* sink) {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_queue.push(sink);
}

SubchannelSink* dablast_pop_sink() {
    std::lock_guard<std::mutex> lk(g_mutex);
    if (g_queue.empty()) return nullptr;
    auto* s = g_queue.front();
    g_queue.pop();
    return s;
}
