/*!
* @Author Jason Hughes
* @Date December 2025
*
* @About graph processing
*/

#include <scenic/core/graphing_processor.hpp>

using namespace Scenic;

GraphingProcessor::GraphingProcess(size_t capacity) : ThreadedProcessor<GraphingInput>(capacity)
{

}

void GraphingProcessor(std::function<void(std::shared_ptr<ScenicGraph>)> callback)
{
    outputCallback = callback;
}

void GraphingProcessor::processBuffer()
{
    while(!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            // TODO graph 
        }
        else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
