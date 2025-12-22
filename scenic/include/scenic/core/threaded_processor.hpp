/*!
* @Author Jason Hughes
* @Date July 2025 
*
* @About Base class for the processors
*/

#pragma once

#include <random>
#include <thread>
#include <atomic>

#include "scenic/core/process_buffer.hpp"

namespace Scenic
{
template <typename F>
class ThreadedProcessor : public ProcessBuffer<F>
{
    public:
        ThreadedProcessor() = default;
        ThreadedProcessor(size_t capacity);

        virtual ~ThreadedProcessor();

        void start();
        void stop();

        bool isStopped() const;

        int generateProcessID() const;

    protected:
        virtual void processBuffer() = 0;
        size_t min_elem_{1};

    private:
        std::thread processing_thread_;
        std::atomic<bool> should_stop_{false};
};

template <typename F>
ThreadedProcessor<F>::ThreadedProcessor(size_t capacity) : ProcessBuffer<F>(capacity)
{
    start();
}

template <typename F>
ThreadedProcessor<F>::~ThreadedProcessor()
{
    stop();
}

template <typename F>
void ThreadedProcessor<F>::start()
{
    if (!processing_thread_.joinable()) {
        should_stop_ = false;
        processing_thread_ = std::thread(&Scenic::ThreadedProcessor<F>::processBuffer, this);
    }
}

template <typename F>
void ThreadedProcessor<F>::stop()
{
    should_stop_ = true;
    if (processing_thread_.joinable()) processing_thread_.join();
}

template <typename F>
bool ThreadedProcessor<F>::isStopped() const
{
    return should_stop_;
}

template <typename F>
int ThreadedProcessor<F>::generateProcessID() const
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, std::numeric_limits<int>::max());
    return dis(gen);
}
} // namespace Scenic
