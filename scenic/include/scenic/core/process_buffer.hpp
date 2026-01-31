/*
* Jason Hughes
* June 2025
*
* Image Buffer
*
*/

#pragma once

#include <vector>
#include <mutex>
#include <memory>
#include <stdexcept>


namespace Scenic
{

enum class Access
{
    LOCK,
    PRELOCK,
    UNLOCK
};

template <typename F>
class ProcessBuffer
{
    public:
        ProcessBuffer() = default;
        ProcessBuffer(size_t capacity);

        void push(const F frame, Access a = Access::LOCK);
        //void push(const std::shared_ptr<F> frame, Access a = Acess::LOCK);    
        // get oldest element
        std::unique_ptr<F> pop(Access a = Access::LOCK);
        // remove the oldest frame
        void del(Access a = Access::LOCK);
        // peek at oldest element without removing it
        F front(Access a = Access::LOCK) const;
        // peek at newest element without removing it
        F back(Access a = Access::LOCK) const;
        // get element at index
        F at(size_t idx, Access a = Access::LOCK) const;
        // get an immutable reference at index
        const F& referenceAt(size_t idx, Access a = Access::LOCK) const;
        // does the buffer contain an element
        bool contains(F frame, Access a = Access::LOCK) const;

        size_t size(Access a = Access::LOCK) const;
        size_t capacity() const;
        
        bool empty(Access a = Access::LOCK) const;
        bool full(Access a = Access::LOCK) const;
        
        void clear(Access a = Access::LOCK);
        void replace(size_t idx, Access a = Access::LOCK);

    protected:
        mutable std::mutex mutex_;

    private:
        std::vector<std::unique_ptr<F>> buffer_;
        size_t capacity_;
        size_t head_;
        size_t tail_;
        size_t size_;
};

template <typename F>
ProcessBuffer<F>::ProcessBuffer(size_t capacity) : buffer_(capacity)
{
    capacity_ = capacity;
    head_ = 0;
    tail_ = 0;
    size_ = 0;

    if (capacity == 0) throw std::invalid_argument("Buffer capacity must be greater than 0");
}

template <typename F>
void ProcessBuffer<F>::push(const F frame, Access a)
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    buffer_[tail_] = std::make_unique<F>(frame);
    tail_ = (tail_ + 1) % capacity_;

    if (size_ < capacity_)
    {
        ++size_;
    }
    else
    {
        head_ = (head_ + 1) % capacity_;
    }
}

template <typename F>
std::unique_ptr<F> ProcessBuffer<F>::pop(Access a)
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    if (size_ == 0) {
        throw std::runtime_error("Buffer is empty");
    }

    std::unique_ptr<F> result = std::move(buffer_[head_]);
    buffer_[head_].reset();
    head_ = (head_ + 1) % capacity_;
    --size_;

    return result;
}

template <typename F>
void ProcessBuffer<F>::del(Access a)
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    if (size_ == 0) {
        throw std::runtime_error("Buffer is empty");
    }

    F result = *buffer_[head_];
    buffer_[head_].reset();
    head_ = (head_ + 1) % capacity_;
    --size_;
}

template <typename F>
F ProcessBuffer<F>::front(Access a) const
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    if (size_ == 0) {
        throw std::runtime_error("Buffer is empty");
    }

    return *buffer_[head_];
}

template <typename F>
F ProcessBuffer<F>::back(Access a) const
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    if (size_ == 0) {
        throw std::runtime_error("Buffer is empty");
    }

    size_t back_idx = (tail_ + capacity_ - 1) % capacity_;

    return *buffer_[back_idx];
}

template <typename F>
F ProcessBuffer<F>::at(size_t index, Access a) const
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    if (index >= size_) throw std::out_of_range("Index out of range");

    size_t actual_index = (head_ + index) % capacity_;

    return *buffer_[actual_index];
}

template <typename F>
const F& ProcessBuffer<F>::referenceAt(size_t index, Access a) const
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    if (index >= size_) throw std::out_of_range("Index out of range");

    size_t actual_index = (head_ + index) % capacity_;

    return *buffer_[actual_index];
}

template <typename F>
size_t ProcessBuffer<F>::size(Access a) const
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    return size_;
}

template <typename F>
size_t ProcessBuffer<F>::capacity() const
{
    return capacity_;
}

template <typename F>
bool ProcessBuffer<F>::empty(Access a) const
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();
    
    return size_ == 0;
}

template <typename F>
bool ProcessBuffer<F>::full(Access a) const
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();
   
    return size_ == capacity_;
}

template <typename F>
void ProcessBuffer<F>::clear(Access a)
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();

    for (size_t i = 0; i < capacity_; ++i) {
        buffer_[i] = nullptr;
    }
    head_ = 0;
    tail_ = 0;
    size_ = 0;
}

template <typename F>
bool ProcessBuffer<F>::contains(F frame, Access a) const
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    if (a == Access::LOCK) lock.lock();
    
    auto it = std::find_if(buffer_.begin(), buffer_.end(),
        [&frame](const auto& ptr) {
            return ptr && *ptr == frame;
        });
    
    return it != buffer_.end();
}
}

