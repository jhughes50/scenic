/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About an unordered map with a fixed size
*/

#pragma once

#include <unordered_map>

namespace Scenic
{
template<typename K, typename V>
class FixedHashMap {
    private:
        std::unordered_map<K, V> data_;
        std::deque<K> order_;
        size_t max_size_{10};
        
    public:
        FixedHashMap() = default;
        //FixedHashMap(size_t max_size = 10) : max_size_(max_size) {}
        
        void setCapacity(size_t new_max_size) 
        {
            max_size_ = new_max_size;
        }
        
        void insert(const K& key, const V& value) 
        {
            // If key already exists, update it
            if (data_.find(key) != data_.end()) {
                data_[key] = value;
                return;
            }
            
            // If we're at capacity, remove the oldest entry
            if (data_.size() >= max_size_) {
                K oldest_key = order_.front();
                order_.pop_front();
                data_.erase(oldest_key);
            }
            
            // Add new entry
            data_[key] = value;
            order_.push_back(key);
        }
        
        const V& get(const K& key) const
        {
            auto it = data_.find(key);
            if (it != data_.end()) {
                return it->second;
            }
            throw std::out_of_range("Key not found");
        }
        
        size_t size() const { return data_.size(); }
        bool empty() const { return data_.empty(); }
        
        void erase(const K& key) 
        {
            data_.erase(key);
            auto it = std::find(order_.begin(), order_.end(), key);
            if (it != order_.end()) {
                order_.erase(it);
            }
        }
};
}
