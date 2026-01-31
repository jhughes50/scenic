/*!
* @Author Jason Hughes
* @Date Januray 2026
*
* @About A database for features
*/
#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <map>
#include <opencv2/opencv.hpp>

#include <glider/core/odometry.hpp>
#include "scenic/core/processor_inputs.hpp"

namespace Scenic 
{

enum class ScenicType
{
    Image,
    Odometry,
    Timestamp,
    Features,
    Homography,
    Reprojection,
    Mask,
    Logits,
    Class,
    Level,
    Priority
};

//enum class GraphLevel
//{
//    OBJECT,
//    REGION
//};
//
//enum class RegionPriority
//{
//    HIGH,
//    MEDIUM,
//    LOW,
//    NONE
//};

template <typename V>
using PidMap = std::unordered_map<int, V>;

template <typename V>
using PidNestedMap = std::unordered_map<int, std::map<size_t, V>>;

template <typename V>
using CidMap = std::map<size_t, V>;

enum class StorageKind
{ 
    ByPid, 
    ByPidCid, 
    ByCid 
};

template <ScenicType T>
struct ScenicTypeTraits;

template <> struct ScenicTypeTraits<ScenicType::Image>        { using type = cv::Mat;                  static constexpr StorageKind kind = StorageKind::ByPid; };
template <> struct ScenicTypeTraits<ScenicType::Homography>   { using type = cv::Mat;                  static constexpr StorageKind kind = StorageKind::ByPid; };
template <> struct ScenicTypeTraits<ScenicType::Reprojection> { using type = double;                   static constexpr StorageKind kind = StorageKind::ByPid; }; 
template <> struct ScenicTypeTraits<ScenicType::Features>     { using type = std::vector<cv::DMatch>;  static constexpr StorageKind kind = StorageKind::ByPid; };
template <> struct ScenicTypeTraits<ScenicType::Odometry>     { using type = Glider::Odometry;         static constexpr StorageKind kind = StorageKind::ByPid; };
template <> struct ScenicTypeTraits<ScenicType::Mask>         { using type = cv::Mat;                  static constexpr StorageKind kind = StorageKind::ByPidCid; };
template <> struct ScenicTypeTraits<ScenicType::Logits>       { using type = cv::Mat;                  static constexpr StorageKind kind = StorageKind::ByPidCid; };
template <> struct ScenicTypeTraits<ScenicType::Class>        { using type = std::string;              static constexpr StorageKind kind = StorageKind::ByCid; };
template <> struct ScenicTypeTraits<ScenicType::Level>        { using type = GraphLevel;               static constexpr StorageKind kind = StorageKind::ByCid; };
template <> struct ScenicTypeTraits<ScenicType::Priority>     { using type = RegionPriority;           static constexpr StorageKind kind = StorageKind::ByCid; };
template <> struct ScenicTypeTraits<ScenicType::Timestamp>    { using type = int64_t;                  static constexpr StorageKind kind = StorageKind::ByPid; };

template <ScenicType T>
using ScenicT = typename ScenicTypeTraits<T>::type;

template <ScenicType T>
constexpr StorageKind storage_kind = ScenicTypeTraits<T>::kind;

template <ScenicType T>
constexpr bool is_by_pid = (storage_kind<T> == StorageKind::ByPid);

template <ScenicType T>
constexpr bool is_by_pid_cid = (storage_kind<T> == StorageKind::ByPidCid);

template <ScenicType T>
constexpr bool is_by_cid = (storage_kind<T> == StorageKind::ByCid);


class ScenicDatabase
{
    public:
        ScenicDatabase() = default;

        template <ScenicType T>
        std::enable_if_t<is_by_pid<T>> insert(int pid, const ScenicT<T>& entry)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>()[pid] = entry;
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid<T>> insert(int pid, ScenicT<T>&& entry)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>()[pid] = std::move(entry);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid<T>, ScenicT<T>&> at(int pid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().at(pid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid<T>, const ScenicT<T>&> at(int pid) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().at(pid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid<T>, bool> contains(int pid) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto& m = getMap<T>();
            return m.find(pid) != m.end();
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid<T>> erase(int pid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>().erase(pid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>> insert(int pid, size_t cid, const ScenicT<T>& entry)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>()[pid][cid] = entry;
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>> insert(int pid, size_t cid, ScenicT<T>&& entry)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>()[pid][cid] = std::move(entry);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>, ScenicT<T>&> at(int pid, size_t cid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().at(pid).at(cid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>, const ScenicT<T>&> at(int pid, size_t cid) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().at(pid).at(cid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>, std::map<size_t, ScenicT<T>>&> at(int pid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().at(pid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>, const std::map<size_t, ScenicT<T>>&> at(int pid) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().at(pid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>, bool> contains(int pid, size_t cid) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto& m = getMap<T>();
            auto it = m.find(pid);
            return it != m.end() && it->second.find(cid) != it->second.end();
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>, bool> contains(int pid) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().find(pid) != getMap<T>().end();
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>> erase(int pid, size_t cid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = getMap<T>().find(pid);
            if (it != getMap<T>().end()) {
                it->second.erase(cid);
            }
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid_cid<T>> erase(int pid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>().erase(pid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_cid<T>> insert(size_t cid, const ScenicT<T>& entry)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>()[cid] = entry;
        }

        template <ScenicType T>
        std::enable_if_t<is_by_cid<T>> insert(size_t cid, ScenicT<T>&& entry)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>()[cid] = std::move(entry);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_cid<T>, ScenicT<T>&> at(size_t cid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().at(cid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_cid<T>, const ScenicT<T>&> at(size_t cid) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return getMap<T>().at(cid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_cid<T>, bool> contains(size_t cid) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto& m = getMap<T>();
            return m.find(cid) != m.end();
        }

        template <ScenicType T>
        std::enable_if_t<is_by_cid<T>> erase(size_t cid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            getMap<T>().erase(cid);
        }

        template <ScenicType T>
        std::enable_if_t<is_by_pid<T> || is_by_pid_cid<T>, std::vector<int>> getPids() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::vector<int> pids;
            const auto& m = getMap<T>();
            pids.reserve(m.size());
            for (const auto& [pid, _] : m) {
                pids.push_back(pid);
            }
            return pids;
        }

        void addCid(size_t cid)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            class_ids_.push_back(cid);
        }

        std::vector<size_t> getCids() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return class_ids_; 
        }

        void eraseAll(int pid) 
        {
            std::lock_guard<std::mutex> lock(mutex_);
            images_.erase(pid);
            homographies_.erase(pid);
            features_.erase(pid);
            odometries_.erase(pid);
            reprojection_errors_.erase(pid);
            masks_.erase(pid);
            logits_.erase(pid);
            timestamps_.erase(pid);
        }

        size_t memory() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            size_t total = sizeof(*this);

            for (const auto& [pid, mat] : images_) {
                total += sizeof(pid) + mat.total() * mat.elemSize();
            }
            for (const auto& [pid, mat] : homographies_) {
                total += sizeof(pid) + mat.total() * mat.elemSize();
            }
            for (const auto& [pid, vec] : features_) {
                total += sizeof(pid) + vec.capacity() * sizeof(cv::DMatch);
            }
            for (const auto& [pid, odom] : odometries_) {
                total += sizeof(pid) + sizeof(odom);
            }
            for (const auto& [pid, val] : reprojection_errors_) {
                total += sizeof(pid) + sizeof(val);
            }

            for (const auto& [pid, inner] : masks_) {
                for (const auto& [cid, mat] : inner) {
                    total += sizeof(cid) + mat.total() * mat.elemSize();
                }
            }
            for (const auto& [pid, inner] : logits_) {
                for (const auto& [cid, mat] : inner) {
                    total += sizeof(cid) + mat.total() * mat.elemSize();
                }
            }

            for (const auto& [cid, str] : classes_) {
                total += sizeof(cid) + str.capacity();
            }
            for (const auto& [cid, level] : graph_levels_) {
                total += sizeof(cid) + sizeof(level);
            }
            for (const auto& [cid, priority] : region_priorities_) {
                total += sizeof(cid) + sizeof(priority);
            }

            total += class_ids_.capacity() * sizeof(size_t);

            return total;
        }

    private:
        template <ScenicType T> 
        auto& getMap();
        template <ScenicType T> 
        const auto& getMap() const;

        mutable std::mutex mutex_;

        PidMap<cv::Mat> images_;
        PidMap<cv::Mat> homographies_;
        PidMap<std::vector<cv::DMatch>> features_;
        PidMap<Glider::Odometry> odometries_;
        PidMap<double> reprojection_errors_;
        PidMap<int64_t> timestamps_;

        PidNestedMap<cv::Mat> masks_;
        PidNestedMap<cv::Mat> logits_;

        CidMap<std::string> classes_;
        CidMap<GraphLevel> graph_levels_;
        CidMap<RegionPriority> region_priorities_;

        std::vector<size_t> class_ids_;
};

template <> inline auto& ScenicDatabase::getMap<ScenicType::Image>()        { return images_; }
template <> inline auto& ScenicDatabase::getMap<ScenicType::Homography>()   { return homographies_; }
template <> inline auto& ScenicDatabase::getMap<ScenicType::Features>()     { return features_; }
template <> inline auto& ScenicDatabase::getMap<ScenicType::Odometry>()     { return odometries_; }
template <> inline auto& ScenicDatabase::getMap<ScenicType::Reprojection>() { return reprojection_errors_; }
template <> inline auto& ScenicDatabase::getMap<ScenicType::Timestamp>()    { return timestamps_; }

template <> inline auto& ScenicDatabase::getMap<ScenicType::Mask>()   { return masks_; }
template <> inline auto& ScenicDatabase::getMap<ScenicType::Logits>() { return logits_; }

template <> inline auto& ScenicDatabase::getMap<ScenicType::Class>()    { return classes_; }
template <> inline auto& ScenicDatabase::getMap<ScenicType::Level>()    { return graph_levels_; }
template <> inline auto& ScenicDatabase::getMap<ScenicType::Priority>() { return region_priorities_; }

template <> inline const auto& ScenicDatabase::getMap<ScenicType::Image>()        const { return images_; }
template <> inline const auto& ScenicDatabase::getMap<ScenicType::Homography>()   const { return homographies_; }
template <> inline const auto& ScenicDatabase::getMap<ScenicType::Features>()     const { return features_; }
template <> inline const auto& ScenicDatabase::getMap<ScenicType::Odometry>()     const { return odometries_; }
template <> inline const auto& ScenicDatabase::getMap<ScenicType::Reprojection>() const { return reprojection_errors_; }
template <> inline const auto& ScenicDatabase::getMap<ScenicType::Timestamp>()    const { return timestamps_; }

template <> inline const auto& ScenicDatabase::getMap<ScenicType::Mask>()   const { return masks_; }
template <> inline const auto& ScenicDatabase::getMap<ScenicType::Logits>() const { return logits_; }

template <> inline const auto& ScenicDatabase::getMap<ScenicType::Class>()    const { return classes_; }
template <> inline const auto& ScenicDatabase::getMap<ScenicType::Level>()    const { return graph_levels_; }
template <> inline const auto& ScenicDatabase::getMap<ScenicType::Priority>() const { return region_priorities_; }

} // namespace Scenic
