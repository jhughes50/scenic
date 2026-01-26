/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About 
*
*/

#include "scenic/core/buffer_search_coords.hpp"

using namespace Scenic;

BufferSearchCoordinates::BufferSearchCoordinates(const cv::Mat& coords, UTMPoint target)
    : coords_(coords), 
      target_(target),
      row_(std::make_shared<std::atomic<int>>(-1)),
      col_(std::make_shared<std::atomic<int>>(-1)),
      found_(std::make_shared<std::atomic<bool>>(false)) {
}

//BufferSearchCoordinates::BufferSearchCoordinates(const BufferSearchCoordinates& other)
//    : coords_(other.coords_), 
//      target_(other.target_),
//      tolerance_(other.tolerance_),
//      row_(other.row_.load()),
//      col_(other.col_.load()),
//      found_(other.found_.load()) {
//}

void BufferSearchCoordinates::operator()(const tbb::blocked_range2d<int>& range) const
{
    if (found_->load()) return;  // Early exit if already found
    
    for (int r = range.rows().begin(); r != range.rows().end(); ++r) {
        if (found_->load()) return;  // Check again in inner loop
        
        for (int c = range.cols().begin(); c != range.cols().end(); ++c) {
            cv::Vec2d pixel = coords_.at<cv::Vec2d>(r, c);
            double easting = pixel[0];
            double northing = pixel[1];
            
            if (std::abs(easting - target_.easting) < tolerance_ && std::abs(northing - target_.northing) < tolerance_) {
                row_->store(r);
                col_->store(c);
                found_->store(true);
                return;
            }
        }
    }
}

std::pair<int, int> BufferSearchCoordinates::search()
{
    tbb::parallel_for(tbb::blocked_range2d<int>(0, coords_.rows, 0, coords_.cols), *this);
    return {row_->load(), col_->load()};
}
