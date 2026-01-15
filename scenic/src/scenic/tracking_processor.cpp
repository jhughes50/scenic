/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About process frames through stickyVO
*/

#include "scenic/core/tracking_processor.hpp"

using namespace Scenic;

TrackingProcessor::TrackingProcessor(size_t capacity, const std::string& rect_path, const std::string& params_path) : ThreadedProcessor<TrackingInput>(capacity)
{
    rectifier_ = Rectifier::Load(rect_path);
    stickyvo::StickyVoCore::Params cop = stickyvo::StickyVoCore::Params::Load(params_path);
    sticky_core_ = std::make_unique<stickyvo::StickyVoCore>(cop);

    cv::Mat k = rectifier_.getIntrinsics<cv::Mat>();
    K_.fx = k.at<double>(0,0);
    K_.fy = k.at<double>(1,1);
    K_.cx = k.at<double>(2,0);
    K_.cy = k.at<double>(2,1);
    
    K_vo_.fx = k.at<double>(0,0);
    K_vo_.fy = k.at<double>(1,1);
    K_vo_.cx = k.at<double>(2,0);
    K_vo_.cy = k.at<double>(2,1);

    try {
        stickyvo_lgs::LgsConfig cfg;
        cfg.python_module = "lgs_py_bridge";
        cfg.python_func = "infer_pair";
        cfg.use_gpu = true;

        lgs_ = std::make_unique<stickyvo_lgs::LgsFrontend>(cfg);
    } catch (const std::exception& e) {
        LOG(FATAL) << "[SCENIC] LGS Construction Failed: %s", e.what();
    }
}

void TrackingProcessor::setCallback(std::function<void(std::shared_ptr<TrackingOutput>)> callback)
{
    outputCallback = callback;
}

void TrackingProcessor::processBuffer()
{
    while (!isStopped()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (size(Access::PRELOCK) >= min_elem_) {
            std::unique_ptr<TrackingInput> raw_input = pop(Access::PRELOCK);
            std::shared_ptr<TrackingOutput> output;

            cv::Mat prev_image = raw_input->prev_image;
            cv::Mat curr_image = raw_input->curr_image;
            stickyvo_lgs::ImageView v0{(uint8_t*)prev_image.data, prev_image.cols, prev_image.rows, (int)prev_image.step, stickyvo_lgs::ImageView::Format::kGray8};
            stickyvo_lgs::ImageView v1{(uint8_t*)curr_image.data, curr_image.cols, curr_image.rows, (int)curr_image.step, stickyvo_lgs::ImageView::Format::kGray8};

            auto matches = lgs_->infer_pair(v0, v1, K_);

            stickyvo::PairMatchesLite pm;
            pm.score = matches.score;
            std::vector<Eigen::Vector2d> p0_px, p1_px;
            for (const auto& k : matches.f0.keypoints) {
                p0_px.emplace_back(k.x, k.y);
            }
            for (const auto& k : matches.f1.keypoints) {
                p1_px.emplace_back(k.x, k.y);
            }

            pm.kpts0_px = rectifier_.undistortPixelPoints(p0_px);
            pm.kpts1_px = rectifier_.undistortPixelPoints(p1_px);

            for (const auto& m : matches.point_matches) {
                pm.matches.emplace_back(m.i0, m.i1);
            }
            
            auto to_sl = [](const stickyvo_lgs::Line2D& l){ 
                return stickyvo::Line2D{{l.p0.x, l.p0.y}, {l.p1.x, l.p1.y}}; 
            };
            std::vector<stickyvo::Line2D> l0_px, l1_px;
            for (const auto& l : matches.f0.lines) {
                l0_px.push_back(to_sl(l));
            }
            for (const auto& l : matches.f1.lines) {
                l1_px.push_back(to_sl(l));
            }

            auto und_lines = [&](const std::vector<stickyvo::Line2D>& in, std::vector<stickyvo::Line2D>& out) {
                std::vector<Eigen::Vector2d> pts, pts_ud;
                for (const auto& l : in) { 
                    pts.push_back(l.p1);
                    pts.push_back(l.p2); 
                }
                pts_ud = rectifier_.undistortPixelPoints(pts);
                for (size_t i=0; i<in.size(); ++i) {
                    out.push_back({pts_ud[2*i], pts_ud[2*i+1]});
                }
            };
            und_lines(l0_px, pm.lines0_px); 
            und_lines(l1_px, pm.lines1_px);
            for (const auto& m : matches.line_matches) {
                pm.line_matches.emplace_back(m.i0, m.i1);
            }

            stickyvo::CameraIntrinsics K_vo = K_vo_;
            K_vo.has_distortion = false;
            const bool ok = sticky_core_->process_and_update(raw_input->prev_stamp, 
                                                             raw_input->curr_stamp, 
                                                             K_vo, 
                                                             pm,
                                                             stickyvo::Quat(qw_, qx_, qy_, qz_));

            if (!ok) {
                if (!sticky_core_->is_bootstrapped()) {
                    bootstrap_fail_count_++;
                    LOG(WARNING) << "[STICKYVO] Bootstrap Failed " << bootstrap_fail_count_ << "/30";
                    if (bootstrap_fail_count_ > 30) {
                        LOG(ERROR) << "[STICKYVO] Bootstrap timed out, restting";
                        sticky_core_->reset();
                        bootstrap_fail_count_ = 0;
                    } else {
                        lgs_->reset_state();
                    }
                } else {
                    tracking_fail_count_++;
                    LOG(WARNING) << "[STICKYVO] Tracking Failed " << tracking_fail_count_ << "/10";
                    if (tracking_fail_count_>10) {
                        LOG(ERROR) << "[STICKYVO] Tracking lost, resetting";
                        sticky_core_->reset();
                        tracking_fail_count_ = 0;
                    }
                }
            } else {
                bootstrap_fail_count_ = 0;
                tracking_fail_count_ = 0;
                const stickyvo::VoState vo_state = sticky_core_->state();
                // make state outpot as shared ptr
                
            }
            if (output) outputCallback(output);
        
        } else {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
