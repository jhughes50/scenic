#pragma once

#include <Eigen/Core>
#include <deque>
#include <optional>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "stickyvo/types.hpp"
#include "stickyvo/track_db.hpp"
#include "stickyvo/vo_frontend.hpp"

namespace stickyvo {

// Simplified match structure passed from LGS frontend
struct PairMatchesLite {
  std::vector<Eigen::Vector2d> kpts0_px;
  std::vector<Eigen::Vector2d> kpts1_px;
  std::vector<Eigen::Vector2i> matches; // (index0, index1)
  int num_inliers_points;

  std::vector<Line2D> lines0_px;
  std::vector<Line2D> lines1_px;
  std::vector<Eigen::Vector2i> line_matches;
  int num_inliers_lines;

  double score;

  PairMatchesLite() : num_inliers_points(0), num_inliers_lines(0), score(0.0) {}
};

/**
 * StickyVoCore manages the high-level VO state machine.
 * Consolidates tracking, keyframing, and landmark management.
 */
class StickyVoCore {
public:
  struct Params {
    int min_inliers;
    double ransac_thresh_px;
    int min_tracked_features;
    double min_keyframe_parallax_deg;
    int max_frames_between_keyframes;
    double tri_min_parallax_deg;
    double tri_max_reproj_err_px;
    int map_max_keyframes;
    int pose_history_max;
    double ba_max_cost_increase;
    double ba_max_motion_thresh;
    double pnp_max_motion_thresh;

    Params() = default;
    Params(const std::string& path)
    {
        try {
            YAML::Node config = YAML::LoadFile(path);
            min_inliers = config["sticky"]["min_inliers"].as<int>();
            ransac_thresh_px = config["sticky"]["ransac_thresh_px"].as<double>();
            min_tracked_features = config["sticky"]["min_tracked_features"].as<int>();
            min_keyframe_parallax_deg = config["sticky"]["min_keyframe_parallax_deg"].as<double>();
            max_frames_between_keyframes = config["sticky"]["max_frames_between_keyframes"].as<int>();
            map_max_keyframes = config["sticky"]["map_max_keyframes"].as<int>();
            tri_min_parallax_deg = config["sticky"]["tri_min_parallax_deg"].as<double>();
            tri_max_reproj_err_px = config["sticky"]["tri_max_reproj_err_px"].as<double>();
            ba_max_motion_thresh = config["sticky"]["ba_max_motion_threshold"].as<double>();
            ba_max_cost_increase = config["sticky"]["ba_max_cost_increase"].as<double>();
            pnp_max_motion_thresh = config["sticky"]["pnp_max_motion_thresh"].as<double>();
            pose_history_max = config["sticky"]["pose_history_max"].as<int>();

        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Error loading YAML File at : " + path + " : " + std::string(e.what()));
        } catch (const std::exception& e) {
            throw std::runtime_error("Error parsing YAML configuration at: " + path + " : " + std::string(e.what()));
        }
    }

    static Params Load(const std::string& path)
    {
        Params p(path);
        return p;
    }

  };

  explicit StickyVoCore(const Params& p = Params());
  void reset();

  /**
   * Main entry point: ingest a pair of matched frames and update pose.
   * Returns true if tracking followed by potential keyframe promotion succeeded.
   */
  bool process_and_update(double t0, double t1,
                        const CameraIntrinsics& K,
                        const PairMatchesLite& pm,
                        const std::optional<Quat>& q_wc_prior = std::nullopt);

  const VoState& state() const { return state_; }
  const std::deque<VoSample>& vo_history() const { return vo_hist_; }
  bool is_bootstrapped() const { return bootstrapped_; }

private:
  Params p_;
  VoState state_;
  std::deque<VoSample> vo_hist_;

  static Eigen::Vector2d norm_from_px(const Eigen::Vector2d& u, const CameraIntrinsics& K);
  static std::optional<size_t> nearest_vo_index(double t_sec, const std::deque<VoSample>& hist);

  stickyvo::TrackDB trackdb_;
  stickyvo::VoFrontend frontend_;
  bool bootstrapped_ = false;
  stickyvo::FrameId frame_id_ = 0;
};

}  // namespace stickyvo
