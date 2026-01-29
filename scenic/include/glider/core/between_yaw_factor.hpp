/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About a between yaw factor 
*/


#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

class BetweenYawFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> 
{
    private:
        double measured_delta_yaw_;

    public:
        BetweenYawFactor(gtsam::Key key1, gtsam::Key key2, double delta_yaw, const gtsam::SharedNoiseModel& model)
            : NoiseModelFactor2(model, key1, key2), measured_delta_yaw_(delta_yaw) {}

        gtsam::Vector evaluateError(const gtsam::Pose3& pose1,
                                    const gtsam::Pose3& pose2,
                                    boost::optional<gtsam::Matrix&> H1 = boost::none,
                                    boost::optional<gtsam::Matrix&> H2 = boost::none) const override 
        {
            double yaw1 = pose1.rotation().yaw();
            double yaw2 = pose2.rotation().yaw();
            
            double predicted_delta = yaw2 - yaw1;
            while (predicted_delta > M_PI) predicted_delta -= 2.0 * M_PI;
            while (predicted_delta < -M_PI) predicted_delta += 2.0 * M_PI;
            
            double err = predicted_delta - measured_delta_yaw_;
            while (err > M_PI) err -= 2.0 * M_PI;
            while (err < -M_PI) err += 2.0 * M_PI;
            
            if (H1)
            {
                gtsam::Matrix3 R1 = pose1.rotation().matrix();
                double r10 = R1(1, 0);
                double r00 = R1(0, 0);
                double denom = r00 * r00 + r10 * r10;
                double dyaw_dr10 = r00 / denom;
                double dyaw_dr00 = -r10 / denom;
                
                Eigen::Vector3d dr10_domega(0.0, R1(1, 2), -R1(1, 1));
                Eigen::Vector3d dr00_domega(0.0, R1(0, 2), -R1(0, 1));
                Eigen::Vector3d dyaw1_domega = dyaw_dr10 * dr10_domega + dyaw_dr00 * dr00_domega;
                
                *H1 = (gtsam::Matrix(1, 6) << dyaw1_domega(0), dyaw1_domega(1), dyaw1_domega(2), 
                                              0.0, 0.0, 0.0).finished();
            }
            
            if (H2)
            {
                gtsam::Matrix3 R2 = pose2.rotation().matrix();
                double r10 = R2(1, 0);
                double r00 = R2(0, 0);
                double denom = r00 * r00 + r10 * r10;
                double dyaw_dr10 = r00 / denom;
                double dyaw_dr00 = -r10 / denom;
                
                Eigen::Vector3d dr10_domega(0.0, R2(1, 2), -R2(1, 1));
                Eigen::Vector3d dr00_domega(0.0, R2(0, 2), -R2(0, 1));
                Eigen::Vector3d dyaw2_domega = dyaw_dr10 * dr10_domega + dyaw_dr00 * dr00_domega;
                
                *H2 = (gtsam::Matrix(1, 6) << -dyaw2_domega(0), -dyaw2_domega(1), -dyaw2_domega(2), 
                                              0.0, 0.0, 0.0).finished();
            }
            
            return (gtsam::Vector(1) << err).finished();
        }
};
