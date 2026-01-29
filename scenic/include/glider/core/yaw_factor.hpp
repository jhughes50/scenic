/*!
* @Author Jason Hughes
* @Date January 2026
*
* @About a factor for Yaw 
*/
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

class YawFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    double measured_yaw_;

    public:
        YawFactor(gtsam::Key key, double yaw, const gtsam::SharedNoiseModel& model) : NoiseModelFactor1(model, key), measured_yaw_(yaw) {}

        gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H = boost::none) const override 
        {
            gtsam::Matrix3 R = pose.rotation().matrix();
            
            double r10 = R(1, 0);
            double r00 = R(0, 0);
            double yaw = std::atan2(r10, r00);
            
            if (H) 
            {
                double denom = r00 * r00 + r10 * r10;
                double dyaw_dr10 = r00 / denom;
                double dyaw_dr00 = -r10 / denom;
                
                Eigen::Vector3d dr10_domega(0.0, R(1, 2), -R(1, 1));
                Eigen::Vector3d dr00_domega(0.0, R(0, 2), -R(0, 1));
                
                Eigen::Vector3d dyaw_domega = dyaw_dr10 * dr10_domega + dyaw_dr00 * dr00_domega;
                
                // Negate the Jacobian
                *H = (gtsam::Matrix(1, 6) << -dyaw_domega(0), -dyaw_domega(1), -dyaw_domega(2), 0.0, 0.0, 0.0).finished();
            }
            
            double err = yaw - measured_yaw_;
            while (err > M_PI) err -= 2.0 * M_PI;
            while (err < -M_PI) err += 2.0 * M_PI;
            
            return (gtsam::Vector(1) << err).finished();
        } 
            
        // For GTSAM factor graph printing/debugging
        void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "YawFactor on " << keyFormatter(this->key()) << " with measurement " << measured_yaw_ << std::endl;
        this->noiseModel_->print("  noise model: ");
        }
        
        bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override {
        const YawFactor* e = dynamic_cast<const YawFactor*>(&expected);
        return e != nullptr && gtsam::NoiseModelFactor1<gtsam::Pose3>::equals(*e, tol) &&
               std::abs(measured_yaw_ - e->measured_yaw_) < tol;
        }
};
