#ifndef POSE_UKF_HPP
#define POSE_UKF_HPP

#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>


class PoseUKF {
public:
    using Vec7 = Eigen::Matrix<double, 7, 1>;
    using Mat7 = Eigen::Matrix<double, 7, 7>;

    // 생성자: Q, R 스케일 설정 가능
    PoseUKF(double q_pos, double q_ori, double r_pos, double r_ori);

    void predict(double dt);  // (선택적 motion model)
    void update(const Vec7& measurement);
    Vec7 getState() const;

    // Auto tunning
    void autoTune(const std::vector<Vec7>& measurements);

private:
    Vec7 x_;    // [x y z qx qy qz qw]
    Mat7 P_;    // Covariance
    Mat7 Q_;    // Process noise
    Mat7 R_;    // Measurement noise

    // Sigma points (UKF 확장 가능 시 사용)
    std::vector<Vec7> generateSigmaPoints(const Vec7& mean, const Mat7& cov);
    Vec7 recoverMean(const std::vector<Vec7>& sigmaPoints, const Eigen::VectorXd& weights);

    // Quaternion tools
    Eigen::Quaterniond vecToQuat(const Vec7& v);
    Vec7 quatToVec(const Eigen::Quaterniond& q, const Eigen::Vector3d& pos);
    Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& quats, const Eigen::VectorXd& weights);
};

#endif
