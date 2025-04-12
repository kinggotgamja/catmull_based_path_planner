#include "pose_ukf_filter/PoseUKF.hpp"

PoseUKF::PoseUKF(double q_pos, double q_ori, double r_pos, double r_ori) {
    x_.setZero();
    x_[3] = 1.0;  // quaternion w = 1

    P_.setIdentity();

    Q_.setZero();
    Q_.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity() * q_pos;  // position Q
    Q_.bottomRightCorner<4,4>() = Eigen::Matrix4d::Identity() * q_ori;  // orientation Q

    R_.setZero();
    R_.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity() * r_pos;
    R_.bottomRightCorner<4,4>() = Eigen::Matrix4d::Identity() * r_ori;
}


void PoseUKF::predict(double dt) {
    // This can be expanded with motion model; for now, we assume static
    P_ += Q_;
}

void PoseUKF::update(const Vec7& z) {
    Eigen::Matrix<double, 7, 7> K;
    Eigen::Matrix<double, 7, 1> y = z - x_;
    y.segment<4>(3).normalize(); // Normalize orientation residual
    K = P_ * (P_ + R_).inverse();
    x_ = x_ + K * y;
    x_.segment<4>(3).normalize(); // Normalize quaternion
    P_ = (Mat7::Identity() - K) * P_;
}

PoseUKF::Vec7 PoseUKF::getState() const {
    return x_;
}

void PoseUKF::autoTune(const std::vector<Vec7>& measurements) {
    if (measurements.size() < 2) {
        ROS_WARN("Not enough data for auto-tuning Q and R.");
        return;
    }

    // 계산용 변수
    Mat7 emp_Q = Mat7::Zero();
    Mat7 emp_R = Mat7::Zero();

    Vec7 prev = measurements[0];

    for (size_t i = 1; i < measurements.size(); ++i) {
        Vec7 curr = measurements[i];

        Vec7 process_diff = curr - prev;
        emp_Q += process_diff * process_diff.transpose();

        Vec7 meas_diff = curr - x_;  // 이전 상태와 측정 비교
        emp_R += meas_diff * meas_diff.transpose();

        prev = curr;
    }

    emp_Q /= (measurements.size() - 1);
    emp_R /= (measurements.size() - 1);

    Q_ = emp_Q;
    R_ = emp_R;

    ROS_INFO_STREAM("Auto-tuned Q:\n" << Q_);
    ROS_INFO_STREAM("Auto-tuned R:\n" << R_);
}

