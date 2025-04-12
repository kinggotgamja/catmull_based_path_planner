#include "catmull_uniform_path_planner/catmull_path.h"

/*** Pose path ***/

/* Catmull-Rom Spline */
// t: 네 개의 위치 중, p1에서 p2까지의 곡선상 어느 지점인지 결정 (예시 t=0.5는 p1과 p2의 중간)
Eigen::Vector3d catmull_path::catmullRomPosition(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, double t) {
    return 0.5 * (
        2.0 * p1 +
        (-p0 + p2) * t +
        (2.0*p0 - 5.0*p1 + 4.0*p2 - p3) * t * t +
        (-p0 + 3.0*p1 - 3.0*p2 + p3) * t * t * t
    );
}

/* Spherical Linear Interpolation (SLERP) */
// t: 회전 q1에서 q2까지 중간 회전의 어느 지점인지 결정 (예시 t=0.5는 q1과 q2의 중간 회전)
Eigen::Quaterniond catmull_path::slerpQuat(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double t) {
    return q1.slerp(t, q2);
}

/* Step1 : Catmull-Rom & SLERP로 곡선에서의 pose를 매우 조밀하게 보간 (reference point 사이를 1/t_step 개수로 보간) */
std::vector<Pose> catmull_path::generateFineCatmullPosePath(const std::vector<Pose>& poses, double t_step) {
    std::vector<Pose> fine_path;
    for (size_t i = 1; i < poses.size() - 2; ++i) {
        for (double t = 0; t <= 1.0; t += t_step) {
            Eigen::Vector3d pos = catmullRomPosition(
                poses[i - 1].position, poses[i].position,
                poses[i + 1].position, poses[i + 2].position, t
            );
            Eigen::Quaterniond ori = slerpQuat(poses[i].orientation, poses[i + 1].orientation, t);
            fine_path.push_back({pos, ori});
        }
    }
    return fine_path;
}

/* Step2 : 조밀한 포인트들의 누적 거리 계산 */
std::vector<double> catmull_path::computeArcLength(const std::vector<Pose>& path) {
    std::vector<double> lengths(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i) {
        lengths[i] = lengths[i - 1] + (path[i].position - path[i - 1].position).norm();
    }
    return lengths;
}

/* Step3 : 일정 거리 간격으로 균일하게 추출 */
std::vector<Pose> catmull_path::resamplePosePath(const std::vector<Pose>& path, const std::vector<double>& lengths, double interval) {
    std::vector<Pose> result;
    if (path.empty()) return result;

    double total_length = lengths.back();
    double target_dist = 0.0;
    size_t idx = 0;

    while (target_dist <= total_length && idx < lengths.size() - 1) {
        while (idx < lengths.size() - 1 && lengths[idx + 1] < target_dist)
            idx++;

        double d1 = lengths[idx];
        double d2 = lengths[idx + 1];
        double ratio = (d2 - d1 == 0) ? 0.0 : (target_dist - d1) / (d2 - d1);;

        Eigen::Vector3d pos = path[idx].position + ratio * (path[idx + 1].position - path[idx].position);
        Eigen::Quaterniond ori = slerpQuat(path[idx].orientation, path[idx + 1].orientation, ratio);

        result.push_back({pos, ori});
        target_dist += interval;
    }

    return result;
}

std::vector<Pose> catmull_path::generateUniformCatmullPosePath(const std::vector<Pose>& control_poses, double distance_interval) {
    auto fine_path = generateFineCatmullPosePath(control_poses,0.1); // VR의 point가 상당히 연속적이므로 기존 0.01 에서 0.1로 변경
    auto arc_lengths = computeArcLength(fine_path);
    return resamplePosePath(fine_path, arc_lengths, distance_interval);
}

/*** Power path ***/

/* Catmull-Rom Spline */
// t: 네 개의 위치 중, p1에서 p2까지의 곡선상 어느 지점인지 결정 (예시 t=0.5는 p1과 p2의 중간)
Eigen::Vector3d catmull_power_path::catmullRomForce(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, double t) {
    return 0.5 * (
        2.0 * p1 +
        (-p0 + p2) * t +
        (2.0*p0 - 5.0*p1 + 4.0*p2 - p3) * t * t +
        (-p0 + 3.0*p1 - 3.0*p2 + p3) * t * t * t
    );
}

/* Catmull-Rom & SLERP로 곡선에서의 pose를 매우 조밀하게 보간 (reference point 사이를 1/t_step 개수로 보간) */
std::vector<Power> catmull_power_path::generateFineCatmullPowerPath(const std::vector<Power>& powers, double t_step) {
    std::vector<Power> fine_power_path;

    for (size_t i = 1; i < powers.size() - 2; ++i) {
        for (double t = 0; t <= 1.0; t += t_step) {

            /* Position interpolation */
            Eigen::Vector3d pos = catmullRomPosition(
                powers[i - 1].position, powers[i].position,
                powers[i + 1].position, powers[i + 2].position, t
            );

            /* Orientation interpolation */
            Eigen::Quaterniond ori = slerpQuat(powers[i].orientation, powers[i + 1].orientation, t);

            /* Force interpolation */
            Eigen::Vector3d force = catmullRomForce(
                powers[i - 1].force, powers[i].force,
                powers[i + 1].force, powers[i + 2].force, t
            );

            /* Data update*/
            fine_power_path.push_back({pos, ori, force});
        }
    }
    return fine_power_path;
}

/* 일정 거리 간격으로 균일하게 추출 */
std::vector<Power> catmull_power_path::resamplePowerPath(const std::vector<Power>& power_path, const std::vector<double>& lengths, double interval) {
    std::vector<Power> result;
    if (power_path.empty()) return result;

    double total_length = lengths.back();
    double target_dist = 0.0;
    size_t idx = 0;

    while (target_dist <= total_length && idx < lengths.size() - 1) {
        while (idx < lengths.size() - 1 && lengths[idx + 1] < target_dist)
            idx++;

        double d1 = lengths[idx];
        double d2 = lengths[idx + 1];
        double ratio = (d2 - d1 == 0) ? 0.0 : (target_dist - d1) / (d2 - d1);;

        Eigen::Vector3d pos = power_path[idx].position + ratio * (power_path[idx + 1].position - power_path[idx].position);
        Eigen::Quaterniond ori = slerpQuat(power_path[idx].orientation, power_path[idx + 1].orientation, ratio);
        Eigen::Vector3d force = power_path[idx].force + ratio * (power_path[idx + 1].force - power_path[idx].force);

        result.push_back({pos, ori, force});
        target_dist += interval;
    }

    return result;
}

std::vector<Power> catmull_power_path::generateUniformCatmullPowerPath(const std::vector<Power>& control_powers, int downsample_rate, double distance_interval) {    
    
    /* Data down sizing */
    std::vector<Power> downsampled_powers;

    for (size_t i = 0; i < control_powers.size(); i+= downsample_rate) {
        downsampled_powers.push_back(control_powers[i]);
    }
    ROS_INFO_STREAM("Downsampled power path size: " << downsampled_powers.size());
    if (downsampled_powers.size() < 4) {
        ROS_ERROR("Need at least 4 control points for Catmull-Rom interpolation");
        return {};
    }

    ROS_INFO_STREAM("Generating Catmull-Rom path with resolution" << distance_interval);
    auto fine_power_path = generateFineCatmullPowerPath(downsampled_powers,1/(double)downsample_rate); // VR의 point가 상당히 연속적이므로 기존 0.01 에서 0.1로 변경
    
    std::vector<Pose> fine_path;
    for (const auto& power : fine_power_path) {
        fine_path.push_back({power.position, power.orientation});
    }

    ROS_INFO_STREAM("Fine path size: " << fine_path.size());
    auto arc_lengths = computeArcLength(fine_path);

    ROS_INFO_STREAM("Arc lengths size: " << arc_lengths.size());
    return resamplePowerPath(fine_power_path, arc_lengths, distance_interval);
}

std::vector<Power> catmull_power_path::generateSmoothCatmullPowerPath(const std::vector<Power>& control_powers, int downsample_rate)
{
    /* Data down sizing */
    std::vector<Power> downsampled_powers;

    for (size_t i = 0; i < control_powers.size(); i+= downsample_rate) {
        downsampled_powers.push_back(control_powers[i]);
    }

    if (downsampled_powers.size() < 4) {
        ROS_ERROR("Need at least 4 control points for Catmull-Rom interpolation");
        return {};
    }

    /* Refill the gap by down sizing */
    auto fine_power_path = generateFineCatmullPowerPath(downsampled_powers,1/(double)downsample_rate);

    return fine_power_path;
}