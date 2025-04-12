#pragma once
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>

struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

class catmull_path
{
    public:
        catmull_path() {};
        ~catmull_path() {};

        static Eigen::Vector3d catmullRomPosition(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, double t);
        static Eigen::Quaterniond slerpQuat(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double t);
        static std::vector<Pose> generateFineCatmullPosePath(const std::vector<Pose>& poses, double t_step=0.01);
        static std::vector<double> computeArcLength(const std::vector<Pose>& path);
        static std::vector<Pose> resamplePosePath(const std::vector<Pose>& path, const std::vector<double>& lengths, double interval);
        static std::vector<Pose> generateUniformCatmullPosePath(const std::vector<Pose>& control_poses, double distance_interval = 0.05);

    private:
        // Private members can be added here if needed
};

struct Power {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d force;
};

class catmull_power_path : public catmull_path
{
    public:
        catmull_power_path() {};
        ~catmull_power_path() {};

        static Eigen::Vector3d catmullRomForce(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, double t);
        static std::vector<Power> generateFineCatmullPowerPath(const std::vector<Power>& powers, double t_step=0.01);
        static std::vector<Power> resamplePowerPath(const std::vector<Power>& power_path, const std::vector<double>& lengths, double interval);
        static std::vector<Power> generateUniformCatmullPowerPath(const std::vector<Power>& control_powers, int downsample_rate=1, double distance_interval= 0.05);
        static std::vector<Power> generateSmoothCatmullPowerPath(const std::vector<Power>& control_powers, int downsample_rate=1);

    private:
        // Private members can be added here if needed
};
