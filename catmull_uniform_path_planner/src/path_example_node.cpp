#include <ros/ros.h>
#include "catmull_uniform_path_planner/catmull_path.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "catmull_example");
    ros::NodeHandle nh;

    /* Define control points for the Catmull-Rom spline */
    std::vector<Pose> control_poses = {
        {Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity()},
        {Eigen::Vector3d(1, 2, 0), Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()))},
        {Eigen::Vector3d(2, 0, 0), Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()))},
        {Eigen::Vector3d(3, 2, 0), Eigen::Quaterniond(Eigen::AngleAxisd(3*M_PI/4, Eigen::Vector3d::UnitZ()))},
        {Eigen::Vector3d(4, 0, 0), Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()))}
    };

    /* Generate a fine path using Catmull-Rom splines */ 
    auto path = catmull_path::generateUniformCatmullPosePath(control_poses, 0.1); // 0.1 m interval

    for (const auto& pose : path) {
        ROS_INFO_STREAM("Position: " << pose.position.transpose());
    }
    
    return 0;
}
