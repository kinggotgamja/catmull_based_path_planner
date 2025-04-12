#include <ros/ros.h>
#include "catmull_uniform_path_planner/catmull_path.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "catmull_power_example");
    ros::NodeHandle nh;

    /* Define control points for the Catmull-Rom spline */
    std::vector<Power> control_powers = {
        {Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity(),Eigen::Vector3d(0, 0, 0)},
        {Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity(),Eigen::Vector3d(0, 0, 0)},
        {Eigen::Vector3d(0.1, 0, 0), Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ())),Eigen::Vector3d(0, 0, 1)},
        {Eigen::Vector3d(0.2, 0, 0), Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())),Eigen::Vector3d(0, 0, 1)},
        {Eigen::Vector3d(0.3, 0, 0), Eigen::Quaterniond(Eigen::AngleAxisd(3*M_PI/4, Eigen::Vector3d::UnitZ())),Eigen::Vector3d(0, 0, 1)},
        {Eigen::Vector3d(0.4, 0, 0), Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())),Eigen::Vector3d(0, 0, 0)},
        {Eigen::Vector3d(0.4, 0, 0), Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())),Eigen::Vector3d(0, 0, 0)}
    };

    /* Generate a fine path using Catmull-Rom splines */ 
    auto path = catmull_power_path::generateUniformCatmullPowerPath(control_powers, 1, 0.1); // downsample 1, 0.1 m interval

    /* Print the generated path */
    for(const auto& power : path) {
        std::cout << "Position: " << power.position.transpose() << ", Orientation: " << power.orientation.coeffs().transpose() << ", Force: " << power.force.transpose() << std::endl;
    }
    
    return 0;
}
