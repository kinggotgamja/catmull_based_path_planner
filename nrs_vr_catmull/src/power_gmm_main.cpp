#include <ros/ros.h>
#include "catmull_uniform_path_planner/catmull_path.h"
#include <fstream>
#include <sstream>
#include <string>


void Pose_GMM_Handling(const std::string& input_filename, const std::string& PoseGMM_filename, const double force_threshold, const double desired_force)
{

    std::ifstream infile(input_filename);
    std::ofstream PoseGMM_file(PoseGMM_filename);
    std::string line;

    /* Open Files */
    if (!infile.is_open()) {
        ROS_ERROR_STREAM("Failed to open input file: " << input_filename);
    }
    if (!PoseGMM_file.is_open()) {
        ROS_ERROR_STREAM("Failed to open output file: " << PoseGMM_filename);
    }

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double x, y, z, roll, pitch, yaw, fx, fy, fz;
        if (!(iss >> x >> y >> z >> roll >> pitch >> yaw >> fx >> fy >> fz)) {
            ROS_WARN_STREAM("Invalid line, skipping: " << line);
            continue;
        }
        Eigen::Vector3d force(fx, fy, fz);

        if (force.norm() < force_threshold) {
            force = {0, 0, 0};
        }
        else
        {
            force = {0, 0, desired_force};
        }

        PoseGMM_file << x << " "
        << y << " "
        << z << " "
        << roll << " " << pitch << " " << yaw << " "
        << force(0) << " "
        << force(1) << " "
        << force(2) << std::endl;
    }
}

void Power_GMM_Handling(const std::string& input_filename, const std::string& PoseGMM_filename, const double force_threshold, const double desired_force)
{

    std::ifstream infile(input_filename);
    std::ofstream PoseGMM_file(PoseGMM_filename);
    std::string line;

    /* Open Files */
    if (!infile.is_open()) {
        ROS_ERROR_STREAM("Failed to open input file: " << input_filename);
    }
    if (!PoseGMM_file.is_open()) {
        ROS_ERROR_STREAM("Failed to open output file: " << PoseGMM_filename);
    }

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double x, y, z, roll, pitch, yaw, fx, fy, fz;
        if (!(iss >> x >> y >> z >> roll >> pitch >> yaw >> fx >> fy >> fz)) {
            ROS_WARN_STREAM("Invalid line, skipping: " << line);
            continue;
        }
        Eigen::Vector3d force(fx, fy, fz);

        if (force.norm() < force_threshold) {
            force = {0, 0, 0};
        }
        else
        {
            force = {0, 0, fabs(fz)};
        }

        PoseGMM_file << x << " "
        << y << " "
        << z << " "
        << roll << " " << pitch << " " << yaw << " "
        << force(0) << " "
        << force(1) << " "
        << force(2) << std::endl;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "power_gmm_node");
    ros::NodeHandle nh;

    std::string resample_path, PoseGMM_path, PowerGMM_path;
    std::vector<double> force_override_vec = {0.0, 0.0, 0.0};
    std::vector<double> auto_override_force_vec = {0.0, 0.0, 0.0};
    double force_threshold = 10.0;
    bool des_par_auto_tune = false;

    if (!nh.getParam("resample_path_filepath", resample_path)) {
        ROS_ERROR("Failed to get param 'resample_path_filepath'");
        return 1;
    }
    if (!nh.getParam("posegmm_path_filepath", PoseGMM_path)) {
        ROS_ERROR("Failed to get param 'posegmm_path_filepath'");
        return 1;
    }
    if (!nh.getParam("powergmm_path_filepath", PowerGMM_path)) {
        ROS_ERROR("Failed to get param 'powergmm_path_filepath'");
        return 1;
    }

    nh.param("des_par_auto_tune", des_par_auto_tune, false);
    nh.getParam("override_force", force_override_vec);
    nh.getParam("auto_override_force", auto_override_force_vec);
    nh.param("force_threshold", force_threshold, force_threshold);

    Pose_GMM_Handling(resample_path, PoseGMM_path, force_threshold, ((double)!des_par_auto_tune)*force_override_vec[2] + ((double)des_par_auto_tune)*auto_override_force_vec[2]);
    Power_GMM_Handling(resample_path, PowerGMM_path, force_threshold, ((double)!des_par_auto_tune)*force_override_vec[2] + ((double)des_par_auto_tune)*auto_override_force_vec[2]);


    return 0;
}