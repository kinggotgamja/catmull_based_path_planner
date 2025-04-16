#include <ros/ros.h>
#include "catmull_uniform_path_planner/catmull_path.h"
#include "pose_ukf_filter/PoseUKF.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    return yawAngle * pitchAngle * rollAngle;
}

void quaternionToRPY(const Eigen::Quaterniond& q, 
    double& roll, double& pitch, double& yaw) {
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
}

std::vector<Power> readAndFilterPowerData(
    const std::string& filename,
    double dt,
    double force_thresh,
    const Eigen::Vector3d& override_force,
    PoseUKF& ukf,
    bool enable_auto_tune,
    std::vector<PoseUKF::Vec7>& measurement_buffer)
{
    std::vector<Power> filtered;
    std::ifstream infile(filename);
    std::string line;

    if (!infile.is_open()) {
        ROS_ERROR_STREAM("Failed to open input file: " << filename);
        return filtered;
    }

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double x, y, z, roll, pitch, yaw, fx, fy, fz;
        if (!(iss >> x >> y >> z >> roll >> pitch >> yaw >> fx >> fy >> fz)) {
            ROS_WARN_STREAM("Invalid line, skipping: " << line);
            continue;
        }

        Eigen::Vector3d pos(x, y, z);
        Eigen::Quaterniond q = rpyToQuaternion(roll, pitch, yaw);

        #if 1 // Use UKF for filtering
        PoseUKF::Vec7 z_vec;
        z_vec.head<3>() = pos;
        z_vec.tail<4>() << q.w(), q.x(), q.y(), q.z();

        if (enable_auto_tune) {
            measurement_buffer.push_back(z_vec);
        }

        ukf.predict(dt);
        ukf.update(z_vec);
        PoseUKF::Vec7 filtered_state = ukf.getState();

        Eigen::Vector3d filtered_pos = filtered_state.segment<3>(0);
        Eigen::Quaterniond filtered_q(filtered_state[6], filtered_state[3], filtered_state[4], filtered_state[5]);

        if (!filtered_pos.allFinite() || !filtered_q.coeffs().allFinite()) {
            ROS_WARN_STREAM("Invalid filtered data at line: " << line);
            continue; // skip this sample
        }
        #endif

        #if 0 // Use simple averaging for filtering (not recommended)
        Eigen::Vector3d filtered_pos(pos[0],pos[1],pos[2]);
        Eigen::Quaterniond filtered_q(q.w(), q.x(), q.y(), q.z());

        #endif

        Eigen::Vector3d force(fx, fy, fz);
        if (force.norm() > force_thresh) {
            force = override_force;
        }
        else
        {
            force = {0,0,0};
        }


        filtered.push_back({filtered_pos, filtered_q.normalized(), force});
    }
    
    ROS_INFO_STREAM("Filtered and loaded " << filtered.size() << " control points.");
    return filtered;
}

void savePowerPathToFile(const std::string& filepath, const std::vector<Power>& path) {
    std::ofstream outfile(filepath);
    if (!outfile.is_open()) {
        ROS_ERROR_STREAM("Failed to open output file: " << filepath);
        return;
    }

    bool init = true;
    double roll, pitch, yaw;

    int removal_size = 500;
    int removal_count = 0;
    for (const auto& power : path) {
        if (removal_count < removal_size) {
            removal_count++;
            continue;
        }
        else
        {
            quaternionToRPY(power.orientation, roll, pitch, yaw);

            /* Special handling for NRS UR10 */
            while(roll<0){roll += 2*M_PI;}
            while(yaw<0){yaw += 2*M_PI;}
    
            roll = M_PI - roll; // Invert roll for the output
            pitch = - pitch;
    
    
            outfile << power.position.x() << " "
                    << power.position.y() << " "
                    << power.position.z() << " "
                    << yaw << " " << pitch << " " << roll << " "
                    << power.force.x() << " "
                    << power.force.y() << " "
                    << power.force.z() << std::endl;
        }
    }

    ROS_INFO_STREAM("Saved regenerated path to " << filepath);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_regen_node");
    ros::NodeHandle nh;

    std::string input_path, resample_path, output_path;
    double resolution = 0.1;
    double auto_resolution = 0.1;
    int downsample_num = 1;
    int resampled_smoothing_num = 1;
    
    double dt = 0.002;
    double force_threshold = 10.0;
    std::vector<double> force_override_vec = {0.0, 0.0, 0.0};
    std::vector<double> auto_override_force_vec = {0.0, 0.0, 0.0};
    double q_pos = 0.001;
    double q_ori = 0.01;
    double r_pos = 0.005;
    double r_ori = 0.05;
    bool enable_auto_tune = false;
    bool des_par_auto_tune = false;

    std::vector<PoseUKF::Vec7> measurement_buffer;

    if (!nh.getParam("raw_path_filepath", input_path)) {
        ROS_ERROR("Failed to get param 'raw_path_filepath'");
        return 1;
    }

    if (!nh.getParam("resample_path_filepath", resample_path)) {
        ROS_ERROR("Failed to get param 'resample_path_filepath'");
        return 1;
    }

    if (!nh.getParam("regen_path_filepath", output_path)) {
        ROS_ERROR("Failed to get param 'regen_path_filepath'");
        return 1;
    }

    nh.param("path_resolution", resolution, resolution);
    nh.param("auto_path_resolution", auto_resolution, auto_resolution);
    nh.param("downsample_num", downsample_num, downsample_num);
    nh.param("resampled_smoothing_num", resampled_smoothing_num, resampled_smoothing_num);
    nh.param("ukf_dt", dt, dt);
    nh.param("force_threshold", force_threshold, force_threshold);
    nh.param("ukf_q_pos", q_pos, q_pos);
    nh.param("ukf_q_ori", q_ori, q_ori);
    nh.param("ukf_r_pos", r_pos, r_pos);
    nh.param("ukf_r_ori", r_ori, r_ori);
    nh.param("ukf_auto_tune", enable_auto_tune, false);
    nh.param("des_par_auto_tune", des_par_auto_tune, false);
    nh.getParam("override_force", force_override_vec);
    nh.getParam("auto_override_force", auto_override_force_vec);
    
    Eigen::Vector3d override_force;
    if(des_par_auto_tune) {override_force = {force_override_vec[0], force_override_vec[1], force_override_vec[2]};}
    else {override_force = {auto_override_force_vec[0], auto_override_force_vec[1], auto_override_force_vec[2]};}

    /* UKF 초기 생성 */
    PoseUKF ukf(q_pos, q_ori, r_pos, r_ori);

    /* 필터 적용 (auto-tune이면 measurement 저장도 같이) */
    ROS_INFO_STREAM("Reading and filtering power data from " << resample_path);
    std::vector<Power> control_powers = readAndFilterPowerData(resample_path, dt, force_threshold, override_force, ukf, enable_auto_tune, measurement_buffer);

    // 3. 자동 튜닝 적용
    if (enable_auto_tune) {
        if (measurement_buffer.size() < 5) {
            ROS_WARN("Not enough measurements for auto-tuning. Skipping tuning.");
        } else {
            ROS_INFO("Running UKF auto-tuning with %lu samples...", measurement_buffer.size());
            ukf.autoTune(measurement_buffer);
    
            std::vector<PoseUKF::Vec7> dummy_buffer;
            control_powers = readAndFilterPowerData(
                resample_path, dt, force_threshold, override_force, ukf,
                false, dummy_buffer
            );
            ROS_INFO_STREAM("Re-filtered power data after auto-tuning. Size: " << control_powers.size());
        }
    }

    if (control_powers.size() < 4) {
        ROS_ERROR("Need at least 4 control points for Catmull-Rom interpolation");
        return 1;
    }

    ROS_INFO_STREAM("Generating Catmull-Rom path with resolution " 
        << ((double)!des_par_auto_tune)*resolution + ((double)des_par_auto_tune)*auto_resolution);
    ROS_INFO("Start generating path. Sample 0:");
    ROS_INFO_STREAM("Pos: " << control_powers[0].position.transpose()
                    << " | Ori (quat): " << control_powers[0].orientation.coeffs().transpose()
                    << " | Force: " << control_powers[0].force.transpose());

    /* AIDIN continuous path generation */
    auto path = catmull_power_path::generateUniformCatmullPowerPath(control_powers, downsample_num, 
        ((double)!des_par_auto_tune)*resolution + ((double)des_par_auto_tune)*auto_resolution);

    savePowerPathToFile(output_path, path);

    return 0;
}
