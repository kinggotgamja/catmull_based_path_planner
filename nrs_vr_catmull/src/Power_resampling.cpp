#include <ros/ros.h>
#include "catmull_uniform_path_planner/catmull_path.h"
#include "pose_ukf_filter/PoseUKF.hpp"
#include "nrs_blender_pkg/Profiler.h"
#include <fstream>
#include <sstream>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>  // yaml-cpp 헤더 추가


class Power_resampling
{
    public :
        Power_resampling(){};
        ~Power_resampling(){};
        static std::vector<Power> Power_data_resampling(const std::string& input_filename, const std::string& resampl_filename, const int times, const int resampled_smoothing_num, const double dt, const double acc_time);
        static std::vector<Power> Power_motion_blender(const std::vector<Power>& Loaded_data, double Sampling_time, double Starting_time=3, double Last_resting_time=3, double Acceleration_time=0.5, int Colum_num=10);
        static void MeanForceVelocity(const std::vector<Power>& path, const double force_threshold, const double dt, double& mean_force, double& mean_velocity);
        static void saveToYaml(const std::string& yaml_file, const std::vector<double>& force_override_vec, const double mean_velocity);
        static Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);
        static void quaternionToRPY(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);

    private:
};

Eigen::Quaterniond Power_resampling::rpyToQuaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    return yawAngle * pitchAngle * rollAngle;
}

void Power_resampling::quaternionToRPY(const Eigen::Quaterniond& q, 
    double& roll, double& pitch, double& yaw) {
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
}

std::vector<Power> Power_resampling::Power_data_resampling(
    const std::string& input_filename, const std::string& resampl_filename, const int times, const int resampled_smoothing_num, const double dt, const double acc_time)
{
    std::vector<Power> raw_data, resampled_data;
    std::ifstream infile(input_filename);
    std::string line;

    /* Read raw data */
    if (!infile.is_open()) {
        ROS_ERROR_STREAM("Failed to open input file: " << input_filename);
        return resampled_data;
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
        Eigen::Vector3d force(fx, fy, fz);

        raw_data.push_back({pos, q.normalized(), force});
    }

    /* Data resampling */
    resampled_data = catmull_power_path::generateFineCatmullPowerPath(raw_data, (double)(1/(double)times));
    resampled_data = catmull_power_path::generateSmoothCatmullPowerPath(resampled_data, resampled_smoothing_num);
    
    /* Motion blending */
    if(acc_time > 0) {
        resampled_data = Power_motion_blender(resampled_data, dt, 3, 3, acc_time); // 0.002초로 resampling (0.01초 -> 0.002초)
    }

    /* Save the resampled data */
    std::ofstream resampled_file(resampl_filename);

    if (!resampled_file.is_open()) {
        ROS_ERROR_STREAM("Failed to open output file: " << resampl_filename);
        return resampled_data;
    }

    double re_roll, re_pitch, re_yaw;
    for (const auto& power : resampled_data) {
        quaternionToRPY(power.orientation, re_roll, re_pitch, re_yaw);

        /* Special handling for NRS UR10 */
        while(re_roll<0){re_roll += 2*M_PI;}

        resampled_file << power.position.x() << " "
                        << power.position.y() << " "
                        << power.position.z() << " "
                        << re_roll << " " << re_pitch << " " << re_yaw << " "
                        << power.force.x() << " "
                        << power.force.y() << " "
                        << power.force.z() << std::endl;
    }

    ROS_INFO_STREAM("Saved resampled path to " << resampl_filename);
    ROS_INFO_STREAM("Resampled data size: " << resampled_data.size());
    return resampled_data;
}

std::vector<Power> Power_resampling::Power_motion_blender(const std::vector<Power>& Loaded_data, 
 double Sampling_time, double Starting_time, double Last_resting_time, double Acceleration_time, int Colum_num)
{
    /* Parameter Initialization */
    double time_counter = 0; // Time counter (Do not modify)
    vector<vector<double>> blendedPath; 
    std::vector<Power> blended_data;

    /* Motion blender */
    for (size_t i = 0; i < Colum_num; i++) { // iterate through each column
        vector<double> time, data;
        time_counter = 0;
        for (const auto& row : Loaded_data) {
            time.push_back(time_counter*Sampling_time);
            if(i<3) {data.push_back(row.position(i));}
            if (i < 7) {
                if (i == 3) data.push_back(row.orientation.w());
                else if (i == 4) data.push_back(row.orientation.x());
                else if (i == 5) data.push_back(row.orientation.y());
                else if (i == 6) data.push_back(row.orientation.z());
            }
            else {data.push_back(row.force(i-7));}
            time_counter ++;
        }
        
        NRSProfiler profiler(time, data, Starting_time, Last_resting_time, Acceleration_time, Sampling_time);
        vector<vector<double>> result = profiler.AccDecProfiling();

        if(i == 0) {blendedPath.resize(result.size(), vector<double>(Colum_num, 0.0));}

        uint64_t BP_counter = 0;
        for(const auto& row : result)
        {
            blendedPath[BP_counter][i] = row[1];
            BP_counter++;
        }
    }

     /* Transmit to output form */
     for(const auto& data:blendedPath)
    {
        Eigen::Vector3d pos(data[0], data[1], data[2]);
        Eigen::Quaterniond ori(data[3], data[4], data[5], data[6]);
        Eigen::Vector3d force(data[7], data[8], data[9]);

        blended_data.push_back({pos, ori.normalized(), force});
        // blended_data.push_back({pos, ori, force});
    }

    return blended_data;

}

void Power_resampling::MeanForceVelocity(const std::vector<Power>& path, const double force_threshold, const double dt, double& mean_force, double& mean_velocity) {
    double total_force = 0.0;
    double total_velocity = 0.0;
    std::vector<double> pre_position = {0,0,0};
    
    int count = 0;

    for (const auto& power : path) {
        if (power.force.norm() > force_threshold) {
            total_force += power.force.norm();
            total_velocity += sqrt(pow((power.position(0) - pre_position[0])/dt,2) +
                                pow((power.position(1) - pre_position[1])/dt,2) +
                                pow((power.position(2) - pre_position[2])/dt,2));
            
            pre_position[0] = power.position(0);
            pre_position[1] = power.position(1);
            pre_position[2] = power.position(2);
            count++;
        }

    }

    if (count > 0) {
        mean_force = total_force / count;
        mean_velocity = total_velocity / count;
    } else {
        mean_force = 0.0;
        mean_velocity = 0.0;
    }
    ROS_INFO_STREAM("Mean Force: " << mean_force << ", Mean Velocity: " << mean_velocity);
}

void Power_resampling::saveToYaml(const std::string& yaml_file, const std::vector<double>& force_override_vec, const double mean_velocity) {
    // YAML 파일을 읽어옵니다.
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_file);  // 기존 YAML 파일 읽기
    } catch (const std::exception& e) {
        ROS_ERROR("Error reading YAML file: %s", e.what());
        return;
    }

    // 'override_force' 키를 수정
    if (config["auto_override_force"]) {
        config["auto_override_force"] = force_override_vec;  // 기존 값을 수정
    } else {
        config["auto_override_force"] = force_override_vec;  // 키가 없으면 새로 추가
    }

    // 'mean_velocity' 키를 수정
    if (config["auto_path_resolution"]) {
        config["auto_path_resolution"] = mean_velocity;  // 기존 값을 수정
    } else {
        config["auto_path_resolution"] = mean_velocity;  // 키가 없으면 새로 추가
    }

    // 수정된 YAML 파일을 저장합니다.
    std::ofstream fout(yaml_file);
    if (fout.is_open()) {
        fout << config;
        fout.close();
        ROS_INFO_STREAM("Saved updated override_force to " << yaml_file);
    } else {
        ROS_ERROR_STREAM("Failed to open YAML file for writing.");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "power_resampling_node");
    ros::NodeHandle nh;

    std::string input_path, resample_path;
    int resampled_smoothing_num = 1;
    
    double dt = 0.002;
    double acc_time = 0.5; // Acceleration time for motion blending
    double force_threshold = 10.0;
    std::vector<double> force_override_vec = {0.0, 0.0, 0.0};

    bool enable_auto_tune = false;
    std::string config_file;

    std::vector<PoseUKF::Vec7> measurement_buffer;

    if (!nh.getParam("raw_path_filepath", input_path)) {
        ROS_ERROR("Failed to get param 'raw_path_filepath'");
        return 1;
    }

    if (!nh.getParam("resample_path_filepath", resample_path)) {
        ROS_ERROR("Failed to get param 'resample_path_filepath'");
        return 1;
    }

    nh.param("resampled_smoothing_num", resampled_smoothing_num, resampled_smoothing_num);
    nh.param("ukf_dt", dt, dt);
    nh.param("acc_time", acc_time, acc_time);
    nh.param("force_threshold", force_threshold, force_threshold);
    nh.getParam("override_force", force_override_vec);
    nh.getParam("config_file", config_file);

    /* Generate resampled data */
    auto resamped_path = Power_resampling::Power_data_resampling(input_path,resample_path, 5, resampled_smoothing_num,dt,acc_time); // 5배로 resampling (0.01초 -> 0.002초)

    /* Get average */
    double MeanForce = 0.0;
    double MeanVelocity = 0.0;
    Power_resampling::MeanForceVelocity(resamped_path, force_threshold, dt, MeanForce, MeanVelocity);

    force_override_vec = {0.0, 0.0, fabs(MeanForce)};  // 절댓값으로 force 값을 설정
    ROS_INFO_STREAM("Updated override_force: " << force_override_vec[0] << ", " << force_override_vec[1] << ", " << force_override_vec[2]);

    Power_resampling::saveToYaml(config_file, force_override_vec, MeanVelocity*dt);

    return 0;
}
