#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Float64MultiArray.h"
#include <fstream>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>

class DataLoggerNode
{
public:
  DataLoggerNode() : recording(false)
  {
    // 구독자 초기화는 별도 멤버 함수에서 진행
    // 키보드 입력 쓰레드를 시작합니다.
    keyboard_thread = std::thread(&DataLoggerNode::keyboardInput, this);
  }

  ~DataLoggerNode()
  {
    // 노드 종료 시, 열려있는 파일 닫기
    if (pose_file.is_open())
      pose_file.close();
    if (wrench_file.is_open())
      wrench_file.close();

    // 키보드 입력 쓰레드 종료 대기
    if (keyboard_thread.joinable())
      keyboard_thread.join();
  }

  // 구독자 초기화 함수
  void initSubscribers(ros::NodeHandle& nh)
  {
    pose_sub = nh.subscribe("/pose_topic", 1000, &DataLoggerNode::poseCallback, this);
    wrench_sub = nh.subscribe("/wrench_topic", 1000, &DataLoggerNode::wrenchCallback, this);
  }

private:
  // 멤버 변수
  ros::Subscriber pose_sub;
  ros::Subscriber wrench_sub;
  std::ofstream pose_file;
  std::ofstream wrench_file;
  std::atomic<bool> recording;
  std::thread keyboard_thread;

  // 현재 시간 기반 타임스탬프 문자열 생성
  std::string getCurrentTimestamp()
  {
    auto now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%y%m%d%H%M%S");
    return ss.str();
  }

  // Pose 메시지 콜백
  void poseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if (recording && pose_file.is_open())
    {
      pose_file << msg->data[0] << " "
                << msg->data[1] << " "
                << msg->data[2] << " "
                << msg->data[3] << " "
                << msg->data[4] << " "
                << msg->data[5] << "\n";
    }
  }

  // Wrench 메시지 콜백
  void wrenchCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if (recording && wrench_file.is_open())
    {
      wrench_file << msg->data[0] << " "
                  << msg->data[0] << " "
                  << msg->data[0] << " "
                  << msg->data[0] << " "
                  << msg->data[0] << " "
                  << msg->data[0] << "\n";
    }
  }

  // 키보드 입력을 처리하는 함수 (별도 쓰레드에서 실행)
  void keyboardInput()
  {
    char input;
    while (ros::ok())
    {
      printf("Press 's' to start recording, 't' to stop recording, or 'q' to quit: ");
      std::cin >> input;
      if (input == 's')
      {
        if (!recording)
        {
          std::string timestamp = getCurrentTimestamp();
          std::string package_path = ros::package::getPath("nrs_data_logger");

          printf("Package path: %s\n", package_path.c_str());

          pose_file.open(package_path + "/data/Posedata_" + timestamp + ".txt", std::ios::out);
          wrench_file.open(package_path + "/data/FTdata_" + timestamp + ".txt", std::ios::out);

          if (pose_file && wrench_file)
          {
            pose_file << "X Y Z Roll Pitch Yaw\n";
            wrench_file << "ForceX ForceY ForceZ TorqueX TorqueY TorqueZ\n";
            recording = true;
            ROS_INFO("Recording started.");
          }
          else
          {
            ROS_ERROR("Failed to open files.");
          }
        }
        else
        {
          ROS_WARN("Already recording.");
        }
      }
      else if (input == 't')
      {
        if (recording)
        {
          recording = false;
          if (pose_file.is_open()) pose_file.close();
          if (wrench_file.is_open()) wrench_file.close();
          ROS_INFO("Recording stopped.");
        }
        else
        {
          ROS_WARN("Not currently recording.");
        }
      }
      else if (input == 'q')
      {
        recording = false;
        if (pose_file.is_open())
          pose_file.close();
        if (wrench_file.is_open())
          wrench_file.close();
        ROS_INFO("Exiting program.");
        ros::shutdown();
        break;
      }
      else
      {
        ROS_WARN("Invalid input. Press 's' to start or 't' to stop recording.");
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_logger_node");
  ros::NodeHandle nh;

  // 클래스 객체를 생성하고 구독자 초기화
  DataLoggerNode logger;
  logger.initSubscribers(nh);

  // ros::spin()이 호출되면 ros::shutdown() 호출 시 종료됩니다.
  ros::spin();

  return 0;
}
