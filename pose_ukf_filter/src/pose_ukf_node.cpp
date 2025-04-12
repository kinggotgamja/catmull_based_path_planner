#include "pose_ukf_filter/PoseUKF.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

double q_pos = 0.001;
double q_ori = 0.01;
double r_pos = 0.005;
double r_ori = 0.05;
PoseUKF ukf(q_pos, q_ori, r_pos, r_ori);

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    PoseUKF::Vec7 z;
    z[0] = msg->pose.position.x;
    z[1] = msg->pose.position.y;
    z[2] = msg->pose.position.z;
    z[3] = msg->pose.orientation.w;
    z[4] = msg->pose.orientation.x;
    z[5] = msg->pose.orientation.y;
    z[6] = msg->pose.orientation.z;

    ukf.predict(0.01); // assume fixed dt
    ukf.update(z);

    PoseUKF::Vec7 filtered = ukf.getState();
    geometry_msgs::PoseStamped out;
    out.header = msg->header;
    out.pose.position.x = filtered[0];
    out.pose.position.y = filtered[1];
    out.pose.position.z = filtered[2];
    out.pose.orientation.w = filtered[3];
    out.pose.orientation.x = filtered[4];
    out.pose.orientation.y = filtered[5];
    out.pose.orientation.z = filtered[6];

    static ros::Publisher pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("filtered_pose", 10);
    pub.publish(out);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_ukf_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("raw_pose", 10, poseCallback);
    ros::spin();
    return 0;
}
