#include <ros/ros.h>
#include <offboard/getRoute.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense> 
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometric_controller/common.h>
#include <std_msgs/Float32MultiArray.h>
#include <offboard/getRouteMsg.h>

double yaw_;
Eigen::Vector3d mav_pos_;
nav_msgs::Odometry current_odom_;
bool odom_received_ = false;
std::vector<Eigen::Vector3d> local3dsp_vector_;
bool route_received_ = false;
bool target_reached = false;
int prev_, next_;

void odomCallback(const nav_msgs::Odometry &odomMsg){
  current_odom_ = odomMsg;
  yaw_ = tf::getYaw(current_odom_.pose.pose.orientation);
  mav_pos_ = toEigen(odomMsg.pose.pose.position);
  odom_received_ = true;
}

void routeMsgCallback(const offboard::getRouteMsg &msg) {
  prev_ = msg.prev_point;
  next_ = msg.next_point;
  route_received_ = true;

}

bool handleServiceRequest(offboard::getRoute::Request &req, offboard::getRoute::Response& res) {
    res.remaining_estimate = -1;
    while(ros::ok() && !route_received_) {
        ros::Rate rate(2);
        ROS_INFO_STREAM("Waiting for route's publisher");
        ros::spinOnce();
        rate.sleep();
    }
    res.prev_point = prev_;
    res.next_point = next_;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_service_server");

    ros::NodeHandle nh;
    ros::Subscriber route_sub = nh.subscribe("route_msg_ewok", 10, &routeMsgCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10, &odomCallback, ros::TransportHints().tcpNoDelay());
    ros::ServiceServer server = nh.advertiseService("/ewok/get_route", handleServiceRequest);
    // ros::ServiceServer srv = nh.advertiseService("hello", &handleServiceRequest, ros::TransportHints().tcpNoDelay());
    ROS_INFO("Ready to receive service requests of Ewok.");
    ros::spin();

    return 0;
} 