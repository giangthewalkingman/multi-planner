#include <ros/ros.h>
#include <offboard/getRoute.h>

bool handleServiceRequest(offboard::getRoute::Request& req,
                          offboard::getRoute::Response& res) {
    res.sum = req.num1 + req.num2;
    ROS_INFO("Received service request: %ld + %ld = %ld",
             req.num1, req.num2, res.sum);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_service_server");

    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("my_service", handleServiceRequest);

    ROS_INFO("Ready to receive service requests.");
    ros::spin();

    return 0;
}