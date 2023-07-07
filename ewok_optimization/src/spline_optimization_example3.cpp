#include <ros/ros.h>
#include <ewok/ed_ring_buffer.h>
#include <thread>
#include <chrono>
#include <map>
#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include<nav_msgs/Odometry.h>


#include <fstream>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>

#include <ewok/uniform_bspline_3d_optimization.h>
#include <ewok/polynomial_3d_optimization.h>
// DuyNguyen
#include <ewok/depth_traits.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_eigen/tf2_eigen.h>

// congtranv
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Bool.h>

const int POW = 6;       

bool initialized = false;

std::ofstream f_time, opt_time;


ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;

ros::Publisher occ_marker_pub, updated_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, upt_marker_pub, current_traj_pub, command_pt_pub, command_pt_viz_pub;

tf::TransformListener * listener;

geometry_msgs::Point last_ctrl_point;
int target_num;
double target_error_;
bool odom_error_;
std::vector<double> x_target;
std::vector<double> y_target; 
std::vector<double> z_target;
geometry_msgs::PoseStamped current_pose;
nav_msgs::Odometry odom_error;
bool start_reached = false;
bool odom_error_reached = false;
std_msgs::Float32MultiArray target_array;
std_msgs::Bool check_last_opt_point;
std::vector<Eigen::Vector3d> local3dsp_vector_;
bool local3dsp_received_ = false;

// DuyNguyen
/***********************************************************
namespace enc = sensor_msgs::image_encodings;
ros::Subscriber depth_info_sub ,depth_image_sub ;
tf2_ros::Buffer buffer;
int depth_height, depth_width;
float depth_cx,depth_cy,depth_fx,depth_fy;
/************************************************************/
//giang
int num_points = 0;
double dt = 0;
//giang

std::vector<Eigen::Vector3d> local3dspConvert(geometry_msgs::PoseArray x) {
    std::vector<Eigen::Vector3d> y;
    for(int i = 0; i < x.poses.size(); i++) {
        y[i](0) = x.poses[i].position.x;
        y[i](1) = x.poses[i].position.y;
        y[i](2) = x.poses[i].position.z;
    }
    return y;
}

void localSetpointVecArrCallback(const std_msgs::Float32MultiArray msg) {
    int count = msg.data.size();
    ROS_INFO_STREAM(count);
    // int count = msg.data.size();
    count = count/3;
    local3dsp_vector_.reserve(count); // Optional, to improve performance by reserving memory beforehand
    for (int i = 0; i < count; i++) {
        local3dsp_vector_.emplace_back( Eigen::Vector3d(msg.data[i * 3], msg.data[i * 3 + 1], msg.data[i * 3 + 2]) );
        std::cout << "Local 3dsp: "<< local3dsp_vector_[i](0) << " " << local3dsp_vector_[i](1) << " " << local3dsp_vector_[i](2) << std::endl;
    }
    // local3dsp_vector_ = local3dspConvert(posearr);
    local3dsp_received_ = true;
}

void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

// void odomErrorCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//     odom_error_reached = true;
//     odom_error = *msg;
// }

geometry_msgs::PoseStamped targetTransfer(double x, double y, double z) {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    return target;
}
bool checkPosition(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target) {
    double xt = target.pose.position.x;
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;
	double xc = current.pose.position.x;
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;

	if(((xt - error) < xc) && (xc < (xt + error)) 
	&& ((yt - error) < yc) && (yc < (yt + error))
	&& ((zt - error) < zc) && (zc < (zt + error)))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*******************************************************************************/
//Depth Image Processing for image topic encoding = "32FC1"
void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg);
        // std::cout << "cv_ptr->encoding = " << cv_ptr->encoding << std::endl;  //32FC1
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // rostopic echo /camera/depth/duy/camera_info => K: [391.49725341796875, 0.0, 360.0, 0.0, 391.49725341796875, 240.0, 0.0, 0.0, 1.0]
    const float fx = 391.49725341796875; 
    const float fy = 391.49725341796875;
    const float cx = 360.0;
    const float cy = 240.0;

    
    // //transform /map & /baselink
    // static tf::TransformBroadcaster br;
    // tf::Transform br_transform;
    // br_transform.setOrigin(tf::Vector3(current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z));
    // br_transform.setRotation(tf::Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w));
    // br.sendTransform(tf::StampedTransform(br_transform, ros::Time::now(),"/map","/base_link")); 

    tf::StampedTransform transform;
    try {
        listener->lookupTransform("/map", "/camera_link", msg->header.stamp, transform);  //camera_link  camera_depth_optical_frame
    }
    catch (tf::TransformException &ex) {
        // ROS_INFO("Couldn't get transform");
        // ROS_WARN("%s",ex.what());
        return;
    }

    Eigen::Affine3d dT_w_c;
    tf::transformTFToEigen(transform, dT_w_c);

    Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

    float * data = (float *) cv_ptr->image.data;

    auto t1 = std::chrono::high_resolution_clock::now();

    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;

    for(int u=0; u < cv_ptr->image.cols; u+=4) {
        for(int v=0; v < cv_ptr->image.rows; v+=4) {
            float val = data[v*cv_ptr->image.cols + u]; 

            //ROS_INFO_STREAM(val);
            //if((std::isfinite(val)) && (val>0.35))
            if((std::isfinite(val)) && (val>0.4)) {
                Eigen::Vector4f p;
                p[0] = val*(u - cx)/fx;
                p[1] = val*(v - cy)/fy;
                p[2] = val;
                p[3] = 1;
                
                p = T_w_c * p;

                cloud.push_back(p);

                // if(cloud.at(dem).z() < 1.0){
            }
        }
    }

    Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

    auto t2 = std::chrono::high_resolution_clock::now();

    if(!initialized) {
        Eigen::Vector3i idx;

        edrb->getIdx(origin, idx);

        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

        edrb->setOffset(idx);

        initialized = true;
    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        edrb->getIdx(origin, origin_idx);
 
        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;

        while(diff.array().any()) {
            //ROS_INFO("Moving Volume");
            edrb->moveVolume(diff);

            offset = edrb->getVolumeCenter();
            diff = origin_idx - offset;
        }
    }

    auto t3 = std::chrono::high_resolution_clock::now();

    edrb->insertPointCloud(cloud, origin);

    edrb->updateDistance();

    auto t4 = std::chrono::high_resolution_clock::now();

    f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

    visualization_msgs::Marker m_occ, m_free, m_dist;
    //DuyNguyen
    edrb->getMarkerOccupied(m_occ);
    edrb->getMarkerFree(m_free);
    edrb->getMarkerDistance(m_dist, 0.8);    //distance mau xanh duong? default=0.5

    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);
    dist_marker_pub.publish(m_dist); 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "spline_optimization_example");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    listener = new tf::TransformListener;

    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5, true);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5, true);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5, true);


    /*********************************************************/
    /*32FC1*/
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_ ;
    // depth_image_sub_.subscribe(nh, "/camera/depth/image_raw", 5);
    // tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_link", 5);

    // depth_image_sub_.subscribe(nh, "/depth_topic_2", 5);
    // tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_link", 5);  //camera_depth_optical_frame

    depth_image_sub_.subscribe(nh, "/depth_topic_2", 5);
    tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_link", 5);
    
    tf_filter_.registerCallback(depthImageCallback);
    /*********************************************************/

    /*********************************************************/
    /*16UC1
    // depth_image_sub = nh.subscribe("camera/depth/image_raw", 5 , depth_handle_callback);
    depth_image_sub = nh.subscribe("/camera/depth/duy/image_raw", 5 , depth_handle_callback);
    depth_info_sub = nh.subscribe("camera/depth/camera_info", 1 , depthInfoCallback);
    /*********************************************************/




    double resolution;
    pnh.param("resolution", resolution, 0.15);
    edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));

    double distance_threshold_;
    pnh.param("distance_threshold", distance_threshold_, 0.8); //0.5
    
    ROS_INFO("Started spline_optimization_example");

    ros::Publisher global_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);
    ros::Publisher before_opt_pub = nh.advertise<visualization_msgs::MarkerArray>("before_optimization", 1, true);
    ros::Publisher after_opt_pub = nh.advertise<visualization_msgs::MarkerArray>("after_optimization", 1, true);

    // congtranv
    // ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 50, currentPoseCallback);
    ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("optimization_point", 1);
    ros::Publisher point_target_pub = nh.advertise<std_msgs::Float32MultiArray>("point_target",1);
    ros::Publisher check_last_opt_point_pub = nh.advertise<std_msgs::Bool>("check_last_opt_point",1);
    ros::Subscriber local_array_sub = nh.subscribe<std_msgs::Float32MultiArray>("local_sp_vector3d_array",1, localSetpointVecArrCallback);
    //DuyNguyen
    // ros::Subscriber odom_error_sub = nh.subscribe<nav_msgs::Odometry>("odom_error", 1, odomErrorCallback);
    nh.getParam("/spline_optimization_example/target_error", target_error_);
    nh.getParam("/spline_optimization_example/number_of_points", num_points);
    nh.getParam("/spline_optimization_example/dt_value", dt);
    
    std :: cout << "dt = " << dt << ", num_of_points = " << num_points << std :: endl;

    const Eigen::Vector4d limits(0.5, 3, 0.2, 0); // ivsr velocity, acceleration, jerk, snap   //A row-vector containing the elements {0.7, 4, 0, 0} 

    ewok::Polynomial3DOptimization<10> po(limits*0.8);//0.8 ??? limits
    //
    typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;   //vec la mang cac vector3
    while(ros::ok() && !local3dsp_received_) {
        ros::Rate rate(2);
        ROS_INFO_STREAM("Waiting for local 3d array");
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "Global setpoints to generate the global trajectory:\n" << std::endl;
    for(auto target : local3dsp_vector_) {
        vec.push_back(target);
        std::cout << "Target: ["<< target(0) << ", " << target(1) << ", " << target(2) << "]\n";
    }
    std::cout << std::endl;

    auto traj = po.computeTrajectory(vec);

    visualization_msgs::MarkerArray traj_marker;
    traj->getVisualizationMarkerArray(traj_marker, "trajectory", Eigen::Vector3d(1, 1, 0), 0.5); //100

    global_traj_pub.publish(traj_marker);

    // Set up spline optimization
    // HM: const int num_points = 7;
    //const int num_points = 10;
    // DuyNguyen: const double dt = 0.5;
    //const double dt = 0.4;

    ewok::UniformBSpline3DOptimization<6> spline_opt(traj, dt);

    for (int i = 0; i < num_points; i++) {
        spline_opt.addControlPoint(vec[0]);
    }

    spline_opt.setNumControlPointsOptimized(num_points);
    spline_opt.setDistanceBuffer(edrb);
    spline_opt.setDistanceThreshold(distance_threshold_);
    spline_opt.setLimits(limits);


    double tc = spline_opt.getClosestTrajectoryTime(Eigen::Vector3d(-3, -5, 1), 2.0);
    ROS_INFO_STREAM("Closest time: " << tc);

    ROS_INFO("Finished setting up data");

    double current_time = 0;

    double total_opt_time = 0;
    int num_iterations = 0;

    ros::Rate r(1.0/dt);

    // congtranv
    ewok::EuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);
    while(ros::ok() && !local3dsp_received_) {
        // start_reached = checkPosition(target_error_, current_pose, targetTransfer(vec[0].x(), vec[0].y(), vec[0].z()));
        ROS_INFO_STREAM("Waiting for local array!");
        ros::spinOnce();
    }
    start_reached = false;
    // while (ros::ok() && current_time < traj->duration()) {
    while (ros::ok() && !start_reached) {
        r.sleep();
        current_time += dt;

        visualization_msgs::MarkerArray before_opt_markers, after_opt_markers;

        spline_opt.getMarkers(before_opt_markers, "before_opt",
                            Eigen::Vector3d(1, 0, 0),
                            Eigen::Vector3d(1, 0, 0));

        auto t1 = std::chrono::high_resolution_clock::now();
        double error = spline_opt.optimize();
        auto t2 = std::chrono::high_resolution_clock::now();

        double miliseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() / 1.0e6;

        total_opt_time += miliseconds;
        num_iterations++;


        ROS_INFO_STREAM("Finished optimization in " << miliseconds << " ms. Error: " << error);

        spline_opt.getMarkers(after_opt_markers, "after_opt",
                            Eigen::Vector3d(0, 1, 0),
                            Eigen::Vector3d(0, 1, 1));

        after_opt_pub.publish(after_opt_markers);

        spline_opt.addLastControlPoint();

        std :: cout << "=============================================" << std::endl;
        std :: cout << "First Control Point: \n" << spline_opt.getFirstOptimizationPoint() << std::endl;
        std :: cout << "=============================================" << std::endl;
        //std :: cout << "dt = " << dt << ", num_of_points = " << num_points << std :: endl;

        last_ctrl_point.x = spline_opt.getFirstOptimizationPoint().x();
        last_ctrl_point.y = spline_opt.getFirstOptimizationPoint().y();
        last_ctrl_point.z = spline_opt.getFirstOptimizationPoint().z();
        point_pub.publish(last_ctrl_point);
        // DuyNguyen
        // local3dsp_vector_[local3dsp_vector_.size()-1](0)
        start_reached = checkPosition(target_error_, current_pose, targetTransfer(local3dsp_vector_[local3dsp_vector_.size()-1](0), local3dsp_vector_[local3dsp_vector_.size()-1](1), local3dsp_vector_[local3dsp_vector_.size()-1](2)));
        ros::spinOnce();
    }

    while (ros::ok()) {
        r.sleep();
        check_last_opt_point.data = true;
        check_last_opt_point_pub.publish(check_last_opt_point);
        std :: cout << "check_last_opt_point = " << check_last_opt_point.data << std::endl;
        ros::spinOnce();
    }

    f_time.close();
    opt_time.close();
    return 0;
}
