#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include<ros/ros.h>
#include<ros/transport_hints.h>
#include<tf/tf.h>
#include<tf/transform_datatypes.h>
#include <iomanip>

#include<mavros_msgs/State.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/CommandBool.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<geographic_msgs/GeoPoseStamped.h>
#include<sensor_msgs/NavSatFix.h>
#include<geometry_msgs/PoseArray.h>

#include<eigen3/Eigen/Dense> 

#include<iostream>
#include<cmath>
#include<cstdio>
#include<vector>


// #include<offboard/traj_gen.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Int32.h>

#include "geometric_controller/geometric_controller.h"
#include <controller_msgs/PositionCommand.h>
#include "geometric_controller/triangle_form.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

// #include "ros/ros.h"
#include <cstdlib>
// #include "geometric_controller/geometric_controller.h"
// #include "geometric_controller/triangle_form.h"
#include <nav_msgs/Odometry.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
#include <controller_msgs/FlatTarget.h>
// #include <controller_msgs/PositionCommand.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <offboard/getRouteMsg.h>
#include <offboard/getRoute.h>

class OffboardControl
{
  public:
	// OffboardControl();
	OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
	~OffboardControl();
  private:
	/* CONSTANT */
	const double PI = 3.141592653589793238463; // PI
	const double eR = 6378.137;         // earth radius in km
	const double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
	const double b = 6356752.314245;    // Derived Earth semiminor axis (m)
	const double f = (a - b) / a;       // Ellipsoid Flatness
	const double f_inv = 1.0 / f;       // Inverse flattening
	const double a_sq = a * a;
	const double b_sq = b * b;
	const double e_sq = f * (2 - f);    // Square of Eccentricity

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	ros::Subscriber state_sub_; // current state subscriber
	ros::Subscriber gps_position_sub_; // current gps position subscriber
	ros::Subscriber opt_point_sub_; // optimization point from planner subscriber
	ros::Subscriber odom_sub_; // odometry subscriber
	ros::Subscriber point_target_sub_;// target point from planner subscriber
	ros::Subscriber check_last_opt_sub_;// check last optimization point from planner subscriber
	
	//DuyNguyen
	ros::Subscriber marker_p_sub_;
	ros::Subscriber check_move_sub_;
	ros::Subscriber ids_detection_sub_;
	ros::Subscriber local_p_sub_;


	ros::Publisher setpoint_pose_pub_; // publish target pose to drone
	ros::Publisher odom_error_pub_; //publish odom error before arm
	ros::ServiceClient set_mode_client_; // set OFFBOARD mode in simulation
	ros::ServiceClient arming_client_; // call arm command in simulation

	nav_msgs::Odometry current_odom_; // current odometry from mavros: pose (position + orientation) + twist (linear + angular)
	mavros_msgs::State current_state_; // current state from mavros, check connect (onboard-pixhawk), arm, flight mode, ...
	geometry_msgs::PoseStamped home_enu_pose_; // pose to store the starting pose (position + orientation) of drone
	geometry_msgs::PoseStamped target_enu_pose_; // target pose to feed into the drone
	geometry_msgs::Point opt_point_; // point (x,y,z) received from optimization planner
	std_msgs::Bool check_last_opt_point_; // check last optimization point have reached your destination yet.
	
	//DuyNguyen
	geometry_msgs::PoseStamped marker_position_; // call back the marker position 
	bool check_mov_; // check move to the centre of UAV
	bool check_ids_; // check have the ids or not ?
	geometry_msgs::PoseStamped current_position_; // call back the current position
	double current_z_; // current z position
	mavros_msgs::SetMode offboard_setmode_; //check mode flight of uav

	std_msgs::Float32MultiArray target_array_; // start point and end point received from optimization planner
	std::vector<geometry_msgs::Point> optimization_point_; // point (x,y,z) list received from optimization planner, use when want to buffer and check reached each optimization point
	sensor_msgs::NavSatFix current_gps_position_; // current GPS informations from mavros: status (satellite fix status information), Latitude [degrees](Positive is north of equator; negative is south), Longitude [degrees](Positive is east of prime meridian; negative is west), Altitude [m](Positive is above the WGS 84 ellipsoid), ...
	// sensor_msgs::NavSatFix home_gps_position_; // GPS position to store the starting point's GPS
	geographic_msgs::GeoPoseStamped goal_gps_position_; // goal GPS position to feed into the drone
	sensor_msgs::NavSatFix ref_gps_position_; // reference GPS position to convert GPS position to ENU position (LLA to xyz)
	mavros_msgs::SetMode flight_mode_; // use to set custom or default flight mode (e.g., OFFBOARD, LAND, ...)
	
	bool opt_point_received_ = false; // check received optimization point from planner or not
	bool gps_received_ = false; // check received GPS or not
	bool final_position_reached_ = false; // check reached final setpoint or not
	bool odom_received_ = false; // check received odometry or not
	bool delivery_mode_enable_; // check enabled delivery mode or not
	bool simulation_mode_enable_; // check enabled simulation mode or not
	bool return_home_mode_enable_; // check enabled return home mode or not
	bool target_reached_ = false;
	
	int num_of_enu_target_; // number of ENU (x,y,z) setpoints
	std::vector<double> x_target_; // array of ENU x position of all setpoints
	std::vector<double> y_target_; // array of ENU y position of all setpoints 
	std::vector<double> z_target_; // array of ENU z position of all setpoints
	
	std::vector<double> yaw_target_; // array of yaw targets of all setpoints
	double yaw_rate_;
	bool odom_error_;
	double yaw_error_;
	int num_of_gps_goal_; // number of GPS (LLA) setpoints
	std::vector<double> lat_goal_; // array of latitude of all setpoints
	std::vector<double> lon_goal_; // array of longitude of all setpoints
	std::vector<double> alt_goal_; // array of altitude of all setpoints
	
	double target_error_, goal_error_, land_error_; // the offset to check when the drone reached the setpoints (for ENU, GPS and land, corresponding)
	double distance_; // distance from current position to next setpoint
	
	double x_off_[100], y_off_[100], z_off_[100]; // array to calculate offset from current ENU (x,y,z) and GPS converted (x,y,z) in a period
	double x_offset_, y_offset_, z_offset_; // average offset of current ENU (x,y,z) and GPS converted (x,y,z)
	double z_takeoff_; // the height to takeoff when start. drone'll takeoff to z_takeoff_ then start the mission
	double z_delivery_; // the height (set to 0.0 for land to ground - need to set disable auto-disarm of pixhawk) want drone go to for delivery in delivery mode

	double vel_desired_, land_vel_, return_vel_; // corresponding desired speed to fly, when land and when return home
	geometry_msgs::Vector3 components_vel_; // components of desired velocity about x, y, z axis
	double hover_time_, takeoff_hover_time_, unpack_time_; // corresponding hover time when reached setpoint, when takeoff and when unpacking
	ros::Time operation_time_1_, operation_time_2_; // checkpoint to calculate operation time of each perform program

	void waitForPredicate(double hz); // wait for connect, GPS received, ...
	void setOffboardStream(double hz, geometry_msgs::PoseStamped first_target); // send a few setpoints before publish
	void waitForArmAndOffboard(double hz); // wait for ARM and OFFBOARD mode switch (in SITL case or HITL/Practical case)
	void waitForStable(double hz); // wait drone get a stable state
	void stateCallback(const mavros_msgs::State::ConstPtr& msg); // state callback
	// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); // odometry callback
	void gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg); // GPS callback
	void optPointPCCallback(const controller_msgs::PositionCommand::ConstPtr& msg); // optimization point callback
	void optPointCallback(const geometry_msgs::Point::ConstPtr& msg); // optimization point callback
	void targetPointCallback(const std_msgs::Float32MultiArray::ConstPtr &msg); // target point callback
	void checkLastOptPointCallback(const std_msgs::Bool::ConstPtr &msg); //check last optimization point callback

	// DuyNguyen
	void markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); // call back the marker position 
	void checkMoveCallback(const std_msgs::Bool msg); // check move to the centre of UAV
	void checkIdsDetectionCallback(const std_msgs::Bool msg); // check have the ids or not ?
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg); // call back the current position

	inline double degreeOf(double rad) // convert from radian to degree
	{
		return (rad*180)/PI;
	}

	inline double radianOf(double deg) // convert from degree to radian
	{
		return (deg*PI)/180;
	}
	
	template <class T>
    inline T sqr(T x) // calculate square of x
	{
        return x*x;
    }; 

	void inputSetpoint(); // manage input: select mode, setpoint type, ...
	void inputENU(); // manage input for ENU setpoint flight mode: manual input from keyboard, load setpoints
	void enuFlight(); // perform flight with ENU (x,y,z) setpoints
	void inputGPS(); // manage input for GPS setpoint flight mode: manual input from keyboard, load setpoints
	void gpsFlight(); // perform flight with GPS (LLA) setpoints
	void inputENUYaw(); // manage input for ENU setpoint & Yaw angle
	void enuYawFlight(); // perform flight with ENU (x,y,z) setpoints & Yaw angle
	void inputENUYawAndLandingSetpoint(); // manage input for ENU setpoint & Yaw angle & Landing at each setpoint to drop the package
	void enuYawFlightAndLandingSetpoint(); // perform flight with ENU (x,y,z) setpoints & Yaw angle & Landing at each setpoint to drop the package

	void inputPlanner(); // manage for flight with optimization point from planner
	void plannerFlight(); // perform flight with ENU (x,y,z) setpoints from optimization planner
	void inputPlannerAndLanding(); // manage for flight with optimization point from planner
	void plannerAndLandingFlight(std::string argv); // perform flight with ENU (x,y,z) setpoints from optimization planner and Landing at marker

	double calculateYawOffset(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped setpoint); // calculate yaw offset between current position and next optimization position

	void takeOff(geometry_msgs::PoseStamped setpoint, double hover_time); // perform takeoff task
	void hovering(geometry_msgs::PoseStamped setpoint, double hover_time); // perform hover task
	void landing(geometry_msgs::PoseStamped setpoint); // perform land task
	void landingYaw(geometry_msgs::PoseStamped setpoint); // perform land task & Yaw
	
	void returnHome(geometry_msgs::PoseStamped home_pose); // perform return home task
	void returnHomeYaw(geometry_msgs::PoseStamped home_pose); // perform return home task & Yaw
	void delivery(geometry_msgs::PoseStamped setpoint, double unpack_time); // perform delivery task
	void deliveryHover(geometry_msgs::PoseStamped setpoint, double unpack_time); // perform delivery task
	
	sensor_msgs::NavSatFix goalTransfer(double lat, double lon, double alt); // transfer lat, lon, alt setpoint to same message type with gps setpoint msg
	geometry_msgs::PoseStamped targetTransfer(double x, double y, double z); // transfer x, y, z setpoint to same message type with enu setpoint msg
	geometry_msgs::PoseStamped targetTransfer(double x, double y, double z, double yaw); // transfer x, y, z (meter) and yaw (degree) setpoint to same message type with enu setpoint msg
	geometry_msgs::PoseStamped targetTransfer(double x, double y, double z, geometry_msgs::Quaternion yaw);

	bool checkPositionError(double error, geometry_msgs::PoseStamped target); // check offset between current position from odometry and setpoint position to decide when drone reached setpoint
	bool checkPositionError(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target); // check offset between current position and setpoint position to decide when drone reached setpoint
	bool checkOrientationError(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target); // check offset between current orientation and setpoint orientation to decide when drone reached setpoint

	bool checkGPSError(double error, sensor_msgs::NavSatFix current, sensor_msgs::NavSatFix goal); // check offset between current GPS and setpoint GPS to decide when drone reached setpoint

	Eigen::Vector3d getRPY(geometry_msgs::Quaternion quat); // get roll, pitch and yaw angle from quaternion
	// geometry_msgs::Quaternion getQuaternionMsg(double roll, double pitch, double yaw); // create quaternion msg from roll, pitch and yaw
	
	// double distanceBetween(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target); // calculate distance between current position and setpoint position
	geometry_msgs::Vector3 velComponentsCalc(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target); // calculate components of velocity about x, y, z axis

	geometry_msgs::Point WGS84ToECEF(sensor_msgs::NavSatFix wgs84); // convert from WGS84 GPS (LLA) to ECEF x,y,z
	geographic_msgs::GeoPoint ECEFToWGS84(geometry_msgs::Point ecef); // convert from ECEF x,y,z to WGS84 GPS (LLA)  
	geometry_msgs::Point ECEFToENU(geometry_msgs::Point ecef, sensor_msgs::NavSatFix ref); // convert from ECEF x,y,z to ENU x,y,z
	geometry_msgs::Point ENUToECEF(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref); // convert from ENU x,y,z to ECEF x,y,z
	geometry_msgs::Point WGS84ToENU(sensor_msgs::NavSatFix wgs84, sensor_msgs::NavSatFix ref); // convert from WGS84 GPS (LLA) to ENU x,y,z
	geographic_msgs::GeoPoint ENUToWGS84(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref); // convert from ENU x,y,z to WGS84 GPS (LLA)
	ros::Publisher pos_cmd_;
	ros::ServiceClient setModeClient;
	// geometric_controller::setmode setModeCall;
	controller_msgs::PositionCommand cmd_;
	// geometric_controller::setmode setModeCall;
	Eigen::Vector3d mav_pos_, global_point_, mav_vel_;
	TriangleForm plan;
	int sample_idx=0,waypoint_idx=0;
	int sample_size=0;
	std::vector<int> Indexwp;
	bool gps_home_init = false, odom_init = false , setmode_init =false ,plan_init = false ,plan_fin =false;
	// std::vector<Eigen::Vector3d> local_setpoint_;
	bool checkPosCmdError(double error, controller_msgs::PositionCommand target);
	double distancePosCmdBetween(controller_msgs::PositionCommand target);
	std::vector<Eigen::Vector3d> gps_target_,local_setpoint_;
	void gpsCallback(const sensor_msgs::NavSatFix &msg);
	Eigen::Quaterniond mav_att_;
	double mav_yaw_ = 0, mav_yawvel_ = 0;
	Eigen::Vector3d gps_home_,gpsraw_,local_start_,offset_;
	double ToEulerYaw(const Eigen::Quaterniond& q);
	void odomCallback(const nav_msgs::Odometry &odomMsg);
	double UTM_X_,UTM_Y_;
	double UTM_SP_X_,UTM_SP_Y_;
	// ros::Subscriber gpsSub_;
	void triangleFLight(std::vector<Eigen::Vector3d> local_sp, double hz);
	// bool check
	bool checkPlanError(double error, Eigen::Vector3d x);
	geometric_controller::setmode setModeCall;
	// ros::ServiceClient setModeClient;
	ros::Publisher local_sp_vector_pub_;
	ros::Publisher local_sp_num_pub_;
	std_msgs::Int32 num_of_enu_msg_;
	geometry_msgs::Vector3Stamped local3dsp_msg_;
	// geometry_msgs::Point local3dspConvert(Eigen::Vector3d x);
	geometry_msgs::Point local3dspConvert(Eigen::Vector3d x);
	geometry_msgs::PoseArray local3dArrayConvert(std::vector<Eigen::Vector3d> x);
	double yaw_ = 0;
	std::string yaml_path_;
	ros::Publisher route_pub_;
	offboard::getRouteMsg get_route_;
	void getRoute(offboard::getRouteMsg &msg);
	bool handleServiceRequest(offboard::getRoute::Request &req, offboard::getRoute::Response& res);
	ros::ServiceServer route_server;
	bool distanceBetween(Eigen::Vector3d cur, Eigen::Vector3d pre, Eigen::Vector3d nxt);
};


#endif