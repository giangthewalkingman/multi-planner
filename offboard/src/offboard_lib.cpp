#include "offboard/offboard.h"

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint) : nh_(nh),
                                                                                                                      nh_private_(nh_private),                                                                                                               
                                                                                                                      return_home_mode_enable_(false) {
    state_sub_ = nh_.subscribe("/mavros/state", 10, &OffboardControl::stateCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &OffboardControl::odomCallback, this);
    gps_position_sub_ = nh_.subscribe("/mavros/global_position/global", 1, &OffboardControl::gpsrawCallback, this);
    opt_point_sub_ = nh_.subscribe("optimization_point", 10, &OffboardControl::optPointCallback, this);
    check_last_opt_sub_ = nh_.subscribe("check_last_opt_point",10, &OffboardControl::checkLastOptPointCallback, this);
    pos_cmd_ = nh_.advertise<controller_msgs::PositionCommand>("/controller/pos_cmd", 1);
    local_sp_vector_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("local_sp_vector3d_array", 10);
    setModeClient = nh_.serviceClient<geometric_controller::setmode>("/controller/set_mode");
    nh_private_.getParam("/offboard_node/target_error", target_error_);
    nh_private_.getParam("/offboard_node/z_takeoff", z_takeoff_);
    nh_private_.getParam("/offboard_node/yaw_rate", yaw_rate_);
    std::string yamlPath = ros::package::getPath("geometric_controller") + "/cfg/gps_calib.yaml";
    std::ifstream file(yamlPath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << yamlPath << std::endl;
        // return 1;
    }
    // Load YAML data from the file
    YAML::Node yamlData = YAML::Load(file);
    // Access the loaded data
    if (yamlData["offsetX"].IsDefined())
    {
        offset_(0) = -yamlData["offsetX"].as<double>();
    }
    if (yamlData["offsetX"].IsDefined())
    {
        offset_(1) = -yamlData["offsetY"].as<double>();
    }
     file.close();
     
    waitForPredicate(10.0);
    plannerAndLandingFlight();
}

OffboardControl::~OffboardControl() {

}

/* wait for connect, GPS received, ...
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForPredicate(double hz) {
    ros::Rate rate(hz);

    std::printf("\n[ INFO] Waiting for FCU connection \n");
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] FCU connected \n");

    std::printf("[ INFO] Waiting for GPS signal \n");
    while (ros::ok() && !gps_received_) {
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] GPS position received \n");
    if (simulation_mode_enable_) {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set true\n");
        std::printf("          OFFBOARD node will automatic ARM and set OFFBOARD mode\n");
        std::printf("          Continue if run a simulation OR SHUTDOWN node if run in drone\n");
        std::printf("          Set parameter 'simulation_mode_enable' to false or not set (default = false)\n");
        std::printf("          and relaunch node for running in drone\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=false\n");
    }
    else {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set false or not set (default = false)\n");
        std::printf("          OFFBOARD node will wait for ARM and set OFFBOARD mode from RC controller\n");
        std::printf("          Continue if run in drone OR shutdown node if run a simulation\n");
        std::printf("          Set parameter 'simulation_mode_enable' to true and relaunch node for simulation\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=true\n");
    }
    operation_time_1_ = ros::Time::now();
}


void OffboardControl::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
}

void OffboardControl::odomCallback(const nav_msgs::Odometry &odomMsg){
  mav_att_.w()=odomMsg.pose.pose.orientation.w;
  mav_att_.x()=odomMsg.pose.pose.orientation.x;
  mav_att_.y()=odomMsg.pose.pose.orientation.y;
  mav_att_.z()=odomMsg.pose.pose.orientation.z;
//   yaw_ = ToEulerYaw(mav_att_); 
  yaw_ = tf::getYaw(current_odom_.pose.pose.orientation);
//   yaw_rate_ = (mav_att_ * toEigen(odomMsg.twist.twist.angular))(2);
  mav_pos_ = toEigen(odomMsg.pose.pose.position);
  current_odom_ = odomMsg;
  mav_vel_ = mav_att_ * toEigen(odomMsg.twist.twist.linear);
//   cmd_.yaw_dot = mav_yawvel_;
//   odom_init = true;
  odom_received_ = true;
}

void OffboardControl::gpsrawCallback(const sensor_msgs::NavSatFix &msg){
 if(!gps_received_ && odom_received_){
 local_start_ = mav_pos_;
 local_start_ += offset_;
 gpsraw_(0) = msg.latitude;
 gpsraw_(1) = msg.longitude;
 gpsraw_(2) = msg.altitude ;
 gps_received_ = true; 
 }
}

void OffboardControl::optPointCallback(const geometry_msgs::Point::ConstPtr &msg) {
    opt_point_ = *msg;
    opt_point_received_ = true;
}

void OffboardControl::checkLastOptPointCallback(const std_msgs::Bool::ConstPtr &msg) {
    check_last_opt_point_.data = false;
    check_last_opt_point_ = *msg;
}

void OffboardControl::plannerAndLandingFlight() {
    ros::Rate rate(50.0);
    std::printf("\n[ INFO] Mission with planner and marker Mode\n");
    std::cout << "Load the number of gps points: " << std::endl;
    std::string llh_Path = ros::package::getPath("geometric_controller") + "/cfg/not.yaml";
    std::ifstream llh_file(llh_Path);
    if (!llh_file.is_open())
  {
      while(ros::ok()) {
        std::cerr << "Failed to open file: " << llh_Path << std::endl;
        rate.sleep();
      }
    //   return 1;
  }
  else{
    YAML::Node yamlNode = YAML::Load(llh_file);
  int argc = yamlNode["N"].as<int>();
  std::cout << argc << " setpoints! " << std::endl;
  try
    {
        for (int i = 1; i < argc; i++) {
            Eigen::Vector3d target;
            target(0) = yamlNode[i][0].as<double>();
            target(1) = yamlNode[i][1].as<double>();
            gps_target_.push_back(target);
            std::cout << "Latitude: " << std::setprecision(15) << target(0) << ", Longitude: " << std::setprecision(15)<< target(1) << std::endl;
        }
        std::cout << "Done parsing YAML file!" << std::endl;
        // std::cout << "Latitude: " << target.x() << ", Longitude: " << target.y() << std::endl;
    }
  catch (const YAML::Exception& e)
    {
        std::cout << "Error while parsing YAML file: " << e.what() << "\n";
    }
    local_setpoint_.push_back(local_start_); //push back the current position to loal_setpoint_
    LatLonToUTMXY(gpsraw_(0),gpsraw_(1),48,UTM_X_,UTM_Y_);//32 zurich 48 VietNam
    for(auto target : gps_target_){
        LatLonToUTMXY(target(0),target(1),48,UTM_SP_X_,UTM_SP_Y_);
        Eigen::Vector3d setpoint;
        setpoint(0) = mav_pos_(0) - UTM_X_ + UTM_SP_X_;
        setpoint(1) = mav_pos_(1) - UTM_Y_ + UTM_SP_Y_;
        setpoint(2) = mav_pos_(2);
        setpoint += offset_;
        local_setpoint_.push_back(setpoint);
        std::cout << "mav_pos 0 1 2: " << mav_pos_(0) << " " << mav_pos_(1) << " " << mav_pos_(2) << std::endl;
        std::cout << "UTM x y: " << UTM_X_ << " " << UTM_Y_ << std::endl;
        std::cout << "UTM sp x y: " << UTM_SP_X_ << " " << UTM_SP_Y_ << std::endl;
        std::cout << "offset: " << offset_(0) << " " << offset_(1) << std::endl;


    }
    std::printf("[ INFO] Loaded global path setpoints: \n");
    for(auto lsp: local_setpoint_){
        ROS_INFO_STREAM("local_setpoint " << lsp(0) << " " << lsp(1) << " " << lsp(2));
    }
  } 

    std::printf("\n[ INFO] Flight with Planner setpoint\n");
    for(auto target : local_setpoint_) {
        target_array_.data.push_back(target(0));
        target_array_.data.push_back(target(1));
        target_array_.data.push_back(target(2));
        std::cout << target_array_.data[ target_array_.data.size()-3] << " " << target_array_.data[ target_array_.data.size()-2] << " " << target_array_.data[ target_array_.data.size()-1] << std::endl;
    }
    while(ros::ok()) {
        ROS_INFO_STREAM("Waiting for optimization points");
        if(opt_point_received_) {
            break;
        }
        ros::Rate r(4);
        local_sp_vector_pub_.publish(target_array_);
        ros::spinOnce();
        r.sleep();
    }
    // yaw_ = mav_yaw_;
    // yaw_rate_ = 0.3;
    bool target_reached = false; 
    if (opt_point_received_) {
        std::printf("\n[ INFO] Fly with optimization points\n");  
        // bool first_receive = true;
        double target_alpha, this_loop_alpha;
        // bool flag = true;
        int size = local_setpoint_.size();
        // int l = 0;
        ros::Time t_check;
        t_check = ros::Time::now();
        Eigen::Vector3d x = local_setpoint_[size-1];
        target_reached = checkPlanError(target_error_, x);
        while(ros::ok() && !target_reached) {
                rate.sleep();
                target_alpha = calculateYawOffset(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), targetTransfer(opt_point_.x, opt_point_.y, opt_point_.z));
                if ((yaw_ - target_alpha) >= PI) {
                    target_alpha += 2*PI;
                }
                else if ((yaw_ - target_alpha) <= -PI) {
                    target_alpha -= 2*PI;
                }
                else{}

                // calculate the input for position controller (this_loop_alpha) so that the input yaw value will always be higher or lower than current yaw angle (yaw_) a value of yaw_rate_
                // this make the drone yaw slower
                if (target_alpha <= yaw_) {
                    if ((yaw_ - target_alpha) > yaw_rate_) {
                        this_loop_alpha = yaw_ - yaw_rate_;
                    }
                    else {
                        this_loop_alpha = target_alpha;
                    }
                }
                else {
                    if ((target_alpha - yaw_) > yaw_rate_) {
                        this_loop_alpha = yaw_ + yaw_rate_;
                    }
                    else {
                        this_loop_alpha = target_alpha;
                    }
                }

                cmd_.position.x = opt_point_.x; 
                cmd_.position.y = opt_point_.y; 
                cmd_.position.z = opt_point_.z;
                cmd_.yaw = this_loop_alpha;
                // cmd_.yaw_dot

                if((abs(opt_point_.x - current_odom_.pose.pose.position.x) < 0.3) && (abs(opt_point_.y - current_odom_.pose.pose.position.y) < 0.3)) {
                    // target_enu_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);
                    cmd_.yaw = yaw_;
                }
                else {
                    // target_enu_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(this_loop_alpha);
                    cmd_.yaw = this_loop_alpha;
                }
                // cmd_.yaw_dot = mav_yawvel_;
                cmd_.header.stamp = ros::Time::now();
                pos_cmd_.publish(cmd_);
                target_reached = checkPlanError(target_error_, x);
                ros::spinOnce();
            }    
    }
    else {
        // std::printf("\n[ WARN] Not received optimization points!\n");
        while(ros::ok() && !setModeCall.response.success){
            std::printf("\n[ WARN] Not received optimization points!\n");
                        setModeCall.request.mode = setModeCall.request.HOLD;
                        setModeCall.request.sub = z_takeoff_; 
                        setModeCall.request.timeout = 50;
                        setModeClient.call(setModeCall);
                        ros::spinOnce();
                        rate.sleep();
                    }
    }
}

//transfer a x,y,z to a poseStamped datatype
geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z, geometry_msgs::Quaternion yaw) {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation = yaw;
    return target;
}

/* transfer x, y, z (meter) and yaw (degree) setpoint to same message type with enu setpoint msg
   input: x, y, z in meter and yaw in degree that want to create geometry_msgs::PoseStamped msg */
geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z, double yaw) {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation = tf::createQuaternionMsgFromYaw(radianOf(yaw));
    return target;
}

/* transfer x, y, z setpoint to same message type with enu setpoint msg
   input: x, y, z that want to create geometry_msgs::PoseStamped msg */
geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z) {
    //std::cout<<x<<" "<<y<<" "<<z<<std::endl;
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    //target.pose.orientation = 0;
    return target;
}


/* calculate yaw offset between current position and next optimization position */
double OffboardControl::calculateYawOffset(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped setpoint) {
    double alpha;
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double xs = setpoint.pose.position.x;
    double ys = setpoint.pose.position.y;

    alpha = atan2(abs(ys - yc), abs(xs - xc));
    if ((xs > xc) && (ys > yc)) {
        return alpha;
    }
    else if ((xs < xc) && (ys > yc)) {
        return (PI - alpha);
    }
    else if ((xs < xc) && (ys < yc)) {
        return (alpha - PI);
    }
    else if ((xs > xc) && (ys < yc)) {
        return (-alpha);
    }
    else if ((xs == xc) && (ys > yc)) {
        return alpha;
    }
    else if ((xs == xc) && (ys < yc)) {
        return (-alpha);
    }
    else if ((xs > xc) && (ys == yc)) {
        return alpha;
    }
    else if ((xs < xc) && (ys == yc)) {
        return (PI - alpha);
    }
    else {
        return alpha;
    }
}

double OffboardControl::ToEulerYaw(const Eigen::Quaterniond& q){
    Vector3f angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }

bool OffboardControl::checkPlanError(double error, Eigen::Vector3d x) {
    Eigen::Vector3d geo_error;
    geo_error << x(0) - current_odom_.pose.pose.position.x, x(1) - current_odom_.pose.pose.position.y, x(2) - current_odom_.pose.pose.position.z;
    return (geo_error.norm() < error) ? true : false;
}
