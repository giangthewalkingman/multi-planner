#include "offboard/offboard.h"

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh),
                                                                                                 nh_private_(nh_private) {
    state_sub_ = nh_.subscribe("/mavros/state", 10, &OffboardControl::stateCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &OffboardControl::odomCallback, this);
    gps_position_sub_ = nh_.subscribe("/mavros/global_position/global", 1, &OffboardControl::gpsCallback, this);
    opt_point_sub_ = nh_.subscribe("optimization_point", 10, &OffboardControl::optPointCallback, this);
    check_last_opt_sub_ = nh_.subscribe("check_last_opt_point",10, &OffboardControl::checkLastOptPointCallback, this);
    pos_cmd_ = nh_.advertise<controller_msgs::PositionCommand>("/controller/pos_cmd", 1);
    local_sp_vector_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("local_sp_vector3d_array", 10);
    route_pub_ = nh_.advertise<offboard::getRouteMsg>("route_msg_ewok", 10);
    setModeClient = nh_.serviceClient<geometric_controller::setmode>("/controller/set_mode");
    route_server = nh_.advertiseService("/ewok/get_route", &OffboardControl::handleServiceRequest, this);
    nh_private_.getParam("/offboard_node/target_error", target_error_);
    nh_private_.getParam("/offboard_node/z_takeoff", z_takeoff_);
    nh_private_.getParam("/offboard_node/yaw_rate", yaw_rate_);
    nh_private_.getParam("/offboard_node/YAML_path", yaml_path_);
    waitForPredicate(10.0);
    plannerAndLandingFlight(yaml_path_);
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
  current_odom_ = odomMsg;
  yaw_ = tf::getYaw(current_odom_.pose.pose.orientation);
  mav_pos_ = toEigen(odomMsg.pose.pose.position);
  odom_received_ = true;
}

void OffboardControl::gpsCallback(const sensor_msgs::NavSatFix &msg){
 if(!gps_received_ && odom_received_){
 local_start_ = mav_pos_;
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

void OffboardControl::plannerAndLandingFlight(std::string argv) {
    ros::Rate rate(50.0);
    std::printf("\n[ INFO] Mission with planner and marker Mode\n");    
    std::string llh_Path = argv;
    std::cout << "Path is: " << llh_Path << std::endl;
    std::cout << "Load the number of local points: " << std::endl;
    std::ifstream llh_file(llh_Path);
    if (!llh_file.is_open())
  {
      while(ros::ok()) {
        std::cerr << "Failed to open file: " << llh_Path << std::endl;
        rate.sleep();
      }
  }
  else{
    YAML::Node yamlNode = YAML::Load(llh_file);
  int argc = yamlNode["N"].as<int>();
  std::cout << argc << " setpoints! " << std::endl;
  try
    {   
        local_setpoint_.push_back(local_start_);
        for (int i = 0; i < argc; i++) {
            Eigen::Vector3d target;
            target(0) = yamlNode[i][0].as<double>();
            target(1) = yamlNode[i][1].as<double>();
            target(2) = yamlNode[i][2].as<double>();
            local_setpoint_.push_back(target);
            std::cout << "Local x: " << target(0) << ", Local y: " << target(1) << ", Local z: " << target(2) << std::endl;
        }
        std::cout << "Done parsing YAML file!" << std::endl;
    }
  catch (const YAML::Exception& e)
    {
        std::cout << "Error while parsing YAML file: " << e.what() << "\n";
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
    if (opt_point_received_) {
        int i = 0; 
        std::printf("\n[ INFO] Fly with optimization points\n");  
        double target_alpha, this_loop_alpha;
        int size = local_setpoint_.size();
        ros::Time t_check;
        t_check = ros::Time::now();
        Eigen::Vector3d x = local_setpoint_[size-1];
        target_reached_ = false;
        while(ros::ok()) {
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
                if((abs(opt_point_.x - current_odom_.pose.pose.position.x) < 0.3) && (abs(opt_point_.y - current_odom_.pose.pose.position.y) < 0.3)) {
                    cmd_.yaw = yaw_;
                }
                else {
                    cmd_.yaw = this_loop_alpha;
                }
                // cmd_.yaw_dot = mav_yawvel_;
                cmd_.header.stamp = ros::Time::now();
                pos_cmd_.publish(cmd_);
                // target_reached_ = distanceBetween(mav_pos_, local_setpoint_[i], local_setpoint_[i+1]);
                target_reached_ = bisectorRay(mav_pos_, local_setpoint_[i], local_setpoint_[i+1], local_setpoint_[i+2]);
                if(target_reached_) {
                    i++;
                    std::cout << "i = " << i << endl;
                }
                get_route_.prev_point = i;
                get_route_.next_point = i+1;
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


bool OffboardControl::checkPlanError(double error, Eigen::Vector3d x) {
    Eigen::Vector3d geo_error;
    geo_error << x(0) - current_odom_.pose.pose.position.x, x(1) - current_odom_.pose.pose.position.y, x(2) - current_odom_.pose.pose.position.z;
    return (geo_error.norm() < error) ? true : false;
}

void OffboardControl::getRoute(offboard::getRouteMsg &msg) {
    // for(int i  = 0; i < local_setpoint_.size(); i++) {
      // Eigen::Vector3d target;
      int i = 0;
      target_reached_ = false;
      if(ros::ok()&&!target_reached_) {
        if((local_setpoint_[i](0) <= mav_pos_(0) && mav_pos_(0) <= local_setpoint_[i+1](0))||(local_setpoint_[i](1) <= mav_pos_(1) && mav_pos_(1) <= local_setpoint_[i+1](1))) {
          msg.prev_point = i;
          msg.next_point = i+1;
          if(i==local_setpoint_.size()) {
            msg.next_point = i;
          }
        }
        else {
          target_reached_ == true;
        }
      }
}

bool OffboardControl::handleServiceRequest(offboard::getRoute::Request &req, offboard::getRoute::Response& res) {
    res.remaining_estimate = -1;
    res.prev_point = get_route_.prev_point;
    res.next_point = get_route_.next_point;
    return true;
}

bool OffboardControl::distanceBetween(Eigen::Vector3d cur, Eigen::Vector3d pre, Eigen::Vector3d nxt) {
    Eigen::Vector3d u, v;
    u = nxt - pre;
    v = nxt - cur;
    double pro = u.dot(v);
    std::cout << "dot product is: " << pro << std::endl;
    if(pro<= target_error_) {
        return true;
    }
    else {
        return false;
    } 
}

bool OffboardControl::bisectorRay(Eigen::Vector3d cur, Eigen::Vector3d pre, Eigen::Vector3d nxt1, Eigen::Vector3d nxt2) {
    Eigen::Vector3d _nxt1_pre,_nxt1_nxt2,_bisec, _nxt1_cur;
    _nxt1_pre = pre - nxt1;
    _nxt1_nxt2 = nxt2 - nxt1;
    if(_nxt1_pre.dot(_nxt1_nxt2) > 0) {
        _bisec = _nxt1_pre/(_nxt1_pre.norm()) + _nxt1_nxt2/(_nxt1_nxt2.norm());
    }
    else {
        _bisec = _nxt1_pre/(_nxt1_pre.norm()) - _nxt1_nxt2/(_nxt1_nxt2.norm());   
    }
    _nxt1_cur = cur - nxt1;
    double dist_cur2nxt1 = _nxt1_cur.norm();
    double dist_bisec = _bisec.norm();
    double pro_dist = dist_bisec*dist_cur2nxt1;
    double dot_dist = abs(_nxt1_cur.dot(_bisec));
    double error = abs(pro_dist - dot_dist);
    // std::cout << "pro dist: " << pro_dist << std::endl;
    // std::cout << "dot dist: " << dot_dist << std::endl;
    // std::cout << "Error is: " << error << std::endl;
    if(error <= target_error_) {
        return true;
    }
    else{
        return false;
    }
}