#include "sexy_dongsu_dm_controller/SexyDongsuMotorController.h"

namespace sexy::dongsu::motor
{
  using namespace damiao;

  bool Controller::init(SexyDongsuJointInterface *robot_hw, ros::NodeHandle &nh)
  {
    std::vector<double> init_pose;
    std::vector<double> init_vel;
    std::vector<double> init_kp;
    std::vector<double> init_kd;
    
    nh.param<std::vector<std::string>>("joint_names", joint_names_, std::vector<std::string>{"yaw_joint", "pitch_joint", "roll_joint"});
    nh.param<std::vector<double>>("init_poses", init_pose, std::vector<double>{1.31, 0.085, 2.0});
    nh.param<std::vector<double>>("init_poses", init_vel, std::vector<double>{2.0, 2.0, 2.0});
    nh.param<std::vector<double>>("init_kp", init_kp, {0.0,0.0,0.0});
    nh.param<std::vector<double>>("init_kd", init_kd, {3.0,3.0,3.0});

    ROS_INFO("[%s] <%d> joint loaded\n", ros::this_node::getName().c_str(), joint_names_.size());
    if (joint_names_.size() != init_pose.size())
      return false;
    for (int idx = 0; idx <joint_names_.size(); idx++)
    {
      ROS_INFO("[%s] name <%s> init\n", ros::this_node::getName().c_str(), joint_names_[idx].c_str());
      hybridJointHandles_.push_back(robot_hw->getHandle(joint_names_[idx]));
      init_cmd_[joint_names_[idx]] = SexyDonguCommand({sexy::dongsu::motor::hardware::Control_Mode::POS_VEL_MODE,init_pose[idx],init_vel[idx],0.0});
      stop_cmd_[joint_names_[idx]] = SexyDonguCommand({sexy::dongsu::motor::hardware::Control_Mode::POS_VEL_MODE,0.0,0.0,0.0});
      pd_cmd_[joint_names_[idx]] = SexyDonguPDCommand({init_kp[idx],init_kd[idx]});
      joint_map_[joint_names_[idx]] = idx;
    }


    pub_state_ = nh.advertise<sensor_msgs::JointState>("/sexy/dongsu/gimbal/recv/state", 1000);
    pub_desired_ = nh.advertise<sensor_msgs::JointState>("/sexy/dongsu/gimbal/recv/desired", 1000);
    pub_parameter_ = nh.advertise<sensor_msgs::JointState>("/sexy/dongsu/gimbal/recv/parameter", 1000);

    all_data_client_   = nh.serviceClient<std_srvs::Empty>("/all_data_reset_cmd");
    euler_init_client_ = nh.serviceClient<std_srvs::Empty>("/euler_angle_init_cmd");
    euler_reset_client_= nh.serviceClient<std_srvs::Empty>("/euler_angle_reset_cmd");
    pose_reset_client_ = nh.serviceClient<std_srvs::Empty>("/pose_velocity_reset_cmd");
    reboot_client_     = nh.serviceClient<std_srvs::Empty>("/reboot_sensor_cmd");

    sub_desired_ = nh.subscribe("/sexy/dongsu/gimbal/desired/control", 1000, &Controller::jointControllCallback, this);
    sub_desired_param_ = nh.subscribe("/sexy/dongsu/gimbal/desired/parameter", 1000, &Controller::jointControllCallback2, this);

    sub_imu_reset_ =  nh.subscribe("/clicked_point", 1000, &Controller::imuResetCallback, this);
    sub_init_ =  nh.subscribe("/initialpose", 1000, &Controller::initCallback, this);
    sub_end_ =  nh.subscribe("/move_base_simple/goal", 1000, &Controller::endCallback, this);
    return true;
  }

  void Controller::starting(const ros::Time &time)
  {
    std::cerr << "SexyDongsu_Gimbal_controller started." << std::endl;
    for (auto &kv : joint_map_)
    {
      hybridJointHandles_[kv.second].setCommand(
        init_cmd_[kv.first].position,
        init_cmd_[kv.first].velocity,
        pd_cmd_[kv.first].kp,
        pd_cmd_[kv.first].kd,
        init_cmd_[kv.first].feedforward,
        init_cmd_[kv.first].mode);
    }
    std_srvs::Empty srv;
    all_data_client_.call(srv);
    euler_reset_client_.call(srv);
    pose_reset_client_.call(srv);
    euler_init_client_.call(srv);
  }

  void Controller::update(const ros::Time &time, const ros::Duration &period)
  {
    sensor_msgs::JointState state_msg;
    state_msg.header.stamp = time; 

    sensor_msgs::JointState desired_msg;
    sensor_msgs::JointState parameter_msg;
    for (auto &kv : joint_map_)
    {
      state_msg.name.push_back(kv.first);
      recv_cmd_[kv.first].velocity = hybridJointHandles_[kv.second].getVelocity();
      recv_cmd_[kv.first].velocity_desired = hybridJointHandles_[kv.second].getVelocityDesired();
      recv_cmd_[kv.first].position = hybridJointHandles_[kv.second].getPosition();
      recv_cmd_[kv.first].position_desired = hybridJointHandles_[kv.second].getPositionDesired();
      recv_cmd_[kv.first].effort = hybridJointHandles_[kv.second].getEffort();
      recv_cmd_[kv.first].feedforward = hybridJointHandles_[kv.second].getFeedforward();
      recv_cmd_[kv.first].kp = hybridJointHandles_[kv.second].getKp();
      recv_cmd_[kv.first].kd = hybridJointHandles_[kv.second].getKd();

      std::map<std::string,SexyDonguCommand> command;

      if(!desired_cmd_ && !last_cmd_)
        command = init_cmd_;
      else if(!desired_cmd_)
        command = *last_cmd_;
      else
        command = *desired_cmd_;

      {
        std::lock_guard<std::mutex> lock(sexy_dongsu_mutex_);
        hybridJointHandles_[kv.second].setCommand(
            command[kv.first].position,
            command[kv.first].velocity,
            pd_cmd_[kv.first].kp,
            pd_cmd_[kv.first].kd,
            command[kv.first].feedforward,
            command[kv.first].mode);
      }
      desired_msg.header = state_msg.header;
      desired_msg.name = state_msg.name;
      parameter_msg.header = state_msg.header;
      parameter_msg.name = state_msg.name;

      state_msg.position.push_back(recv_cmd_[kv.first].position);
      state_msg.velocity.push_back(recv_cmd_[kv.first].velocity);
      state_msg.effort.push_back(recv_cmd_[kv.first].effort);

      desired_msg.position.push_back(recv_cmd_[kv.first].position_desired);
      desired_msg.velocity.push_back(recv_cmd_[kv.first].velocity_desired);
      desired_msg.velocity.push_back(recv_cmd_[kv.first].feedforward);

      parameter_msg.position.push_back(recv_cmd_[kv.first].kp);
      parameter_msg.velocity.push_back(recv_cmd_[kv.first].kd);
      parameter_msg.effort.push_back(recv_cmd_[kv.first].feedforward);
      {
        std::lock_guard<std::mutex> lock(sexy_dongsu_mutex_);
        desired_cmd_.reset();
      }
    if(!last_cmd_)
      last_cmd_ = std::make_shared<std::map<std::string,SexyDonguCommand>>();
    last_cmd_ = desired_cmd_;

    }
    pub_state_.publish(state_msg);
    pub_desired_.publish(desired_msg);
    pub_parameter_.publish(parameter_msg);

  }

  void Controller::jointControllCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    int joint_num =  msg->name.size();
    int num_cmd = msg->position.size() +  msg->velocity.size() + msg->effort.size();
    int prefix = num_cmd / joint_num;
    if((num_cmd % msg->name.size()) != 0 || num_cmd == 0)
      return;
    if(!desired_cmd_)
    {
      std::lock_guard<std::mutex> lock(sexy_dongsu_mutex_);
      desired_cmd_ = std::make_shared<std::map<std::string,SexyDonguCommand>>();
    }
    sexy::dongsu::motor::hardware::Control_Mode mode;
    std::vector<double> position(joint_num,0.0);
    std::vector<double> velocity(joint_num,0.0);
    std::vector<double> feedforward(joint_num,0.0);
    if(prefix == 1)
    {
      mode = sexy::dongsu::motor::hardware::Control_Mode::VEL_MODE;
      velocity = msg->velocity;
    }
    else if(prefix == 2)
    {
      mode = sexy::dongsu::motor::hardware::Control_Mode::POS_VEL_MODE;
      velocity = msg->velocity;
      position = msg->position;
    }
    else if(prefix == 3)
    {
      mode = sexy::dongsu::motor::hardware::Control_Mode::MIT_MODE;
      velocity = msg->velocity;
      position = msg->position;
      feedforward = msg->effort;
    }
    {
      std::lock_guard<std::mutex> lock(sexy_dongsu_mutex_);
      for(auto &n : msg->name)
      {
          desired_cmd_->at(n).mode = mode;
          desired_cmd_->at(n).position = position[joint_map_[n]];
          desired_cmd_->at(n).velocity = velocity[joint_map_[n]];
          desired_cmd_->at(n).feedforward = feedforward[joint_map_[n]];
      }
    }

  }

  void Controller::jointControllCallback2(const sensor_msgs::JointState::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(sexy_dongsu_mutex_);
    for(auto &n : msg->name)
    {
      pd_cmd_[n].kp = msg->position[joint_map_[n]];
      pd_cmd_[n].kd = msg->velocity[joint_map_[n]];
    }
  }


  void Controller::stopping(const ros::Time &time)
  {
    for (auto &kv : joint_map_)
      hybridJointHandles_[kv.second].setCommand(
      stop_cmd_[kv.first].position,
      stop_cmd_[kv.first].velocity,
      pd_cmd_[kv.first].kp,
      pd_cmd_[kv.first].kd,
      stop_cmd_[kv.first].feedforward,
      stop_cmd_[kv.first].mode);

  }


  void Controller::imuResetCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    std_srvs::Empty srv;
    reboot_client_.call(srv);
  }

  void Controller::initCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    std::cerr << "SexyDongsu_Gimbal_controller re-init." << std::endl;
    starting(msg->header.stamp);
    {
      std::mutex sexy_dongsu_mutex_;
      desired_cmd_.reset();
      last_cmd_.reset();
    }
  }

  void Controller::endCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    std_srvs::Empty srv;
    reboot_client_.call(srv);
    stopping(msg->header.stamp);
  }
}


PLUGINLIB_EXPORT_CLASS(sexy::dongsu::motor::Controller, controller_interface::ControllerBase);