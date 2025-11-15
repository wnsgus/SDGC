
#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dm_common/HybridJointInterface.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <vector>
#include <optional>

#include <std_srvs/Empty.h>
#include <std_srvs/EmptyResponse.h>
#include <std_srvs/EmptyRequest.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <cstdio>
#include "sexy_dongsu_dm_controller/SexyDongsuJointInterface.h"
#include "sexy_dongsu_dm_controller/SexyDongsuMotor.h"

namespace sexy::dongsu::motor
{
using namespace damiao;
typedef struct 
{
  sexy::dongsu::motor::hardware::Control_Mode mode;
  double position;
  double velocity;
  double feedforward;
}SexyDonguCommand;

typedef struct 
{
  double kp;
  double kd;
}SexyDonguPDCommand;

typedef struct 
{
  sexy::dongsu::motor::hardware::Control_Mode mode;
  double position;
  double position_desired;
  double velocity;
  double velocity_desired;
  double feedforward;
  double effort;
  double kp;
  double kd;
}SexyDonguRecv;


class Controller : public controller_interface::Controller<SexyDongsuJointInterface>
{
public:
  Controller() = default;
  ~Controller() = default;
  std::vector<std::string> joint_names_;
  std::map<std::string,int> joint_map_;
  std::map<std::string,SexyDonguCommand> init_cmd_;
  std::map<std::string,SexyDonguCommand> stop_cmd_;
  std::map<std::string,SexyDonguPDCommand> pd_cmd_;
  std::shared_ptr<std::map<std::string,SexyDonguCommand>> desired_cmd_;
  std::shared_ptr<std::map<std::string,SexyDonguCommand>> last_cmd_;
  std::map<std::string,SexyDonguRecv> recv_cmd_;

  bool init(SexyDongsuJointInterface* robot_hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  std::mutex sexy_dongsu_mutex_;
  std::vector<SexyDongsuJointHandle> hybridJointHandles_;

  ros::Publisher pub_state_;
  ros::Publisher pub_desired_;
  ros::Publisher pub_parameter_;

  ros::ServiceClient all_data_client_;
  ros::ServiceClient euler_init_client_;
  ros::ServiceClient euler_reset_client_;
  ros::ServiceClient pose_reset_client_;
  ros::ServiceClient reboot_client_;

  ros::Subscriber sub_desired_;
  ros::Subscriber sub_desired_param_;
  ros::Subscriber sub_imu_reset_;
  ros::Subscriber sub_init_;
  ros::Subscriber sub_end_;
  void jointControllCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void jointControllCallback2(const sensor_msgs::JointState::ConstPtr& msg);
  void imuResetCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void initCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void endCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

};

}  
