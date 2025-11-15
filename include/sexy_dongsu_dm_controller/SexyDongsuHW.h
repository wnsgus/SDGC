

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>



#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dm_hw/DmHW.h>
#include <dm_hw/DmHWLoop.h>
#include <std_msgs/Bool.h>
#include <dm_hw/damiao.h>

#include "sexy_dongsu_dm_controller/SexyDongsuJointInterface.h"



namespace sexy::dongsu::moter::hardware
{
using namespace damiao;

struct SexyDongsuData : public damiao::DmActData
{
  damiao::Control_Mode mode;
};

class SexyDongsuHW : public damiao::DmHW
{
public:
  SexyDongsuHW() = default;

  bool parseSexyDongsuData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh);
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
  ros::Subscriber sub_;
protected:
  DM_Motor_Type motorType;
  std::vector<std::shared_ptr<Motor_Control>> motor_ports_{};
  std::unordered_map<std::string,  std::unordered_map<int,SexyDongsuData>> port_id2dm_data_{};

  // Interface
  hardware_interface::JointStateInterface jointStateInterface_;  
  SexyDongsuJointInterface hybridJointInterface_;              


private:

};

}  // namespace legged