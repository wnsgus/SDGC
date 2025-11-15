#include "sexy_dongsu_dm_controller/SexyDongsuHWLoop.h"
#include "sexy_dongsu_dm_controller/SexyDongsuHW.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dongsu");
  ros::NodeHandle nh;
  ros::NodeHandle robot_hw_nh("damiao");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try
  {
    std::shared_ptr<sexy::dongsu::motor::hardware::SexyDongsuHW> motor_dm_hw = std::make_shared<sexy::dongsu::motor::hardware::SexyDongsuHW>();

    motor_dm_hw->init(robot_hw_nh, robot_hw_nh);

    sexy::dongsu::motor::hardware::SexyDongsuLoop control_loop(nh, motor_dm_hw);

    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}
