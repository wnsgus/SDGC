#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include "dm_common/HybridJointInterface.h"
#include "dm_hw/damiao.h"

namespace sexy::dongsu::moter
{
using namespace damiao;

class SexyDongsuJointHandle : public  HybridJointHandle
{
public:
  SexyDongsuJointHandle() = default;

  SexyDongsuJointHandle(const JointStateHandle& js, double* posDes, double* velDes, double* kp, double* kd, double* ff,damiao::Control_Mode* mode)
    : HybridJointHandle(js,posDes,velDes,kp, kd, ff), mode_(mode)
  {
    if (mode == nullptr)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. mode data pointer is null.");
    }
 
  }
  void setMode(damiao::Control_Mode cmd)
  {
    assert(mode_);
    *mode_ = cmd;
  }
  double getMode()
  {
    assert(mode_);
    return *mode_;
  }

  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff, damiao::Control_Mode mode)
  {
    setMode(mode);
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
  }

private:
  damiao::Control_Mode* mode_ = { nullptr };
  double* ff_ = { nullptr };
};

class SexyDongsuJointInterface
  : public hardware_interface::HardwareResourceManager<SexyDongsuJointHandle, hardware_interface::ClaimResources>
{
};

}  // namespace damiao
