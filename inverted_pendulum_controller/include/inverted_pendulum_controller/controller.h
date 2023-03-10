//
// Created by yuchen on 2023/3/10.
//

#pragma once

#include "inverted_pendulum_controller/InvertedPendulumCmd.h"

#include <controller_interface/multi_interface_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

namespace inverted_pendulum_controller {
class Controller : public controller_interface::MultiInterfaceController<
                       hardware_interface::EffortJointInterface> {
public:
  Controller() = default;
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void starting(const ros::Time & /*time*/) override;

private:
  void commandCB(
      const inverted_pendulum_controller::InvertedPendulumCmdConstPtr &data) {
    cmd_ = *data;
  }

  hardware_interface::EffortJointInterface *effort_joint_interface_;
  effort_controllers::JointPositionController ctrl_leg_1_, ctrl_leg_2_;

  inverted_pendulum_controller::InvertedPendulumCmd cmd_;
  ros::Subscriber cmd_subscriber_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<
      inverted_pendulum_controller::InvertedPendulumCmd>>
      RtpublisherPtr;
  RtpublisherPtr leg_state_pub_;
};

} // namespace inverted_pendulum_controller