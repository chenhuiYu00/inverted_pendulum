//
// Created by yuchen on 2023/3/10.
//

#include "inverted_pendulum_controller/controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace inverted_pendulum_controller {
bool Controller::init(hardware_interface::RobotHW *robot_hw,
                      ros::NodeHandle &root_nh,
                      ros::NodeHandle &controller_nh) {
  // cmd subscriber
  cmd_subscriber_ =
      controller_nh
          .subscribe<inverted_pendulum_controller::InvertedPendulumCmd>(
              "command", 1, &Controller::commandCB, this);
  // realtime publisher
  leg_state_pub_.reset(new realtime_tools::RealtimePublisher<
                       inverted_pendulum_controller::InvertedPendulumCmd>(
      controller_nh, "leg_states", 100));

  ros::NodeHandle nh_leg_1 = ros::NodeHandle(controller_nh, "leg_1");
  ros::NodeHandle nh_leg_2 = ros::NodeHandle(controller_nh, "leg_2");
  effort_joint_interface_ =
      robot_hw->get<hardware_interface::EffortJointInterface>();

  return ctrl_leg_1_.init(effort_joint_interface_, nh_leg_1) &&
         ctrl_leg_2_.init(effort_joint_interface_, nh_leg_2);
}

void Controller::starting(const ros::Time & /*time*/) {}

void Controller::update(const ros::Time &time, const ros::Duration &period) {
  ctrl_leg_1_.setCommand(cmd_.leg_1);
  ctrl_leg_2_.setCommand(cmd_.leg_2);

  if (leg_state_pub_->trylock()) {
    leg_state_pub_->msg_.leg_1 = ctrl_leg_1_.getPosition();
    leg_state_pub_->msg_.leg_2 = ctrl_leg_2_.getPosition();
    leg_state_pub_->msg_.stamp = ros::Time::now();

    leg_state_pub_->unlockAndPublish();
  }
  ctrl_leg_1_.update(time, period);
  ctrl_leg_2_.update(time, period);
}
} // namespace inverted_pendulum_controller
PLUGINLIB_EXPORT_CLASS(inverted_pendulum_controller::Controller,
                       controller_interface::ControllerBase)
