//
// Created by yuchen on 2023/3/10.
//

#pragma once

#include "inverted_pendulum_msg/InvertedPendulumCmd.h"
#include "inverted_pendulum_msg/InvertedPendulumState.h"
#include "inverted_pendulum_msg/QRConfig.h"

#include <angles/angles.h>
#include <cmath>
#include <controller_interface/multi_interface_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/lqr.h>

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
  void moveJoint(const ros::Time &time, const ros::Duration &period);
  void
  commandCB(const inverted_pendulum_msg::InvertedPendulumCmdConstPtr &data) {
    cmd_ = *data;
  }
  void reconfigCB(inverted_pendulum_msg::QRConfig &config);
  static const int STATE_DIM = 4;   // theta1,theta2,dtheta1,dtheta2
  static const int CONTROL_DIM = 2; // T1,T2
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_;

  hardware_interface::EffortJointInterface *effort_joint_interface_;
  hardware_interface::JointHandle leg_1_handle_, leg_2_handle_;

  bool dynamic_reconfig_initialized_ = false;

  dynamic_reconfigure::Server<inverted_pendulum_msg::QRConfig> *reconf_server_;
  double q_dynamic_[STATE_DIM], r_dynamic_[CONTROL_DIM], q_config_[STATE_DIM],
      r_config_[CONTROL_DIM];

  inverted_pendulum_msg::InvertedPendulumCmd cmd_;
  ros::Subscriber cmd_subscriber_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<
      inverted_pendulum_msg::InvertedPendulumState>>
      RtpublisherPtr;
  RtpublisherPtr leg_state_pub_;
};

} // namespace inverted_pendulum_controller