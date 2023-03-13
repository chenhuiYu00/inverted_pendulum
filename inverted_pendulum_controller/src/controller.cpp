//
// Created by yuchen on 2023/3/10.
//

#include "inverted_pendulum_controller/controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace inverted_pendulum_controller {
bool Controller::init(hardware_interface::RobotHW *robot_hw,
                      ros::NodeHandle &root_nh,
                      ros::NodeHandle &controller_nh) {

  // m_w is mass of single wheel
  // m is mass of the robot except wheels and momentum_blocks
  // m_b is mass of single momentum_block
  // i_w is the moment of inertia of the wheel around the rotational axis of the
  // motor l is the vertical component of the distance between the wheel center
  // and the center of mass of robot y_b is the y-axis component of the
  // coordinates of the momentum block in the base_link coordinate system z_b is
  // the vertical component of the distance between the momentum block and the
  // center of mass of robot i_m is the moment of inertia of the robot around
  // the y-axis of base_link coordinate.
  double m1, l_c1, L1, m2, l_c2, L2, g;

  if (!controller_nh.getParam("m1", m1)) {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m2", m2)) {
    ROS_ERROR("Params m doesn't given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l_c1", l_c1)) {
    ROS_ERROR("Params m_b doesn't given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("L1", L1)) {
    ROS_ERROR("Params i_w doesn't given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l_c2", l_c2)) {
    ROS_ERROR("Params l doesn't given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("L2", L2)) {
    ROS_ERROR("Params y_b doesn't given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("g", g)) {
    ROS_ERROR("Params z_b doesn't given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }

  q_.setZero();
  r_.setZero();
  XmlRpc::XmlRpcValue q, r;
  controller_nh.getParam("q", q);
  controller_nh.getParam("r", r);
  // Check and get Q
  ROS_ASSERT(q.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(q.size() == STATE_DIM);
  for (int i = 0; i < STATE_DIM; ++i) {
    ROS_ASSERT(q[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
               q[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (q[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      q_(i, i) = static_cast<double>(q[i]);
      q_config_[i] = static_cast<double>(q[i]);
    } else if (q[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      q_(i, i) = static_cast<int>(q[i]);
      q_config_[i] = static_cast<int>(q[i]);
    }
  }
  // Check and get R
  ROS_ASSERT(r.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(r.size() == CONTROL_DIM);
  for (int i = 0; i < CONTROL_DIM; ++i) {
    ROS_ASSERT(r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
               r[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      r_(i, i) = static_cast<double>(r[i]);
      r_config_[i] = static_cast<double>(r[i]);
    } else if (r[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      r_(i, i) = static_cast<int>(r[i]);
      r_config_[i] = static_cast<int>(r[i]);
    }
  }

  // Continuous model \dot{x} = A x + B u
  double a_0_2, a_1_3, a_2_0, a_2_1, a_3_0, a_3_1;
  a_0_2 = 1.0;
  a_1_3 = 1.0;
  a_2_0 = (-g * l_c1 * m1 * cos(1) -
           g * m2 * (L1 * cos(1) + 0.362357754476674 * l_c2)) /
          (pow(l_c1, 2) * m1);
  a_2_1 = -0.362357754476674 * g * l_c2 * m2 / (pow(l_c1, 2) * m1);
  a_3_0 = -0.362357754476674 * g / l_c2;
  a_3_1 = -0.362357754476674 * g / l_c2;
  a_ << 0., 0., a_0_2, 0., 0., 0., 0., a_1_3, a_2_0, a_2_1, 0., 0., a_3_0,
      a_3_1, 0., 0.;

  double b_2_0, b_2_1, b_3_1;
  b_2_0 = 1 / (pow(l_c1, 2) * m1);
  b_2_1 = -1 / (pow(l_c1, 2) * m1);
  b_3_1 = 1 / (pow(l_c2, 2) * m2);
  b_ << 0., 0., 0., 0., b_2_0, b_2_1, 0., b_3_1;

  ROS_INFO_STREAM("A:" << a_);
  ROS_INFO_STREAM("B:" << b_);

  // Lqr
  Lqr<double> lqr(a_, b_, q_, r_);
  if (!lqr.computeK()) {
    ROS_ERROR("Failed to compute K of LQR.");
    return false;
  }
  k_ = lqr.getK();
  ROS_INFO_STREAM("K of LQR:" << k_);

  ros::NodeHandle nh_leg_1 = ros::NodeHandle(controller_nh, "leg_1");
  ros::NodeHandle nh_leg_2 = ros::NodeHandle(controller_nh, "leg_2");
  effort_joint_interface_ =
      robot_hw->get<hardware_interface::EffortJointInterface>();

  std::string leg_1_joint, leg_2_joint;
  if (!nh_leg_1.getParam("joint", leg_1_joint) ||
      !nh_leg_2.getParam("joint", leg_2_joint)) {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  leg_1_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(
          leg_1_joint);
  leg_2_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(
          leg_2_joint);

  // cmd subscriber
  cmd_subscriber_ =
      controller_nh.subscribe<inverted_pendulum_msg::InvertedPendulumCmd>(
          "command", 1, &Controller::commandCB, this);
  // realtime publisher
  leg_state_pub_.reset(new realtime_tools::RealtimePublisher<
                       inverted_pendulum_msg::InvertedPendulumState>(
      controller_nh, "leg_states", 100));
  // dynamic config
  reconf_server_ =
      new dynamic_reconfigure::Server<inverted_pendulum_msg::QRConfig>(
          ros::NodeHandle("~/qr"));
  dynamic_reconfigure::Server<inverted_pendulum_msg::QRConfig>::CallbackType
      cb = boost::bind(&Controller::reconfigCB, this, _1);
  reconf_server_->setCallback(cb);

  leg_state_pub_->msg_.position.resize(2);
  leg_state_pub_->msg_.velocity.resize(2);

  return true;
}

void Controller::starting(const ros::Time & /*time*/) {}

void Controller::moveJoint(const ros::Time &time, const ros::Duration &period) {
  x_[0] = leg_1_handle_.getPosition();
  x_[1] = leg_2_handle_.getPosition();
  x_[2] = leg_1_handle_.getVelocity();
  x_[3] = leg_2_handle_.getVelocity();

  double vel_1_cmd = angles::shortest_angular_distance(cmd_.leg_1, x_(0)) /
                     (10 * period.toSec());
  double vel_2_cmd = angles::shortest_angular_distance(cmd_.leg_2, x_(1)) /
                     (10 * period.toSec());

  auto x = x_;
  x(0) -= vel_1_cmd * period.toSec();
  x(1) -= vel_2_cmd * period.toSec();
  x(2) -= vel_1_cmd;
  x(3) -= vel_2_cmd;
  Eigen::Matrix<double, CONTROL_DIM, 1> u;
  u = k_ * (-x);
  leg_1_handle_.setCommand(u(0));
  leg_2_handle_.setCommand(u(1));
}

void Controller::update(const ros::Time &time, const ros::Duration &period) {
  if (leg_state_pub_->trylock()) {
    leg_state_pub_->msg_.position[0] = leg_1_handle_.getPosition();
    leg_state_pub_->msg_.position[1] = leg_2_handle_.getPosition();
    leg_state_pub_->msg_.velocity[0] = leg_1_handle_.getVelocity();
    leg_state_pub_->msg_.velocity[1] = leg_2_handle_.getVelocity();
    leg_state_pub_->msg_.stamp = ros::Time::now();

    leg_state_pub_->unlockAndPublish();
  }
  moveJoint(time, period);
}

void Controller::reconfigCB(inverted_pendulum_msg::QRConfig &config) {
  if (!dynamic_reconfig_initialized_) {
    config.q_0 = q_config_[0];
    config.q_1 = q_config_[1];
    config.q_2 = q_config_[2];
    config.q_3 = q_config_[3];
    config.r_0 = r_config_[0];
    config.r_1 = r_config_[1];
    dynamic_reconfig_initialized_ = true;
    return;
  }
  ROS_INFO("[QR] Dynamic params change");
  q_dynamic_[0] = config.q_0;
  q_dynamic_[1] = config.q_1;
  q_dynamic_[2] = config.q_2;
  q_dynamic_[3] = config.q_3;
  r_dynamic_[0] = config.r_0;
  r_dynamic_[1] = config.r_1;

  // Update Q
  for (int i = 0; i < STATE_DIM; ++i) {
    q_(i, i) = static_cast<double>(q_dynamic_[i]);
  }
  // Update R
  for (int i = 0; i < CONTROL_DIM; ++i) {
    r_(i, i) = static_cast<double>(r_dynamic_[i]);
  }

  Lqr<double> lqr(a_, b_, q_, r_);
  if (!lqr.computeK()) {
    ROS_ERROR("Failed to compute K of LQR.");
  }

  k_ = lqr.getK();
  ROS_INFO_STREAM("K of LQR:" << k_);
}
} // namespace inverted_pendulum_controller
PLUGINLIB_EXPORT_CLASS(inverted_pendulum_controller::Controller,
                       controller_interface::ControllerBase)
