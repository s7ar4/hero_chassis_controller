#include "hero_chassis_controller/HeroChassisController.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <vector>

#include <algorithm>
#include <cmath>

namespace hero_chassis_controller {

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface *hardware_interface,
    ros::NodeHandle &,
    ros::NodeHandle &controller_node_handle) {
  if (hardware_interface == nullptr) {
    ROS_ERROR("EffortJointInterface is unavailable.");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!controller_node_handle.getParam("joints", joint_names) ||
      joint_names.size() != wheel_joints_.size()) {
    ROS_ERROR("Parameter 'joints' must contain exactly four wheel joint names.");
    return false;
  }

  std::vector<double> target_speeds;
  if (!controller_node_handle.getParam("target_wheel_speeds", target_speeds) ||
      target_speeds.size() != target_wheel_speeds_.size()) {
    ROS_ERROR("Parameter 'target_wheel_speeds' must contain four values.");
    return false;
  }

  if (!controller_node_handle.getParam("max_effort", max_effort_) ||
    !std::isfinite(max_effort_) || max_effort_ <= 0.0) {
    ROS_ERROR("Parameter 'max_effort' must be a finite positive value.");
    return false;
    }

  try {
    for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
      wheel_joints_[index] = hardware_interface->getHandle(joint_names[index]);

      ros::NodeHandle pid_node_handle(
          controller_node_handle, "gains/" + joint_names[index]);
      if (!wheel_pids_[index].init(pid_node_handle)) {
        ROS_ERROR_STREAM("Could not initialize PID for joint "
                         << joint_names[index]);
        return false;
      }

      target_wheel_speeds_[index] = target_speeds[index];
    }
  } catch (const hardware_interface::HardwareInterfaceException &exception) {
    ROS_ERROR_STREAM("Could not get wheel joint handle: " << exception.what());
    return false;
  }

  ROS_INFO("Hero chassis wheel-speed PID controller initialized.");
  return true;
}

void HeroChassisController::starting(const ros::Time &) {
  for (std::size_t index = 0; index < wheel_pids_.size(); ++index) {
    wheel_pids_[index].reset();
    wheel_joints_[index].setCommand(0.0);
  }
}

void HeroChassisController::update(const ros::Time &,
                                   const ros::Duration &period) {
  for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
    const double speed_error =
        target_wheel_speeds_[index] - wheel_joints_[index].getVelocity();
    const double raw_effort =
    wheel_pids_[index].computeCommand(speed_error, period);
    const double effort_command =
        std::max(-max_effort_, std::min(max_effort_, raw_effort));
    wheel_joints_[index].setCommand(effort_command);
  }
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(
    hero_chassis_controller::HeroChassisController,
    controller_interface::ControllerBase)