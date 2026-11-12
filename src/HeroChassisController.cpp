#include "hero_chassis_controller/HeroChassisController.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>

namespace hero_chassis_controller {

namespace {

/// 把角度归一化到 [-pi, pi]，避免积分后数值越滚越大
double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface *hardware_interface,
    ros::NodeHandle &root_node_handle,
    ros::NodeHandle &controller_node_handle) {
  if (hardware_interface == nullptr) {
    ROS_ERROR("EffortJointInterface is unavailable.");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!controller_node_handle.getParam("joints", joint_names) ||
      joint_names.size() != wheel_joints_.size()) {
    ROS_ERROR(
        "Parameter 'joints' must contain exactly four wheel joint names.");
    return false;
  }

  controller_node_handle.param("wheel_radius", wheel_radius_, wheel_radius_);
  controller_node_handle.param("half_wheelbase", half_wheelbase_,
                               half_wheelbase_);
  controller_node_handle.param("half_track", half_track_, half_track_);
  controller_node_handle.param("max_effort", max_effort_, max_effort_);
  controller_node_handle.param("max_wheel_speed", max_wheel_speed_,
                               max_wheel_speed_);
  controller_node_handle.param("max_wheel_acceleration",
                               max_wheel_acceleration_,
                               max_wheel_acceleration_);
  controller_node_handle.param("cmd_vel_timeout", cmd_vel_timeout_,
                               cmd_vel_timeout_);
  controller_node_handle.param("odom_frame", odom_frame_, odom_frame_);
  controller_node_handle.param("base_frame", base_frame_, base_frame_);
  controller_node_handle.param("world_frame", world_frame_, world_frame_);

  std::string velocity_mode = "chassis";
  controller_node_handle.param("velocity_mode", velocity_mode, velocity_mode);
  if (velocity_mode == "world") {
    world_frame_mode_ = true;
  } else if (velocity_mode == "chassis") {
    world_frame_mode_ = false;
  } else {
    ROS_ERROR_STREAM("Parameter 'velocity_mode' must be 'chassis' or 'world', "
                     "but got '" << velocity_mode << "'.");
    return false;
  }

  if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0 ||
      !std::isfinite(half_wheelbase_) || half_wheelbase_ < 0.0 ||
      !std::isfinite(half_track_) || half_track_ < 0.0) {
    ROS_ERROR("'wheel_radius' must be positive and "
              "'half_wheelbase'/'half_track' non-negative.");
    return false;
  }
  if (!std::isfinite(max_effort_) || max_effort_ <= 0.0) {
    ROS_ERROR("Parameter 'max_effort' must be a finite positive value.");
    return false;
  }

  // 收不到 /cmd_vel 时保持的"测试转速"，用于单独演示 PID 跟踪
  std::vector<double> default_speeds;
  if (controller_node_handle.getParam("target_wheel_speeds", default_speeds) &&
      default_speeds.size() == wheel_joints_.size()) {
    for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
      default_wheel_speeds_[index] = default_speeds[index];
    }
  }

  try {
    for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
      wheel_joints_[index] = hardware_interface->getHandle(joint_names[index]);

      ros::NodeHandle pid_node_handle(controller_node_handle,
                                      "gains/" + joint_names[index]);
      if (!wheel_pids_[index].init(pid_node_handle)) {
        ROS_ERROR_STREAM("Could not initialize PID for joint "
                         << joint_names[index]);
        return false;
      }
      // Pid::init() 内部已经建好 dynamic_reconfigure，rqt_reconfigure 可在线调参
    }
  } catch (const hardware_interface::HardwareInterfaceException &exception) {
    ROS_ERROR_STREAM("Could not get wheel joint handle: " << exception.what());
    return false;
  }

  odom_publisher_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(
      root_node_handle, "odom", 10));
  for (int index = 0; index < 6; ++index) {
    odom_publisher_->msg_.pose.covariance[index * 7] = 1e-3;
    odom_publisher_->msg_.twist.covariance[index * 7] = 1e-3;
  }

  target_speed_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(
          root_node_handle, "target_wheel_speeds", 10));
  target_speed_publisher_->msg_.layout.dim.resize(1);
  target_speed_publisher_->msg_.layout.dim[0].label = "wheel";
  target_speed_publisher_->msg_.layout.dim[0].size =
      static_cast<uint32_t>(wheel_joints_.size());
  target_speed_publisher_->msg_.layout.dim[0].stride = 1;
  target_speed_publisher_->msg_.data.resize(wheel_joints_.size());

  cmd_vel_subscriber_ =
      root_node_handle.subscribe("cmd_vel", 10,
                                 &HeroChassisController::cmdVelCallback, this);

  ROS_INFO_STREAM(
      "Hero chassis controller initialized: r="
      << wheel_radius_ << " m, lx=" << half_wheelbase_ << " m, ly="
      << half_track_ << " m, mode=" << (world_frame_mode_ ? "world" : "chassis")
      << ", command topic='/cmd_vel', odometry topic='/odom'.");
  return true;
}

void HeroChassisController::starting(const ros::Time &time) {
  for (std::size_t index = 0; index < wheel_pids_.size(); ++index) {
    wheel_pids_[index].reset();
    target_wheel_speeds_[index] = 0.0;
    last_wheel_speeds_[index] = 0.0;
    wheel_joints_[index].setCommand(0.0);
  }

  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_theta_ = 0.0;
  last_odom_time_ = time;

  target_velocity_ = geometry_msgs::Twist();
  cmd_vel_received_ = false;

  ROS_INFO("Hero chassis controller started, odometry reset.");
}

void HeroChassisController::stopping(const ros::Time &) {
  for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
    target_wheel_speeds_[index] = 0.0;
    last_wheel_speeds_[index] = 0.0;
    wheel_joints_[index].setCommand(0.0);
  }
  ROS_INFO("Hero chassis controller stopped.");
}

void HeroChassisController::update(const ros::Time &time,
                                   const ros::Duration &period) {
  const double period_seconds = period.toSec();

  // 1) 期望轮速从哪来：/cmd_vel 逆运动学，或超时停车，或默认测试转速
  const bool cmd_vel_active =
      cmd_vel_received_ && cmd_vel_timeout_ > 0.0 &&
      (time - last_cmd_vel_time_).toSec() <= cmd_vel_timeout_;

  std::array<double, 4> desired_speeds = default_wheel_speeds_;
  if (cmd_vel_active) {
    inverseKinematics(target_velocity_.linear.x, target_velocity_.linear.y,
                      target_velocity_.angular.z, &desired_speeds);
  } else if (cmd_vel_received_ && cmd_vel_timeout_ > 0.0) {
    desired_speeds.fill(0.0);
  }

  // 把逆运动学算出的期望轮速发出来，方便和实际轮速逐项对表
  publishTargetSpeeds(time, desired_speeds);

  // 2) 加速度限制 + 角速度限幅，然后逐轮做 PID
  std::array<double, 4> measured_speeds{};
  for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
    double command =
        std::max(-max_wheel_speed_, std::min(max_wheel_speed_, desired_speeds[index]));

    if (max_wheel_acceleration_ > 0.0 && period_seconds > 0.0) {
      const double max_delta = max_wheel_acceleration_ * period_seconds;
      const double current = last_wheel_speeds_[index];
      command = std::max(current - max_delta, std::min(current + max_delta, command));
    }

    target_wheel_speeds_[index] = command;
    last_wheel_speeds_[index] = command;

    const double speed_error = command - wheel_joints_[index].getVelocity();
    const double raw_effort = wheel_pids_[index].computeCommand(speed_error, period);
    wheel_joints_[index].setCommand(
        std::max(-max_effort_, std::min(max_effort_, raw_effort)));

    measured_speeds[index] = wheel_joints_[index].getVelocity();
  }

  // 3) 正运动学 + 积分，得到里程计
  if (last_odom_time_.isZero()) {
    last_odom_time_ = time;
    return;
  }
  const double delta_time = (time - last_odom_time_).toSec();
  if (delta_time <= 0.0) {
    return;
  }

  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  forwardKinematics(measured_speeds, &vx, &vy, &wz);

  // 用本周期内的平均航向角积分，比直接用末端航向角更接近真实轨迹
  const double previous_theta = odom_theta_;
  odom_theta_ = normalizeAngle(odom_theta_ + wz * delta_time);
  const double mean_theta = 0.5 * (previous_theta + odom_theta_);
  odom_x_ += (vx * std::cos(mean_theta) - vy * std::sin(mean_theta)) * delta_time;
  odom_y_ += (vx * std::sin(mean_theta) + vy * std::cos(mean_theta)) * delta_time;
  last_odom_time_ = time;

  publishOdometry(time, vx, vy, wz);
}

void HeroChassisController::cmdVelCallback(
    const geometry_msgs::Twist &command) {
  geometry_msgs::Twist target = command;

  if (world_frame_mode_) {
    // 选做 7：/cmd_vel 在世界坐标系 (odom/map) 下，用 tf 旋转到底盘坐标系
    try {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(base_frame_, world_frame_, ros::Time(0),
                                   transform);
      const tf::Vector3 linear_in_world(command.linear.x, command.linear.y, 0.0);
      const tf::Vector3 linear_in_base = transform.getBasis() * linear_in_world;
      target.linear.x = linear_in_base.x();
      target.linear.y = linear_in_base.y();
    } catch (const tf::TransformException &exception) {
      // tf 还没准备好时，退化成用自己积分出来的航向角做旋转
      ROS_WARN_THROTTLE(1.0,
                        "World frame velocity mode: tf unavailable (%s), "
                        "falling back to integrated yaw.",
                        exception.what());
      target.linear.x = std::cos(odom_theta_) * command.linear.x +
                        std::sin(odom_theta_) * command.linear.y;
      target.linear.y = -std::sin(odom_theta_) * command.linear.x +
                        std::cos(odom_theta_) * command.linear.y;
    }
  }

  target_velocity_ = target;
  last_cmd_vel_time_ = ros::Time::now();
  cmd_vel_received_ = true;
}

void HeroChassisController::inverseKinematics(
    double vx, double vy, double wz, std::array<double, 4> *wheel_speeds) const {
  // 底盘坐标系：x 向前、y 向左、z 向上，wz > 0 为逆时针（向左转）。
  // 第 i 个轮子的接地点速度：Vx_i = vx - wz * y_i,  Vy_i = vy + wz * x_i
  // 麦轮的 45° 辊子把这两个分量按 ±1 叠加成轮子自身的滚动速度：
  //   左前: Vx - Vy    右前: Vx + Vy    左后: Vx + Vy    右后: Vx - Vy
  // 代入 x_i = ±lx, y_i = ±ly，得到下面四个式子（k = lx + ly）。
  const double k = half_wheelbase_ + half_track_;
  (*wheel_speeds)[0] = (vx - vy - k * wz) / wheel_radius_;  // 左前
  (*wheel_speeds)[1] = (vx + vy + k * wz) / wheel_radius_;  // 右前
  (*wheel_speeds)[2] = (vx + vy - k * wz) / wheel_radius_;  // 左后
  (*wheel_speeds)[3] = (vx - vy + k * wz) / wheel_radius_;  // 右后
}

void HeroChassisController::forwardKinematics(
    const std::array<double, 4> &wheel_speeds, double *vx, double *vy,
    double *wz) const {
  // 逆运动学矩阵 A(4x3) 的列互相正交，最小二乘解就是列方向上的加权和。
  const double w0 = wheel_speeds[0];  // 左前
  const double w1 = wheel_speeds[1];  // 右前
  const double w2 = wheel_speeds[2];  // 左后
  const double w3 = wheel_speeds[3];  // 右后
  const double k = half_wheelbase_ + half_track_;

  *vx = wheel_radius_ * 0.25 * (w0 + w1 + w2 + w3);
  *vy = wheel_radius_ * 0.25 * (-w0 + w1 + w2 - w3);
  *wz = wheel_radius_ * 0.25 * (-w0 + w1 - w2 + w3) / k;
}

void HeroChassisController::publishOdometry(const ros::Time &time, double vx,
                                            double vy, double wz) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(odom_x_, odom_y_, 0.0));
  transform.setRotation(tf::createQuaternionFromYaw(odom_theta_));
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, time, odom_frame_, base_frame_));

  if (odom_publisher_ && odom_publisher_->trylock()) {
    nav_msgs::Odometry &message = odom_publisher_->msg_;
    message.header.stamp = time;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = odom_x_;
    message.pose.pose.position.y = odom_y_;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_theta_);
    message.twist.twist.linear.x = vx;
    message.twist.twist.linear.y = vy;
    message.twist.twist.angular.z = wz;
    odom_publisher_->unlockAndPublish();
  }
}

void HeroChassisController::publishTargetSpeeds(
    const ros::Time &time, const std::array<double, 4> &speeds) {
  // 控制在 50 Hz 左右发布，避免 1 kHz 控制周期把 rqt_plot 刷爆
  if (!target_speed_publisher_ ||
      (time - last_speed_publish_time_).toSec() < 0.02) {
    return;
  }
  last_speed_publish_time_ = time;
  if (!target_speed_publisher_->trylock()) {
    return;
  }
  std_msgs::Float64MultiArray &message = target_speed_publisher_->msg_;
  message.layout.data_offset = 0;
  for (std::size_t index = 0; index < speeds.size(); ++index) {
    message.data[index] = speeds[index];
  }
  target_speed_publisher_->unlockAndPublish();
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)