#ifndef HERO_CHASSIS_CONTROLLER_HEROCHASSISCONTROLLER_HPP_
#define HERO_CHASSIS_CONTROLLER_HEROCHASSISCONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace hero_chassis_controller {

/**
 * @brief 麦克纳姆轮底盘的速度控制器。
 *
 * 结构：/cmd_vel (Twist) --逆运动学--> 四个轮子的期望角速度
 *       --> 每个轮子一个 control_toolbox::Pid --> 力矩指令 (EffortJointInterface)
 *       四个轮子的实测角速度 --正运动学--> 底盘速度 --> 积分成 /odom + tf(odom->base_link)
 *
 * 轮子顺序固定为：左前(0)、右前(1)、左后(2)、右后(3)。
 */
class HeroChassisController
    : public controller_interface::Controller<
          hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::EffortJointInterface *hardware_interface,
            ros::NodeHandle &root_node_handle,
            ros::NodeHandle &controller_node_handle) override;

  void starting(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void stopping(const ros::Time &time) override;

 private:
  /// 接收 /cmd_vel，必要时把世界坐标系速度旋转到底盘坐标系后缓存
  void cmdVelCallback(const geometry_msgs::Twist &command);

  /// 底盘速度 (vx, vy, wz) -> 四个轮子的期望角速度 (rad/s)
  void inverseKinematics(double vx, double vy, double wz,
                         std::array<double, 4> *wheel_speeds) const;

  /// 四个轮子的实测角速度 (rad/s) -> 底盘速度 (vx, vy, wz)
  void forwardKinematics(const std::array<double, 4> &wheel_speeds, double *vx,
                         double *vy, double *wz) const;

  /// 用积分得到的位姿发布 /odom 与 tf(odom -> base_link)
  void publishOdometry(const ros::Time &time, double vx, double vy,
                       double wz);

  /// 把逆运动学算出的四个轮子期望角速度发到 /target_wheel_speeds
  void publishTargetSpeeds(const ros::Time &time,
                           const std::array<double, 4> &speeds);

  std::array<hardware_interface::JointHandle, 4> wheel_joints_;
  std::array<control_toolbox::Pid, 4> wheel_pids_;
  /// 每个轮子当前下发给 PID 的期望角速度（经过加速度限制与限幅）
  std::array<double, 4> target_wheel_speeds_{};
  /// 上一控制周期的期望角速度，用于加速度限制
  std::array<double, 4> last_wheel_speeds_{};
  /// 没有收到 /cmd_vel 时保持的"测试转速"（仅用于演示 PID 跟踪）
  std::array<double, 4> default_wheel_speeds_{};

  double wheel_radius_{0.07625};        ///< 轮子半径 r (m)
  double half_wheelbase_{0.2};          ///< 前/后轮到 base_link 的 x 距离 lx (m)
  double half_track_{0.2};              ///< 左/右轮到 base_link 的 y 距离 ly (m)
  double max_effort_{3.0};              ///< 力矩限幅 (N*m)
  double max_wheel_speed_{20.0};        ///< 轮子角速度限幅 (rad/s)
  double max_wheel_acceleration_{0.0};  ///< 轮子角加速度限幅 (rad/s^2)，<=0 关闭
  double cmd_vel_timeout_{0.5};         ///< /cmd_vel 超时时间 (s)

  bool world_frame_mode_{false};        ///< true: /cmd_vel 在世界坐标系下
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string world_frame_{"odom"};

  geometry_msgs::Twist target_velocity_;  ///< 底盘坐标系下的速度指令
  bool cmd_vel_received_{false};
  ros::Time last_cmd_vel_time_;

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_theta_{0.0};
  ros::Time last_odom_time_;

  ros::Subscriber cmd_vel_subscriber_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>
      odom_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>
      target_speed_publisher_;
  ros::Time last_speed_publish_time_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HEROCHASSISCONTROLLER_HPP_