#pragma once

#include <array>

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>

namespace hero_chassis_controller {

    class HeroChassisController
        : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        bool init(hardware_interface::EffortJointInterface *hardware_interface,
                  ros::NodeHandle &root_node_handle,
                  ros::NodeHandle &controller_node_handle) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

    private:
        std::array<hardware_interface::JointHandle, 4> wheel_joints_;
        std::array<control_toolbox::Pid, 4> wheel_pids_;
        std::array<double, 4> target_wheel_speeds_{};
        double max_effort_{0.2};
    };

}  // namespace hero_chassis_controller