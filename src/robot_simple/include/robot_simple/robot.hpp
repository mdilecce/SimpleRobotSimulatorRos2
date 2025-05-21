#ifndef   ROBOT_SIMPLE_ROBOT_HPP
#define   ROBOT_SIMPLE_ROBOT_HPP

#include "robot_simple/battery.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/action/move_to_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <map>
#include <string>
#include <vector>

namespace robot_simple
{
    class Robot: public rclcpp::Node
    {
        public:
        
            Robot(const rclcpp::NodeOptions & options);

            Robot(
                const std::string &name = "robot",
                const std::string &battery = "battery",
                const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
            );


        private:
            std::string battery_name_;

            tf2::Transform frame_robot_;
            double speed_ = 0;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  
            
            tf2::Vector3 posBaseGlobal_;
            
            double acceleration_ = 0.5;
            double deceleration_ = 1;
            double speedMax_ = 2;
            double mass_ = 5;
            double fresistive_ = 2;
            double stepTime_ = 1;
            double tollerance_ = 0.1;
            double odometer_ = 0;

            std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> acceleration_handle_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> deceleration_handle_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> speedMax_handle_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> mass_handle_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> fresistive_handle_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> stepTime_handle_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> tollerance_handle_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> position_base_handle_;

            using RobotStatus = robot_interfaces::msg::RobotStatus;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
            
            using BatteryCheck = robot_interfaces::srv::EnergyCheck;

            rclcpp::Client<BatteryCheck>::SharedPtr energy_check_client_;
            rclcpp::Client<BatteryCheck>::SharedPtr energy_request_client_;

            using BatteryCharge = robot_interfaces::action::BatteryCharge;
            using BatteryChargeGoalHandle = rclcpp_action::ClientGoalHandle<BatteryCharge>;
            using MoveToTarget = robot_interfaces::action::MoveToTarget;
            using MoveToTargetGoalHandle = rclcpp_action::ServerGoalHandle<MoveToTarget>;

            rclcpp_action::Client<BatteryCharge>::SharedPtr action_battery_client_;
            rclcpp_action::Server<MoveToTarget>::SharedPtr action_server_;

            rclcpp::TimerBase::SharedPtr move_timer_;

            void initializeParameters();
            void initializePublisher();
            
            void initializeBatteryServices();
            bool checkBattery(double energy = 0) const;
            bool consumeEnergy(double energy = 0);
            void chargeBattery(double energy_required = 0);

            void initializeActionServer();

            double calculateDistance(const tf2::Vector3 &target) const;
            double calculateDistanceToBase(const tf2::Vector3 &target) const;
            double calculateEnergy(double distance) const;

            void rotateRobot(const tf2::Vector3 &target);
            void moveRobot(std::shared_ptr<MoveToTargetGoalHandle> goal_handle, bool returnToBase = false);
            void moveToTarget(std::shared_ptr<MoveToTargetGoalHandle> goal_handle);

            void updateTransform();

        };

}

#endif