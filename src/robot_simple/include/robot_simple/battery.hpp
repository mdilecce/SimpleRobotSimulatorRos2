#ifndef ROBOT_SIMPLE_BATTERY_HPP
#define ROBOT_SIMPLE_BATTERY_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_interfaces/action/battery_charge.hpp"
#include "robot_interfaces/msg/battery_status.hpp"
#include "robot_interfaces/srv/energy_check.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <vector>

namespace robot_simple
{
    class Battery: public rclcpp::Node
    {
        public:
            Battery(const rclcpp::NodeOptions& options);
            explicit Battery(
                double capacity = 250.0, 
                double charge_rate = 10000.0, 
                const std::string &name = "battery", 
                const std::string &container_name = "robot",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
            ); 



            double capacity_; // in Ah
            double energy_; // in Wh
            double charge_rate_; // in W
            double tollerance_ = 0.1; // in m

            std::vector<double> posBaseGlobal_ = {0, 0};

            rclcpp::TimerBase::SharedPtr timer_;
            using BatteryStatus = robot_interfaces::msg::BatteryStatus;
            rclcpp::Publisher<BatteryStatus>::SharedPtr status_pub_;
            
            std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> param_change_handle_;

            using BatteryCheck = robot_interfaces::srv::EnergyCheck;
            rclcpp::Service<BatteryCheck>::SharedPtr energy_check_service_;
            rclcpp::Service<BatteryCheck>::SharedPtr energy_request_service_;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            
            using BatteryCharge = robot_interfaces::action::BatteryCharge;
            using BatteryChargeGoalHandle = rclcpp_action::ServerGoalHandle<BatteryCharge>;
            rclcpp_action::Server<BatteryCharge>::SharedPtr action_server_;

            void chargeBattery(std::shared_ptr<BatteryChargeGoalHandle> goal_handle);
            int getBatteryPercentage() const;
            bool checkBattery(double energy = 0) const;
            bool consumeEnergy(double energy = 0);

            void createPublisher();
            void createServices();
            void createActionServer();
            void createParameterEventHandler();
            void createTransformListener();

            std::string parent_frame_;
    };
}

#endif