#include "robot_simple/battery.hpp"

#include <cassert>
#include "tf2/utils.h"
#include <cmath>


namespace robot_simple{

   Battery::Battery(double capacity, double charge_rate, 
            const std::string &name, const std::string &container_name,
            const rclcpp::NodeOptions& options)
        : Node(name, options), 
          capacity_(capacity), 
          energy_(capacity),
          charge_rate_(charge_rate), 
          parent_frame_(container_name){
        RCLCPP_INFO(this->get_logger(), "Battery constructor start");
        this->createPublisher();
        this->createServices();
        this->createActionServer();
        this->createParameterEventHandler();
        this->createTransformListener();
        RCLCPP_INFO(this->get_logger(), "Battery constructor end");
    };  

    Battery::Battery(const rclcpp::NodeOptions& options)
    : Battery(250, 10000, "battery", "robot", options) {}

    void Battery::createPublisher(){

            // Publisher the battery status@S1nello10!
            this->status_pub_ = this->create_publisher<BatteryStatus>(
                "battery_status", rclcpp::QoS(rclcpp::KeepLast(10)));

            // Lambda function publishing battery status
            auto publish_battery_status = [this]() {
                BatteryStatus msg;
                msg.battery_percentage = this->getBatteryPercentage();
                this->status_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), 
                    "Battery Status: %d%%", msg.battery_percentage);
            };
            
            // Timer to periodically publish battery status
            this->timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                publish_battery_status
            );
        }

    void Battery::createParameterEventHandler(){

            // Parameter callback to check base position from node robot
            param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
            auto base_change_handle = [this](const rclcpp::Parameter & p) {
                if (p.get_name() == "position_base_global") {
                    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                        this->posBaseGlobal_ = p.as_double_array();
                        RCLCPP_INFO(this->get_logger(), "Base position updated: [%f, %f]", 
                            this->posBaseGlobal_[0], this->posBaseGlobal_[1]);
                    }
                }
                return rcl_interfaces::msg::SetParametersResult();
            };
            auto remote_node_name = this->parent_frame_;
            auto remote_param_name = std::string("position_base_global");
            this->param_change_handle_ = param_subscriber_->add_parameter_callback(
                remote_param_name, base_change_handle, remote_node_name);
            }

    void Battery::createServices(){
            // Action server to handle battery charging requests

            // Service to check energy
            this->energy_check_service_ = this->create_service<BatteryCheck>(
                "check_energy",
                // Lambda function to handle energy check requests
                [this](const std::shared_ptr<BatteryCheck::Request> request,
                       const std::shared_ptr<BatteryCheck::Response> response) {
                    response->success = this->checkBattery(request->energy_required);
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Battery has enough energy: %f Wh", 
                            request->energy_required);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Battery does not have enough energy: %f Wh", 
                            request->energy_required);
                    }
                }
            );

            // Service to handle energy requests    
            this->energy_request_service_ = this->create_service<BatteryCheck>(
                "request_energy",
                // Lambda function to handle energy request
                [this](const std::shared_ptr<BatteryCheck::Request> request,
                       const std::shared_ptr<BatteryCheck::Response> response) {
                    if (this->consumeEnergy(request->energy_required)) {
                        response->success = true;
                        RCLCPP_INFO(this->get_logger(), 
                            "Energy consumed: %f Wh", request->energy_required);
                    } else {
                        response->success = false;
                        RCLCPP_WARN(this->get_logger(), "Not enough energy to consume: %f Wh", 
                            request->energy_required);
                    }
                }
            );
        }
        void Battery::createTransformListener(){
            // Create a transform listener to get the robot position

            // Robot frame follower
            tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        }

        void Battery::createActionServer(){

            // Action server to handle battery charging requests
            auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const BatteryCharge::Goal> goal) {
                // Check if the energy required is valid
                if (goal->energy_required < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid energy required: %f", goal->energy_required);
                    return rclcpp_action::GoalResponse::REJECT;
                }
                RCLCPP_INFO(this->get_logger(), "Received battery charge request");
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };
            auto handle_cancel = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<BatteryCharge>> goal_handle) {
                // Cancel the charging request
                auto result = std::make_shared<BatteryCharge::Result>();
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Charging battery cancelled");
                return rclcpp_action::CancelResponse::ACCEPT;
            };  
            auto handle_accepted = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<BatteryCharge>> goal_handle) {
                
                auto execute_in_thread = [this, goal_handle](){
                    return this->chargeBattery(goal_handle);};
                std::thread{execute_in_thread}.detach();
            };

            // Create the action server
            this->action_server_ = rclcpp_action::create_server<BatteryCharge>(
                this,
                "charge_battery",
                handle_goal,
                handle_cancel,
                handle_accepted
            );

        }

        bool Battery::checkBattery(double energy) const {
            return this->energy_ >= energy;
        }

        int Battery::getBatteryPercentage() const{
            return 100*this->energy_ /this->capacity_ ;
        }

        bool Battery::consumeEnergy(double energy) {
            this->energy_  = this->energy_ - energy;
            assert(this->energy_  >= 0 && "Battery level cannot be negative");
            return true;
        }

        void Battery::chargeBattery(std::shared_ptr<BatteryChargeGoalHandle> goal_handle){
            
            double energy_required = goal_handle->get_goal()->energy_required;
            energy_required = std::min(
                energy_required,
                this->capacity_ 
            );

            if(energy_required == 0)
                energy_required = this->capacity_;
            
            auto result = std::make_shared<BatteryCharge::Result>();
            auto feedback = std::make_shared<BatteryCharge::Feedback>();
            rclcpp::Rate rate(0.5); 

            // Check if the battery is on the charging base
            //  Look up for the transformation between global frame and robot frame
            geometry_msgs::msg::TransformStamped frame_robot;

            try{
                while (this->energy_ < energy_required) {
                    try{
                        // Name Transform since this node is a component using the container name
                        // as the robot name
                        
                        frame_robot = this->tf_buffer_->lookupTransform(
                            "world", this->get_name(), tf2::TimePointZero);
                    } catch (const tf2::TransformException & ex) {
                        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
                        rate.sleep();
                        continue;
                    }

                    // Check if the robot is on the charging base
                    double distanceBase = std::sqrt(
                        std::pow(frame_robot.transform.translation.x - this->posBaseGlobal_[0], 2) +
                        std::pow(frame_robot.transform.translation.y - this->posBaseGlobal_[1], 2)
                    );
                    if (distanceBase > this->tollerance_) {
                        RCLCPP_WARN(this->get_logger(), "Robot is not on the charging base");
                        throw std::runtime_error("Robot is not on the charging base");
                        continue;
                    }
                    
                    //charge
                    this->energy_ = std::min(this->capacity_,this->energy_+this->charge_rate_ / 3600.0); 
                    feedback->battery_percentage = this->getBatteryPercentage();
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Charging... %d%%", feedback->battery_percentage);
                    rate.sleep();
                }
            }
            catch (const std::runtime_error & e) {
                // Cancel Charging
                RCLCPP_ERROR(this->get_logger(), "Error during charging: %s", e.what());
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            
            result->success = true;
            goal_handle->succeed(result);
        }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_simple::Battery)

