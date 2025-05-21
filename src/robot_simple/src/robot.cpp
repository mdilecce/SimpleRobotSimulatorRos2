#include <functional>
#include <memory>
#include <thread>

#include "robot_simple/robot.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>

namespace robot_simple{

    Robot::Robot(
        const std::string &name,
        const std::string &battery,
        const rclcpp::NodeOptions & options
    ) : Node(name, options),
        battery_name_(battery){

        RCLCPP_INFO(this->get_logger(), "Robot constructor start");

        this->frame_robot_ = tf2::Transform();
        this->frame_robot_.setOrigin(tf2::Vector3(0, 0, 0));
        auto q = tf2::Quaternion(0,0,0,1);
        this->frame_robot_.setRotation(q);
        this->posBaseGlobal_ = tf2::Vector3({0,0,0});
        this->speed_ = 0;

        this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        this->updateTransform();

        RCLCPP_INFO(this->get_logger(), "Initializing parameters...");
        this->initializeParameters();
        RCLCPP_INFO(this->get_logger(), "Initializing battery services...");
        this->initializeBatteryServices();
        RCLCPP_INFO(this->get_logger(), "Initializing action server...");
        this->initializeActionServer();
        RCLCPP_INFO(this->get_logger(), "Initializing publisher...");
        this->initializePublisher();

        RCLCPP_INFO(this->get_logger(), "Robot constructor end");
    }

    Robot::Robot(const rclcpp::NodeOptions & options):
        Robot("robot", "battery", options) {};



    void Robot::initializeParameters(){

        this->param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Initialize parameters with default values and add callbacks
        std::vector<double> posBaseGlobal = {posBaseGlobal_[0], posBaseGlobal_[1]};
        this->declare_parameter("position_base_global", posBaseGlobal);
        this->position_base_handle_ = this->param_subscriber_->add_parameter_callback(
            "position_base_global", [this](const rclcpp::Parameter & param) {
                auto newBasePos = param.as_double_array();
                this->posBaseGlobal_ = tf2::Vector3(newBasePos[0], newBasePos[1], 0);
                RCLCPP_INFO(this->get_logger(), "Base position updated: [%f, %f]", 
                    this->posBaseGlobal_[0], this->posBaseGlobal_[1]);
            }
        );

        this->declare_parameter("acceleration", this->acceleration_);
        this -> acceleration_handle_ = this->param_subscriber_->add_parameter_callback(
            "acceleration", [this](const rclcpp::Parameter & param) {
                this->acceleration_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Acceleration updated: %f m/s^2", 
                    this->acceleration_);
            }
        );

        this->declare_parameter("deceleration", this->deceleration_);

        this -> deceleration_handle_ = this->param_subscriber_->add_parameter_callback(
            "deceleration", [this](const rclcpp::Parameter & param) {
                this->deceleration_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Deceleration updated: %f m/s^2", 
                    this->deceleration_);
            }
        );
        this->declare_parameter("speedMax", this->speedMax_);
        this -> speedMax_handle_ = this->param_subscriber_->add_parameter_callback(
            "speedMax", [this](const rclcpp::Parameter & param) {
                this->speedMax_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Max speed updated: %f  m/s", 
                    this->speedMax_);
            }
        );
        this->declare_parameter("mass", this->mass_);
        this -> mass_handle_ = this->param_subscriber_->add_parameter_callback(
            "mass", [this](const rclcpp::Parameter & param) {
                this->mass_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Mass updated: %f kg", 
                    this->mass_);
            }
        );
        this->declare_parameter("fresistive", this->fresistive_);   
        this -> fresistive_handle_ = this->param_subscriber_->add_parameter_callback(
            "fresistive", [this](const rclcpp::Parameter & param) {
                this->fresistive_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Fresistive updated: %f N", 
                    this->fresistive_);
            }
        );
        this->declare_parameter("stepTime", this->stepTime_);
        this -> stepTime_handle_ = this->param_subscriber_->add_parameter_callback(
            "stepTime", [this](const rclcpp::Parameter & param) {
                this->stepTime_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Step time updated: %f s", 
                    this->stepTime_);
            }
        );
        this->declare_parameter("tollerance", this->tollerance_);
        this -> tollerance_handle_ = this->param_subscriber_->add_parameter_callback(
            "tollerance", [this](const rclcpp::Parameter & param) {
                this->tollerance_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Tollerance updated: %f m", 
                    this->tollerance_);
            }
        );

    }

    void Robot::initializePublisher(){
        // Create a publisher to publish the robot status
        this->status_pub_ = this->create_publisher<RobotStatus>(
            "robot_status", rclcpp::QoS(rclcpp::KeepLast(10)));
        
        // Lambda function to publish robot status
        auto publish_robot_status = [this]() {
            RobotStatus status_msg;
            status_msg.x = this->frame_robot_.getOrigin().x();
            status_msg.y = this->frame_robot_.getOrigin().y();
            tf2::Vector3 speed = this->frame_robot_*tf2::Vector3(this->speed_,0,0);
            status_msg.speed_x = speed.x();
            status_msg.speed_y = speed.y();
            tf2::Matrix3x3 m(this->frame_robot_.getRotation());
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            status_msg.yaw = yaw;
            this->status_pub_->publish(status_msg);
            RCLCPP_INFO(this->get_logger(), "Robot status: Position (%f, %f) m, Speed (%f, %f) m/s, Yaw %f rad",
                status_msg.x, status_msg.y, status_msg.speed_x, status_msg.speed_y, status_msg.yaw);
        };
        
        // Timer to periodically publish robot status
        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            publish_robot_status
        );
    }

    void Robot::initializeBatteryServices(){

        // Create a service to check the energy status
        this->energy_check_client_ = this->create_client<BatteryCheck>(
            "check_energy"); // changed from "energy_check" to "check_energy"
        
        // Create a service to request energy
        this->energy_request_client_ = this->create_client<BatteryCheck>(
            "request_energy"); // changed from "energy_request" to "request_energy"

        this->action_battery_client_ = rclcpp_action::create_client<BatteryCharge>(
            this,
            "charge_battery"
        );
    }

    bool Robot::checkBattery(double energy) const{
        // Check if the battery has enough energy
        auto energy_request = std::make_shared<BatteryCheck::Request>();
        energy_request->energy_required = energy;
        
        auto response = this->energy_check_client_->async_send_request(energy_request);
        if (response.get()->success) {
            //RCLCPP_INFO(this->get_logger(), "Battery has enough energy");
            return true;
        } else {
            //RCLCPP_WARN(this->get_logger(), "Battery does not have enough energy");
            return false;
        }
    }

    bool Robot::consumeEnergy(double energy){
        // Consume energy from the battery
        auto energy_request = std::make_shared<BatteryCheck::Request>();
        energy_request->energy_required = energy;
        
        auto response = this->energy_request_client_->async_send_request(energy_request);
        if (response.get()->success) {
            RCLCPP_INFO(this->get_logger(), "%f W consumed", energy);
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to consume energy");
            return false;
        }
    }

    void Robot::chargeBattery(double energy_required){

        //Check if the action server is available
        if(!this->action_battery_client_->wait_for_action_server(std::chrono::seconds(1))){
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        // Charge the battery
        auto energy_request = BatteryCharge::Goal();
        energy_request.energy_required = energy_required;
        
        RCLCPP_INFO(this->get_logger(), "Sending Charging request");

        auto send_goal_options = rclcpp_action::Client<BatteryCharge>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = [this](const BatteryChargeGoalHandle::SharedPtr & goal_handle){
            if (!goal_handle) 
                RCLCPP_ERROR(this->get_logger(), "Charging request rejected");
            else 
                RCLCPP_INFO(this->get_logger(), "Charging request accepted");
        };

        send_goal_options.feedback_callback = [this](
            BatteryChargeGoalHandle::SharedPtr,
            const std::shared_ptr<const BatteryCharge::Feedback> feedback) {
            RCLCPP_INFO(this->get_logger(), "Battery Charging .. Battery  %d %% charged", 
                feedback->battery_percentage);};

        send_goal_options.result_callback = [this](const BatteryChargeGoalHandle::WrappedResult & result){
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Charging completed");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Charging failed");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Charging cancelled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }
        };

        // Send the goal to the action server   
        this->action_battery_client_->async_send_goal(
            energy_request, send_goal_options
        );

    }


    void Robot::initializeActionServer(){
        // Create an action server to handle move to target requests

        auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MoveToTarget::Goal> goal) {
            RCLCPP_INFO(this->get_logger(), "Received move to target (%f,%f) request",
                goal->target_position_x, goal->target_position_y);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };
        auto handle_cancel = [this](const std::shared_ptr<MoveToTargetGoalHandle> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Move to target (%f,%f) m cancelled",
                goal_handle->get_goal()->target_position_x, goal_handle->get_goal()->target_position_y);
            return rclcpp_action::CancelResponse::ACCEPT;
        };
        auto handle_accepted = [this](const std::shared_ptr<MoveToTargetGoalHandle> goal_handle) {
            auto execute_in_thread = [this, goal_handle](){return this->moveToTarget(goal_handle);};
            std::thread{execute_in_thread}.detach();
  
        };
        // Create the action server
        this -> action_server_ = rclcpp_action::create_server<MoveToTarget>(
            this,
            "move_to_target",
            handle_goal,
            handle_cancel,
            handle_accepted
        );
    }

    double Robot::calculateDistance(const tf2::Vector3 &target) const{
        return this->frame_robot_.getOrigin().distance(target);
    }

    double Robot::calculateDistanceToBase(const tf2::Vector3 &target)  const{
        return this->posBaseGlobal_.distance(target);
    }           

    double Robot::calculateEnergy(double distance) const {
        double energy = 0;

        // Peak Velocity
        double v_max = std::min(
            std::sqrt((2*distance)/(1/this->acceleration_ + 1/this->deceleration_)),
            this->speedMax_
        );    

        // Kinematic Energy to accelerate
        energy += 0.5*this->mass_*v_max*v_max;

        // Energy required for for Resistence forces
        energy += this->fresistive_ * distance;

        return energy;
    }

    void Robot::updateTransform(){
        // Update the robot's position
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = this->get_name();
        t.transform.translation.x = this->frame_robot_.getOrigin().x();
        t.transform.translation.y = this->frame_robot_.getOrigin().y();
        t.transform.translation.z = this->frame_robot_.getOrigin().z();
        t.transform.rotation.x = this->frame_robot_.getRotation().x();
        t.transform.rotation.y = this->frame_robot_.getRotation().y();  
        t.transform.rotation.z = this->frame_robot_.getRotation().z();
        t.transform.rotation.w = this->frame_robot_.getRotation().w();
        // Send the transform to the tf2 broadcaster 
        this->tf_broadcaster_->sendTransform(t);
    }

    void Robot::rotateRobot(const tf2::Vector3& target){

        tf2::Vector3 towards = target - this->frame_robot_.getOrigin();
        tf2::Vector3 heading = this->frame_robot_.getBasis().getColumn(0);

        double angle = std::atan2(towards.y(), towards.x()) - std::atan2(heading.y(), heading.x());

        // Normalize angle to [-pi, pi]
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;

        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        this->frame_robot_.setRotation(this->frame_robot_.getRotation() * q);

        this->updateTransform();
        RCLCPP_INFO(this->get_logger(), "Robot rotated: %f deg", 
            angle * 180 / M_PI);
    }

    void Robot::moveRobot(std::shared_ptr<MoveToTargetGoalHandle> goal_handle, bool returnToBase){

        tf2::Vector3 target;
        if(!returnToBase){
            target = tf2::Vector3(
                goal_handle->get_goal()->target_position_x, 
                goal_handle->get_goal()->target_position_y, 
                0
            );
        }else{
            target = this->posBaseGlobal_;
        }

        auto distance = this->calculateDistance(target);
        
        auto feedback = std::make_shared<MoveToTarget::Feedback>();

        double time = 0, velocity = 0,  position=0, energyTotal = 0;
        
        // Determine if the max velocity is reached on the path to target
        double v_max = std::min(
            std::sqrt((2*distance)/(1/this->acceleration_ + 1/this->deceleration_)),
            this->speedMax_
        );    

       // Acceleration and deceleration distances
        double x_a = 0.5*std::pow(v_max,2)/this->acceleration_;
        double x_d = 0.5*std::pow(v_max,2)/this->deceleration_;

        // Kinematic Energy to accelerate
        double energyAccelerationStep = 0.5*this->mass_*v_max*this->acceleration_;
  
        try{
            //Make profile
            while (position < distance) {

                double energy = 0;
                
                // Accelerate in the first half
                if (position < x_a) {
                    velocity += this->acceleration_ * this->stepTime_;
                    // Energy required for acceleration
                    energy += energyAccelerationStep* this->stepTime_;
                }
                //Decelerate in the second half
                else if (position > (distance - x_d)) {
                    velocity -= this->deceleration_ * this->stepTime_;
                    if(velocity < 0) 
                        throw std::runtime_error("Failed to move: negative velocity");           
                }
                
                // Movement in current local frame
                double deltaPositionLocal = velocity * this->stepTime_;

                // Energy required for for Resistence forces
                energy += this->fresistive_ * deltaPositionLocal;

                //Ttime update
                time += this->stepTime_;

                //Update position local
                position += deltaPositionLocal;

                // Update Global Position
                // Transform the local position to global coordinates
                tf2::Vector3 positionGlobal = this->frame_robot_*tf2::Vector3(deltaPositionLocal, 0, 0);

                // Update Robot Position
                this->frame_robot_.setOrigin(positionGlobal);
                this->updateTransform();

                // Update Odometer
                this->odometer_ += std::abs(deltaPositionLocal);

                // Update feedback
                feedback->distance_to_target = distance - position;
                feedback->speed = velocity;
                
                goal_handle->publish_feedback(feedback);

                // Update battery
                this->consumeEnergy(energy);
                energyTotal += energy;
                
                // Print status
                RCLCPP_INFO(this->get_logger(), "Robot Distance to target: %f m, Speed: %f m/s, Odometer: %f m", 
                    feedback->distance_to_target, feedback->speed, this->odometer_);

                std::this_thread::sleep_for(std::chrono::duration<double>(this->stepTime_));
            }

            // Removed unused lambda that referenced local variables and 'this' without capturing.
        }
        catch (const std::runtime_error& e) {
            // Fixing last decimals for position
            velocity = 0;
            double positionFix = distance - position;
            this->odometer_ += abs(positionFix);
            // Update Global Position
            tf2::Vector3 positionFixGlobal = this->frame_robot_*tf2::Vector3(positionFix, 0, 0);
            // Update Robot Position
            this->frame_robot_.setOrigin(positionFixGlobal);
            this->updateTransform();
        }
        
        // Update feedback
        feedback->distance_to_target = distance - position;
        feedback->speed = velocity;
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Robot Distance to target: %f m, Speed: %f m/s, Odometer: %f m", 
            feedback->distance_to_target, feedback->speed, this->odometer_);

        RCLCPP_INFO(this->get_logger(), "Total Energy consumed: %f Wh", energyTotal);

        
    }


    void Robot::moveToTarget(std::shared_ptr<MoveToTargetGoalHandle> goal_handle){
        
        auto feedback = std::make_shared<MoveToTarget::Feedback>();
        auto result = std::make_shared<MoveToTarget::Result>();

        tf2::Vector3 target(
            goal_handle->get_goal()->target_position_x, 
            goal_handle->get_goal()->target_position_y, 
            0
        );
        
        if(target == this->frame_robot_.getOrigin()){
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Robot already at target");
            return;
        }
        auto distance = this->calculateDistance(target);
        auto distanceBase = this->calculateDistanceToBase(target);

        auto energy = this->calculateEnergy(distance) +
            this->calculateEnergy(distanceBase);
        RCLCPP_INFO(this->get_logger(), "Energy required: %f Ah", energy);

        if (this->checkBattery(energy)) {

            //Rotate to the target
            this->rotateRobot(target);
            // Move to the target
            this->moveRobot(goal_handle);
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Robot reached target");
        } else {

            RCLCPP_ERROR(this->get_logger(), "Battery does not have enough energy, returning to base for charging");
            // Return to base
            RCLCPP_INFO(this->get_logger(), "Returning to base...");
            // Rotate to the base
            this->rotateRobot(this->posBaseGlobal_);
            // Move to the base
            this->moveRobot(goal_handle, true);
            this->chargeBattery();    
            RCLCPP_INFO(this->get_logger(), "Battery charged, returning to target");

            distance = this->calculateDistance(target);
            energy = this->calculateEnergy(2*distance);

            if(!this->checkBattery(energy)){
                RCLCPP_ERROR(this->get_logger(), "Not enough energy to move to target");
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            // Rotate to the target
            this->rotateRobot(target);
            // Move to the target
            this->moveRobot(goal_handle);
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Robot reached target");
        }

    }


}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_simple::Robot)

