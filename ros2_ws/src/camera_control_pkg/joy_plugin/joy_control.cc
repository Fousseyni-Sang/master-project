#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <ignition/math.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#define _USE_MATH_DEFINES
#include <cmath>
#include <functional>


namespace gazebo_plugins
{
    class JoySubscriberPlugin : public gazebo::ModelPlugin
    {
    public:
        JoySubscriberPlugin() {}

        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
        {
            try{
                

            // Get pose

                this->model = _model;
                auto pose = model->WorldPose();
                this->left_x = pose.Pos().X();
                this->left_y = pose.Pos().Y();
                this->angle_x = 0;
                this->angle_y = 0;
                this->angle_z = 0;
                this->deadzone_joy = 0.1;
                this->deadzone_key = 0.001;

                // Initialize ROS node
                //rclcpp::init(0, nullptr);
                // Create a ROS node
                this->node = rclcpp::Node::make_shared("joy_subscriber_node");
                
                // Subscribe to the /joy topic
                this->joy_subscriber = this->node->create_subscription<sensor_msgs::msg::Joy>(
                    "/joy", 10, std::bind(&JoySubscriberPlugin::OnJoyMessage, this, std::placeholders::_1));

                // Subscribe to the /key topic
                this->key_subscriber = this->node->create_subscription<std_msgs::msg::Float32MultiArray>(
                    "/key", 10, std::bind(&JoySubscriberPlugin::OnKeyMessage, this, std::placeholders::_1));
       
                // Start the ROS spin thread
                this->spin_thread = std::thread([this]() {
                    rclcpp::spin(this->node);
                });
               
                lastJoyMessage = nullptr;
                lastKeyMessage = nullptr;
                this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&JoySubscriberPlugin::OnUpdate, this));
                
            }

            catch (const std::exception& ex)
            {
                // Print the exception message to the terminal
                std::cerr << "Exception occurred during plugin initialization: " << ex.what() << std::endl;
            }
        }

        void OnJoyMessage(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            
            lastJoyMessage = msg;
            if (lastJoyMessage->axes[5]==1) lastJoyMessage->axes[5]=0;
            if (lastJoyMessage->axes[2]==1) lastJoyMessage->axes[2]=0;
        }

        void OnKeyMessage(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {   
            lastKeyMessage = msg;
        }

        void OnUpdate(){

            UseJoyStick = false;
            UseKeyBoard = false;

            pose = this->model->WorldPose();
            pose.Pos().Z() = 0.15; 

            // Handle joystick movement
            if (lastKeyMessage){

                // Get current pose
                /* pose = this->model->WorldPose();
                pose.Pos().Z() = 0.15; */ 

                if (abs(lastKeyMessage->data[3]) > deadzone_key){
                    angle_x += lastKeyMessage->data[3] * M_PI*0.001;
                    UseKeyBoard = true;
                }

                if (abs(lastKeyMessage->data[4]) > deadzone_key) {
                    angle_y += lastKeyMessage->data[4] * M_PI*0.001;
                    UseKeyBoard = true;
                }                

                if (abs(lastKeyMessage->data[5]) > deadzone_key){
                    angle_z += -lastKeyMessage->data[5] * M_PI*0.001;
                    UseKeyBoard = true;
                }
                

                //RCLCPP_INFO(node->get_logger(), "Rot Diff: %f %f %f", lastJoyMessage->axes[3], lastJoyMessage->axes[4], lastJoyMessage->axes[5]);
                //RCLCPP_INFO(node->get_logger(), "KeyRotation: %f %f %f", angle_x, angle_y, angle_z);

                if ((abs(lastKeyMessage->data[0]) > deadzone_key) || (abs(lastKeyMessage->data[1]) > deadzone_key)){
                    RCLCPP_INFO(node->get_logger(), "KeyTrans Diff: %f %f", abs(lastKeyMessage->data[0]), abs(lastKeyMessage->data[1]));
                    left_x = lastKeyMessage->data[0] * 0.001;
                    left_y = lastKeyMessage->data[1] * 0.001;
                    UseKeyBoard = true;
                }
                else{
                    left_x = 0;
                    left_y = 0;
                }
                


                // Handle Keyboard movement
                if ((lastJoyMessage) && (!UseKeyBoard)){

                    // Get current pose
                    /* pose = this->model->WorldPose();
                    pose.Pos().Z() = 0.15;  */

                    if (abs(lastJoyMessage->axes[4]) > deadzone_joy){
                        angle_x += lastJoyMessage->axes[4] * M_PI*0.001;
                        UseJoyStick = true;
                    }

                    if (abs(lastJoyMessage->axes[5]) > deadzone_joy) {
                        angle_y += abs(lastJoyMessage->axes[5]) * M_PI*0.001;
                        UseJoyStick = true;
                    }

                    if (abs(lastJoyMessage->axes[2]) > deadzone_joy) {
                        angle_y -= abs(lastJoyMessage->axes[2]) * M_PI*0.001;
                        UseJoyStick = true;
                    }

                    if (abs(lastJoyMessage->axes[3]) > deadzone_joy){
                        angle_z += lastJoyMessage->axes[3] * M_PI*0.001;
                        UseJoyStick = true;
                    }

                    //RCLCPP_INFO(node->get_logger(), "Rot Diff: %f %f %f", lastJoyMessage->axes[3], lastJoyMessage->axes[4], lastJoyMessage->axes[5]);
                    //RCLCPP_INFO(node->get_logger(), "JoyRotation: %f %f %f", angle_x, angle_y, angle_z);

                    if ((abs(lastJoyMessage->axes[0]) > deadzone_joy) || (abs(lastJoyMessage->axes[1]) > deadzone_joy)){
                        RCLCPP_INFO(node->get_logger(), "JoyTrans Diff: %f %f", abs(lastJoyMessage->axes[0]), abs(lastJoyMessage->axes[1]));
                        left_x = -lastJoyMessage->axes[0] * 0.001;
                        left_y = lastJoyMessage->axes[1] * 0.001;
                        UseJoyStick = true;
                    }
                    else{
                        left_x = 0;
                        left_y = 0;
                        }
                } 

                //RCLCPP_INFO(node->get_logger(), "JoyTranslation: %f %f", left_x, left_y);

                //ignition::math::Vector3d translation(left_x, left_y, 0);
                if (UseJoyStick || UseKeyBoard){
                    ignition::math::Quaterniond rotation(1, 0, 0, 0);

                    RCLCPP_INFO(node->get_logger(), "Angles: %f %f %f", angle_x, angle_y, angle_z);

                    rotation = EulerToQuaternion(angle_y, angle_x, angle_z);
                    // Apply rotations to the current orientation
                    pose.Rot() = rotation;// * rotationQuaternionY;
                    // Set the new position
                    pose.Pos() += ignition::math::Vector3d(left_x, left_y, 0);
                    
                    RCLCPP_INFO(node->get_logger(), "Rotation Quaternion: %f %f %f %f", pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());

                    this->model->SetWorldPose(pose); 
                }
            }
            
        }

        ignition::math::Quaterniond EulerToQuaternion(double roll, double pitch, double yaw){ // roll (x), pitch (y), yaw (z), angles are in radians
            
            // Abbreviations for the various angular functions

            double cr = cos(roll * 0.5);
            double sr = sin(roll * 0.5);
            double cp = cos(pitch * 0.5);
            double sp = sin(pitch * 0.5);
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);

            ignition::math::Quaterniond q(1, 0, 0, 0);
            q.W() = cr * cp * cy + sr * sp * sy;
            q.X() = sr * cp * cy - cr * sp * sy;
            q.Y() = cr * sp * cy + sr * cp * sy;
            q.Z() = cr * cp * sy - sr * sp * cy;

            return q;
        }

    private:
        gazebo::physics::ModelPtr model;
        ignition::math::Pose3d pose;
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr key_subscriber;
        std::thread spin_thread;
        sensor_msgs::msg::Joy::SharedPtr lastJoyMessage;
        std_msgs::msg::Float32MultiArray::SharedPtr lastKeyMessage;
        // Pointer to the update event connection
        gazebo::event::ConnectionPtr updateConnection;
        double left_x;
        double left_y;

        double deadzone_key;
        double deadzone_joy;
        

        double angle_x;
        double angle_y;
        double angle_z;

        bool UseJoyStick;
        bool UseKeyBoard;

        bool doRotation = false;

        
    };

    GZ_REGISTER_MODEL_PLUGIN(JoySubscriberPlugin)
} // namespace gazebo_plugins

