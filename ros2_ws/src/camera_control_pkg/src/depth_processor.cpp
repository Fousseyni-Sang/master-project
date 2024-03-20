#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <SFML/Audio.hpp>
#include <iostream>
#include <chrono>
#include <thread>

class DepthProcessor : public rclcpp::Node {
public:
    DepthProcessor() : Node("depth_processor") {
        depth_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&DepthProcessor::depthCallback, this, std::placeholders::_1));
        
        camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/depth/camera_info", 10, std::bind(&DepthProcessor::cameraInfoCallback, this, std::placeholders::_1));

        // Initialize obstacle detection threshold (adjust this based on your requirements)
        obstacle_threshold_ = 0.4;  // in meters

        // Load the sound from the .wav file
        if (!buffer_.loadFromFile("/home/fousseyni/SMR_Project/ros2_ws/src/audio_beep/beep-02.wav")) {
            RCLCPP_ERROR(get_logger(), "Failed to load sound file");
        }
        sound.setBuffer(buffer_);

        soundPlaying = false;
        cooldownDuration = std::chrono::milliseconds(3000);

    }

private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            //Convert ROS message to OpenCV image
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Create a mask for pixels that are above the threshold and not equal to zero
        cv::Mat mask = ( cv_ptr->image <= obstacle_threshold_) ; //| (depth_image != 128.0);
        //print_Mat(cv_ptr->image);
        //print_MinMax(cv_ptr->image);
        // Count the number of obstacle pixels
        
        int num_obstacle_pixels = cv::countNonZero(mask);
        int min_pixel_obstacle = 0.15*cv_ptr->image.cols*cv_ptr->image.rows; // We consider there is obstacle when at 15% of pixel are below the threshold
        if (num_obstacle_pixels > min_pixel_obstacle) {
            RCLCPP_INFO(get_logger(), "Obstacle detected! Number of obstacle pixels: %d", num_obstacle_pixels);
            if (!soundPlaying && cooldownElapsed()) {
                bipSound();
                lastPlaybackTime = std::chrono::steady_clock::now();
                soundPlaying = false;
            }
        } 
        else{
            RCLCPP_INFO(get_logger(), "No Obstacle detected! Number of obstacle pixels: %d", num_obstacle_pixels);
        }  
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        // Retrieve calibration information from CameraInfo message
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
    }

    void bipSound(){
        sound.play();
        soundPlaying = true;
        std::cout << "bipSound produced" << std::endl;
    }

    bool cooldownElapsed() {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastPlaybackTime);
        return elapsedTime >= cooldownDuration;
    }

    void print_Mat(cv::Mat image){
        

        // Iterate over the pixels and print their values
        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                float pixel_value = image.at<float>(y, x);
                RCLCPP_INFO(get_logger(), "Pixel value at (%d, %d): %f", x, y, pixel_value);
                //std::cout << "Pixel value at (" << x << ", " << y << "): " << pixel_value << std::endl;
            }
        }
    
    }

    void print_MinMax(cv::Mat image){
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;

    // Calculate the minimum and maximum values and their locations
    cv::minMaxLoc(image, &minVal, &maxVal, &minLoc, &maxLoc);

    // Print the minimum and maximum pixel values
    RCLCPP_INFO(get_logger(), "Minimum pixel value: %f, Maximum pixel value: %f", minVal, maxVal);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;

    double fx_;
    double fy_;
    double cx_;
    double cy_;

    double obstacle_threshold_;
    float minDist = 0.05f; // Camera min range
    float maxDist = 3.0f; // Camera max range

    sf::SoundBuffer buffer_;

    bool soundPlaying;
    std::chrono::steady_clock::time_point lastPlaybackTime;
    std::chrono::milliseconds cooldownDuration;
    sf::Sound sound;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthProcessor>());
    rclcpp::shutdown();
    return 0;
}
