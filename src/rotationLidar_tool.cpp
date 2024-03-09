#include <chrono>
#include <memory>
 #include <rclcpp/rclcpp.hpp>  
#include <std_msgs/msg/float64.hpp>  
#include <tf2_ros/transform_broadcaster.h>  
#include <tf2/LinearMath/Quaternion.h>  
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>  
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
 using std::placeholders::_1;

class ExamplePublisher : public rclcpp::Node {
 
public:
  ExamplePublisher()
      : Node("example_publisher") {
    angle_ = 0.0;
    omega_ = 6.28 * 0.01 / 20; // 10s一圈
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/model/x500_mono_cam_0/servo_lidar", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/servo_lidar_states", 10, std::bind(&ExamplePublisher::jointStateCallback, this, _1));
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    //休眠10s后在开始工作
    std::this_thread::sleep_for(std::chrono::seconds(10));  
    RCLCPP_INFO(this->get_logger(), "10 seconds have passed!");
    timer_ = this->create_wall_timer(1ms, std::bind(&ExamplePublisher::timer_callback, this));
  }
 
private:
    void jointStateCallback(const sensor_msgs::msg::JointState & msg) {
        RCLCPP_INFO(this->get_logger(), "subscriber heard: %f", cur_angle_);
        cur_angle_ = msg.position[0];
    }

    void timer_callback() {
        auto message = std_msgs::msg::Float64();
        message.data = angle_;
        angle_ += omega_;
        angle_publisher_->publish(message);
        //降低tf变化发布频率
        static int cnt = 0;
        if(cnt<5) {
            cnt++;
            return;
        }
        cnt = 0;
        //pub tf transform
        geometry_msgs::msg::TransformStamped t;
        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "x500_mono_cam_0/my_lidar/base_link/gpu_lidar";
        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = 0;
        t.transform.translation.y = 0;
        t.transform.translation.z = 0.5;
        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        tf2::Quaternion q;
        q.setRPY(0, cur_angle_, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    double angle_, omega_, cur_angle_;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExamplePublisher>());
  rclcpp::shutdown();
  return 0;
}