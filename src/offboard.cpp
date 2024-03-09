#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "simple_traj.h"


class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("OffboardPx4CPP") {
        initParam();
        state_sub = this->create_subscription<mavros_msgs::msg::State>("mavros/state", 10, std::bind(&OffboardControl::state_callback, this, std::placeholders::_1));
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/setTarget/uavPose", 10, std::bind(&OffboardControl::target_callback, this, std::placeholders::_1));
        //要设置QOS符合mavros发布才能接受话题
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", rclcpp::SensorDataQoS(), std::bind(&OffboardControl::pose_callback, this, std::placeholders::_1));
        local_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        PlannerState_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/plannerState", 10);
        set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        
        arm_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        
        while (!set_mode_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "set_mode service not available, waiting...");
        }
        
        while (!arm_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "arming service not available, waiting...");
        }

        timer_period = std::chrono::milliseconds(20);
        timer = this->create_wall_timer(timer_period, std::bind(&OffboardControl::timer_callback, this));
        state_timer = this->create_wall_timer(std::chrono::seconds(1),
                                              std::bind(&OffboardControl::state_timer_callback, this));
    }

private:
    void initParam() {
        target_pose_.header.frame_id = "world";
        target_pose_.pose.position.x = 0;
        target_pose_.pose.position.y = 0;
        target_pose_.pose.position.z = 2;
        target_pose_.pose.orientation.w = 1;
        target_pose_.pose.orientation.x = 0;
        target_pose_.pose.orientation.y = 0;
        target_pose_.pose.orientation.z = 0;
        simple_traj_.setParams(0.5, 0.5);
        planner_state = WAIT_TARGET;
        start_time_ = -1;
    }

    void timer_callback() {
        if(planner_state!=WAIT_TARGET) {
            double curr_time = rclcpp::Clock().now().seconds();
            double whole_dt = curr_time - start_time_;
            std::vector<Eigen::Vector3d> pva(3, Eigen::Vector3d());
            if(simple_traj_.getPVA(pva, whole_dt)) {
                RCLCPP_INFO(this->get_logger(), "Planner finished!");
                planner_state = WAIT_TARGET;
                return;
            }
            else {
                target_pose_.pose.position.x = pva[0](0);
                target_pose_.pose.position.y = pva[0](1);
                target_pose_.pose.position.z = pva[0](2);
                target_pose_.pose.orientation.w = 1;
                target_pose_.pose.orientation.x = 0;
                target_pose_.pose.orientation.y = 0;
                target_pose_.pose.orientation.z = 0;
            }
        }
        //不发布不能切offboard
        local_pose_pub->publish(target_pose_);
    }

    void state_timer_callback() {
        if (cur_state.mode != "OFFBOARD") {
            RCLCPP_INFO(this->get_logger(), "offboard");
            set_mode("OFFBOARD");
        } else if (!cur_state.armed) {
            RCLCPP_INFO(this->get_logger(), "armed");
            arm(true);
        }
        auto message = std_msgs::msg::UInt8();  
        message.data = static_cast<uint8_t>(planner_state);
        PlannerState_pub_->publish(message);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cur_pose_ = *msg;
    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        cur_state = *msg;
    }
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::vector<Eigen::Vector3d> way_points(2);
        way_points[0](0) = cur_pose_.pose.position.x;
        way_points[0](1) = cur_pose_.pose.position.y;
        way_points[0](2) = cur_pose_.pose.position.z;

        way_points[1](0) = msg->pose.position.x;
        way_points[1](1) = msg->pose.position.y;
        way_points[1](2) = msg->pose.position.z;
        simple_traj_.setWayPoints(way_points);
        RCLCPP_INFO(this->get_logger(), "Get targets!");
        planner_state = WAIT_FLYING;
        start_time_ = rclcpp::Clock().now().seconds();
    }
    void set_mode(const std::string& mode) {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = mode;
        using ServiceResponseFuture =
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
          RCLCPP_INFO(this->get_logger(), "Got result: [%d]", future.get()->mode_sent);
        };

        auto future_result = set_mode_client->async_send_request(req, response_received_callback);

    }

    void arm(bool arm) {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = arm;
        using ServiceResponseFuture =
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
          RCLCPP_INFO(this->get_logger(), "Got result: [%d]", future.get()->success);
        };
        auto future_result = arm_client->async_send_request(req, response_received_callback);
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_, pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr PlannerState_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client;
    SimpleTraj simple_traj_;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr state_timer;
    geometry_msgs::msg::PoseStamped target_pose_, cur_pose_;
    mavros_msgs::msg::State cur_state;
    std::chrono::milliseconds timer_period;

    enum State{
        WAIT_TARGET, WAIT_OPTIM, WAIT_FLYING
    } planner_state;
    double start_time_;  //开始规划的时间
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
