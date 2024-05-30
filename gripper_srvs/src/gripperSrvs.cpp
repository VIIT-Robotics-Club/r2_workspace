
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <r2_interfaces/srv/gripper_cmd.hpp>
#include <math.h>

using namespace std::placeholders;

// joystick button indexes for mapping
#define GRIPPER_OPEN_BTN 3
#define GRIPPER_CLOSE_BTN 1
#define GRIPPER_LIFTUP_BTN 2
#define GRIPPER_LIFTDOWN_BTN 0

/**
 * @brief GripperSrv node exposes services to operate gripper in simulation
 *  gripper can be operated via joystick or call exposed services 
 * 
 */
class gripperSrv : public rclcpp::Node {

public:

    gripperSrv() : Node("gripperSrv"){

        joySub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&gripperSrv::joyCallback, this, _1));
        publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("traject_out", 10);
        grabService = this->create_service<r2_interfaces::srv::GripperCmd>("gripper_grab", std::bind(&gripperSrv::grabServiceCallback, this, _1, _2));
        liftService = this->create_service<r2_interfaces::srv::GripperCmd>("gripper_lift", std::bind(&gripperSrv::liftServiceCallback, this, _1, _2));

        timer = this->create_wall_timer( std::chrono::milliseconds(100) , std::bind(&gripperSrv::publishCallback, this));
        RCLCPP_INFO(get_logger(), "gripper service initialized");

        traject.set__joint_names({"lift_base_link", "claw_left_lift", "claw_right_lift"});
    };


    void publishCallback(){
        if(time){
            point.set__positions(positions);
            point.time_from_start.sec = time;
            traject.set__points({point});
            publisher->publish(traject);
            time = 0;
        }
    };

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){

        if(msg->buttons[GRIPPER_OPEN_BTN]) {
            positions[1] = positions[2] = M_PI / 4.0f;
            time = 1;
        }
        else if(msg->buttons[GRIPPER_CLOSE_BTN]) {
            positions[1] = positions[2] = 0.015f;
            time = 1;
        }

        if(msg->buttons[GRIPPER_LIFTUP_BTN]) {
            positions[0] = 0.546f;
            time = 7;
        }
        else if(msg->buttons[GRIPPER_LIFTDOWN_BTN]) {
            positions[0] = 0.0f;
            time = 7;
        }
    };


    void grabServiceCallback(const std::shared_ptr<r2_interfaces::srv::GripperCmd::Request> request, std::shared_ptr<r2_interfaces::srv::GripperCmd::Response> response){
        response->result = true;
        RCLCPP_INFO(get_logger(), " grab service called");

        if(request->data) positions[1] = positions[2] = 0.015f;
        else positions[1] = positions[2] = M_PI / 4.0f;

        time = 1;
    };


    void liftServiceCallback(const std::shared_ptr<r2_interfaces::srv::GripperCmd::Request> request, std::shared_ptr<r2_interfaces::srv::GripperCmd::Response> response){
        response->result = true;
        RCLCPP_INFO(get_logger(), " lift service called");

        if(request->data) positions[0] = 0.546f;
        else positions[0] = 0.0f;

        time = 7;
    };

    // save state of the gripper
    std::vector<double> positions = {0.0f, 0.015f, 0.015f};
    int32_t time = 4;

private:
    
    trajectory_msgs::msg::JointTrajectory traject;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<r2_interfaces::srv::GripperCmd>::SharedPtr grabService, liftService;
};






int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gripperSrv>());
    rclcpp::shutdown();
}