
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <r2_interfaces/srv/gripper_cmd.hpp>
#include <math.h>


// forward declaration of service callbacks
void grabServiceCallback(const std::shared_ptr<r2_interfaces::srv::GripperCmd::Request> request, std::shared_ptr<r2_interfaces::srv::GripperCmd::Response> response);
void liftServiceCallback(const std::shared_ptr<r2_interfaces::srv::GripperCmd::Request> request, std::shared_ptr<r2_interfaces::srv::GripperCmd::Response> response);



class gripperSrv : public rclcpp::Node {

public:

    gripperSrv() : Node("gripperSrv"){

        publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
        grabService = this->create_service<r2_interfaces::srv::GripperCmd>("gripper_grab", &grabServiceCallback);
        liftService = this->create_service<r2_interfaces::srv::GripperCmd>("gripper_lift", &liftServiceCallback);

        timer = this->create_wall_timer( std::chrono::milliseconds(100) , std::bind(&gripperSrv::publishCallback, this));
        RCLCPP_INFO(get_logger(), "gripper service initialized");

        traject = trajectory_msgs::msg::JointTrajectory();
        traject.set__joint_names({"lift_base_link", "claw_left_lift", "claw_right_lift"});
    };


    void publishCallback(){

        point.set__positions(positions);
        point.time_from_start.sec = time;
        traject.set__points({point});
        publisher->publish(traject);
    };

    // save state of the gripper
    std::vector<double> positions = {0.0f, 0.015f, 0.015f};
    int32_t time = 4;

private:
    
    trajectory_msgs::msg::JointTrajectory traject;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<r2_interfaces::srv::GripperCmd>::SharedPtr grabService, liftService;

};

std::shared_ptr<gripperSrv> srvObj;


void grabServiceCallback(const std::shared_ptr<r2_interfaces::srv::GripperCmd::Request> request, std::shared_ptr<r2_interfaces::srv::GripperCmd::Response> response){
    response->result = true;
    RCLCPP_INFO(srvObj->get_logger(), " grap service called");

    srvObj->time = 1;
    if(request->data) srvObj->positions[1] = srvObj->positions[2] = 0.015f;
    else srvObj->positions[1] = srvObj->positions[2] = M_PI / 4.0f;
};


void liftServiceCallback(const std::shared_ptr<r2_interfaces::srv::GripperCmd::Request> request, std::shared_ptr<r2_interfaces::srv::GripperCmd::Response> response){
    response->result = true;
    RCLCPP_INFO(srvObj->get_logger(), " lift service called");

    srvObj->time = 5;
    if(request->data) srvObj->positions[0] = 0.546f;
    else srvObj->positions[0] = 0.0f;
};




int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(srvObj = std::make_shared<gripperSrv>());
    rclcpp::shutdown();
}