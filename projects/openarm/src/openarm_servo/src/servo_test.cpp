#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_tutorials.servo_node.cpp");
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_;
    rclcpp::NodeOptions node_options;

    // This is false for now until we fix the QoS settings in moveit to enable intra process comms
    node_options.use_intra_process_comms(false);
    node_ = std::make_shared<rclcpp::Node>("servo_node", node_options);
    std::cout<<"====================node created===================="<<std::endl;
    // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
    rclcpp::sleep_for(std::chrono::seconds(4));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
  // before initializing any collision objects
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", tf_buffer, "planning_scene_monitor");
    
    if (!planning_scene_monitor->getPlanningScene())
    {
        RCLCPP_ERROR(LOGGER, "Planning scene not configured");
        return EXIT_FAILURE;
    }
    
    // Start the planning scene monitor
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor("/joint_states");  // Explicitly specify joint_states topic
    planning_scene_monitor->startWorldGeometryMonitor();
    
    // Wait for initial joint states
    RCLCPP_INFO(LOGGER, "Waiting for joint states...");
    planning_scene_monitor->waitForCurrentRobotState(node_->now(), 5.0);
    
    std::cout<<"====================planning scene monitor created===================="<<std::endl;

    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
    if (!servo_parameters){
        RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
        return EXIT_FAILURE;
    }
    std::cout<<"====================servo parameters loaded===================="<<std::endl;

    auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);
    servo->start();
    //   std::cout<<"====================servo started===================="<<std::endl;

    // Sending Commands
    // ^^^^^^^^^^^^^^^^
    // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot

    // CALL_SUB_TUTORIAL publishCommands
    auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_);
    executor->spin();

    // END_TUTORIAL

    rclcpp::shutdown();
    return 0;
}