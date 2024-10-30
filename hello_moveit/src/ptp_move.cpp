#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);
   
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "manipulator");

  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
  arm_group_interface.setPlanningTime(5.0);
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());    
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  // Wait for the current robot state to be available
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Set a target pose for the end effector of the arm 
  auto const arm_target_pose = [&node] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now(); 
    msg.pose.position.x = 0.5;
    msg.pose.position.y = 0.176;
    msg.pose.position.z = 0.9;
    msg.pose.orientation.x = -0.346183;
    msg.pose.orientation.y = 0.938166; // ce je na ena vrtenje po globalni z osi
    msg.pose.orientation.z = 0.0002756;
    msg.pose.orientation.w = 0.0007470; // Valid quaternion orientation
    return msg;
  }();
  arm_group_interface.setPoseTarget(arm_target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const ok = arm_group_interface.plan(plan);

  if (ok == moveit::core::MoveItErrorCode::SUCCESS)
  {
    arm_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
   
  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}
