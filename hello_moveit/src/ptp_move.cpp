#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

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

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Set the planning pipeline to Pilz
  arm_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm_group_interface.setPlannerId("PTP");
  arm_group_interface.setPlanningTime(20.0);
  arm_group_interface.setMaxVelocityScalingFactor(0.5);
  arm_group_interface.setMaxAccelerationScalingFactor(0.5);

  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());    
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  // Wait for the current robot state to be available
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Define the target poses for making a square of 10 cm sides
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Initial point (bottom-left corner of the square)
  geometry_msgs::msg::Pose point1;
  point1.position.x = 0.5;
  point1.position.y = -0.176;
  point1.position.z = 0.85;
  point1.orientation.x = 0.0;
  point1.orientation.y = 0.0;
  point1.orientation.z = 0.0;
  point1.orientation.w = 1.0;
  waypoints.push_back(point1);

  // Second point
  geometry_msgs::msg::Pose point2 = point1;
  point2.position.x += 0.1;
  waypoints.push_back(point2);

  // Third point
  geometry_msgs::msg::Pose point3 = point2;
  point3.position.y += 0.1;
  waypoints.push_back(point3);

  // Fourth point
  geometry_msgs::msg::Pose point4 = point3;
  point4.position.x -= 0.1;
  waypoints.push_back(point4);

  // Return to the starting point 
  waypoints.push_back(point1);

  // Iterate over waypoints to execute the square path
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    arm_group_interface.setPoseTarget(waypoints[i]);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = arm_group_interface.plan(plan);

    if (ok == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Executing move to waypoint %zu", i + 1);
      arm_group_interface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning failed for waypoint %zu!", i + 1);
      break;
    }
  }

  // Draw the path in RViz
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
  moveit_visual_tools.trigger();

  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}
