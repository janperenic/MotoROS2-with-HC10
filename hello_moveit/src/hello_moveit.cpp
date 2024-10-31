#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.5;  // Place text 1.5m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "manipulator")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  // Example Cartesian Path Integration Section (refer to the previous answer)
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

  // Set start_pose as the first waypoint
  waypoints.push_back(start_pose);

  // Define other waypoints
  geometry_msgs::msg::Pose target_pose = start_pose;
  target_pose.position.z -= 0.2;  // Move down 20 cm
  waypoints.push_back(target_pose);

  target_pose.position.z += 0.2;  // Move up 20 cm back to origin
  waypoints.push_back(target_pose);

  target_pose.position.z += 0.2;  // Move up 20 cm
  waypoints.push_back(target_pose);

  target_pose.position.z -= 0.2;  // Move down 20 cm back to origin
  waypoints.push_back(target_pose);

  target_pose.position.y -= 0.2;  // Move right 20 cm
  waypoints.push_back(target_pose);

  target_pose.position.z += 0.2;  // Move up 20 cm
  target_pose.position.y += 0.2;  // Move back to original Y
  target_pose.position.x -= 0.2;  // Move left 20 cm
  waypoints.push_back(target_pose);

  // Compute Cartesian Path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing plan (Cartesian path) (%.2f%% achieved)",
              fraction * 100.0);

  // Visualize Cartesian Path in RViz
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.publishText(start_pose, "Cartesian_Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    moveit_visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
  moveit_visual_tools.trigger();
  prompt("Press 'next' in the RvizVisualToolsGui window to execute Cartesian Path");

  // Execute Cartesian Path
  move_group_interface.execute(trajectory);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}