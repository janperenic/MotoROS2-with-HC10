#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // We spin up a SingleThreadedExecutor so MoveItVisualTools can interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Set the planning pipeline to Pilz
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("LIN"); // Set to 'PTP' for point-to-point movement
  move_group_interface.setPlanningTime(20.0);
  move_group_interface.setMaxVelocityScalingFactor(0.5); // sprememba hitrosti
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  RCLCPP_INFO(logger, "Planning pipeline: %s", move_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", move_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", move_group_interface.getPlanningTime());

  // Wait for the current robot state to be available
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Define target waypoints for movement
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Initial point (bottom-left corner of the square)
  geometry_msgs::msg::Pose point1;
  point1.position.x = 0.5;
  point1.position.y = 0.1;
  point1.position.z = 1.25;
  point1.orientation.x = 0.0;
  point1.orientation.y = 0.0;
  point1.orientation.z = 0.0;
  point1.orientation.w = 1.0;
  waypoints.push_back(point1);

  // ##################  LIN 10  ######################

    // !!!!!!!!!!!!!!  X  !!!!!!!!!!!!!!

  // Other waypoints to form a square
  geometry_msgs::msg::Pose point2 = point1;
  point2.position.x += 0.2;
  waypoints.push_back(point2);

  geometry_msgs::msg::Pose point3 = point2;
  point3.position.y += 0.2;
  waypoints.push_back(point3);

  geometry_msgs::msg::Pose point4 = point3;
  point4.position.x -= 0.2;
  waypoints.push_back(point4);

  // Return to the starting point
  waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  X  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.1;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.x -= 0.1;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.1;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  Y  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.y += 0.1;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.y -= 0.1;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.y -= 0.1;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  Z  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.z += 0.1;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z -= 0.1;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.z -= 0.1;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);


  // ##################  LIN 20  ######################

    // !!!!!!!!!!!!!!  X  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.2;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.x -= 0.2;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.2;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);


    // !!!!!!!!!!!!!!  Y  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.y += 0.2;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.y -= 0.2;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.y -= 0.2;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  Z  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.z += 0.2;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z -= 0.2;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.z -= 0.2;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);



  // ##################  LIN 40  ######################

    // !!!!!!!!!!!!!!  X  !!!!!!!!!!!!!!


  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.4;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.x -= 0.4;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.4;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);


    // !!!!!!!!!!!!!!  Y  !!!!!!!!!!!!!!


  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.y += 0.4;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.y -= 0.4;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.y -= 0.4;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  Z  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.z += 0.4;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z -= 0.4;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.z -= 0.4;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);



  // ##################  PLAIN 10  ######################

    // !!!!!!!!!!!!!!  XY  !!!!!!!!!!!!!!
  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.1;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.y += 0.1;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.1;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  YZ  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.y += 0.1;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z += 0.1;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.y -= 0.1;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  XZ  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.1;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z += 0.1;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.1;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);




  // ##################  PLAIN 20  ######################

    // !!!!!!!!!!!!!!  XY  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.2;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.y += 0.2;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.2;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  YZ  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.y += 0.2;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z += 0.2;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.y -= 0.2;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);    

    // !!!!!!!!!!!!!!  XZ  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.2;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z += 0.2;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.2;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);



  // ##################  PLAIN 40  ######################

    // !!!!!!!!!!!!!!  XY  !!!!!!!!!!!!!!
  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.4;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.y += 0.4;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.4;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  YZ  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.y += 0.4;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z += 0.4;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.y -= 0.4;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);

    // !!!!!!!!!!!!!!  XZ  !!!!!!!!!!!!!!

  // geometry_msgs::msg::Pose point2 = point1;
  // point2.position.x += 0.4;
  // waypoints.push_back(point2);

  // geometry_msgs::msg::Pose point3 = point2;
  // point3.position.z += 0.4;
  // waypoints.push_back(point3);

  // geometry_msgs::msg::Pose point4 = point3;
  // point4.position.x -= 0.4;
  // waypoints.push_back(point4);

  // // Return to the starting point
  // waypoints.push_back(point1);




  // Execute the waypoints
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    move_group_interface.setPoseTarget(waypoints[i]);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = move_group_interface.plan(plan);

    if (ok == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Executing move to waypoint %zu", i + 1);
      move_group_interface.execute(plan);

      // Draw the path in RViz after execution
      moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
      moveit_visual_tools.trigger();
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning failed for waypoint %zu!", i + 1);
      break;
    }
  }

  // Shut down ROS 2 cleanly when we're done
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
