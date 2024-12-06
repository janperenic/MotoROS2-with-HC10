// #include <memory>
// #include <thread>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// /**
//  * Pilz Example -- MoveGroup Interface for Motoman HC10DT
//  *
//  * To run this example, ensure the correct setup for HC10DT.
//  *
//  * For best results, hide the "MotionPlanning" widget in RViz.
//  *
//  * Then, run this file:
//  * ros2 run your_package_name hc10dt_move_group
//  */

// int main(int argc, char* argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//       "hc10dt_move_group_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hc10dt_move_group_node");

//   // Create the MoveIt MoveGroup Interface for "manipulator"
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "manipulator");

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools =
//       moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
//                                               move_group_interface.getRobotModel() };
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   // Create closures for visualization
//   auto const draw_title = [&moveit_visual_tools](const auto& text) {
//     auto const text_pose = [] {
//       auto msg = Eigen::Isometry3d::Identity();
//       msg.translation().z() = 1.0;
//       return msg;
//     }();
//     moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//   };
//   auto const prompt = [&moveit_visual_tools](const auto& text) { moveit_visual_tools.prompt(text); };
//   auto const draw_trajectory_tool_path =
//       [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("manipulator")](
//           auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

//   // Helper to plan and execute motion
//   auto const plan_and_execute = [&](const std::string& title) {
//     prompt("Press 'Next' in the RVizVisualToolsGui window to plan");
//     draw_title("Planning " + title);
//     moveit_visual_tools.trigger();
//     auto const [success, plan] = [&move_group_interface] {
//       moveit::planning_interface::MoveGroupInterface::Plan msg;
//       auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//       return std::make_pair(ok, msg);
//     }();

//     // Execute the plan
//     if (success)
//     {
//       draw_trajectory_tool_path(plan.trajectory_);
//       moveit_visual_tools.trigger();
//       prompt("Press 'Next' in the RVizVisualToolsGui window to execute");
//       draw_title("Executing " + title);
//       moveit_visual_tools.trigger();
//       move_group_interface.execute(plan);
//     }
//     else
//     {
//       RCLCPP_ERROR(logger, "Planning failed!");
//       draw_title("Planning Failed!");
//       moveit_visual_tools.trigger();
//     }
//   };

//   // Plan and execute a multi-step sequence using Pilz
//   move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");

//   {
//     // Move to a pre-grasp pose
//     move_group_interface.setPlannerId("PTP");
//     auto const pre_grasp_pose = [] {
//       geometry_msgs::msg::PoseStamped msg;
//       msg.header.frame_id = "world";
//       msg.pose.orientation.w = 1.0;
//       msg.pose.position.x = 0.5;
//       msg.pose.position.y = -0.2;
//       msg.pose.position.z = 0.85;
//       return msg;
//     }();
//     move_group_interface.setPoseTarget(pre_grasp_pose, "tool0");
//     plan_and_execute("[PTP] Approach");
//   }

//   {
//     // Move in a linear trajectory to a grasp pose using the LIN planner.
//     move_group_interface.setPlannerId("LIN");
//     auto const grasp_pose = [] {
//       geometry_msgs::msg::PoseStamped msg;
//       msg.header.frame_id = "world";
//       msg.pose.orientation.w = 1.0;
//       msg.pose.position.x = 0.5;
//       msg.pose.position.y = -0.2;
//       msg.pose.position.z = 0.9;
//       return msg;
//     }();
//     move_group_interface.setPoseTarget(grasp_pose, "tool0");
//     plan_and_execute("[LIN] Grasp");
//   }

//   {
//     // Move back home using the PTP planner.
//     move_group_interface.setPlannerId("PTP");
//     move_group_interface.setNamedTarget("ready");
//     plan_and_execute("[PTP] Return");
//   }

//   // Shutdown ROS
//   spinner.join();
//   rclcpp::shutdown();
//   return 0;
// }








// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// ne dela ker ne zazna pilz






// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// int main(int argc, char * argv[])
// {
//   // Start up ROS 2
//   rclcpp::init(argc, argv);
   
//   auto const node = std::make_shared<rclcpp::Node>(
//     "hello_moveit",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   auto const logger = rclcpp::get_logger("hello_moveit");

//   using moveit::planning_interface::MoveGroupInterface;
//   auto arm_group_interface = MoveGroupInterface(node, "manipulator");

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC};
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   // Set the planning pipeline to Pilz
//   arm_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
//   arm_group_interface.setPlannerId("PTP");
//   arm_group_interface.setPlanningTime(20.0);
//   arm_group_interface.setMaxVelocityScalingFactor(0.5);
//   arm_group_interface.setMaxAccelerationScalingFactor(0.5);

//   RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());    
//   RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
//   RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

//   // Wait for the current robot state to be available
//   rclcpp::sleep_for(std::chrono::seconds(5));

//   // Define the target poses for making a square of 10 cm sides
//   std::vector<geometry_msgs::msg::Pose> waypoints;

//   // Initial point (bottom-left corner of the square)
//   geometry_msgs::msg::Pose point1;
//   point1.position.x = 0.5;
//   point1.position.y = -0.176;
//   point1.position.z = 0.85;
//   point1.orientation.x = 0.0;
//   point1.orientation.y = 0.0;
//   point1.orientation.z = 0.0;
//   point1.orientation.w = 1.0;
//   waypoints.push_back(point1);

//   // Second point
//   geometry_msgs::msg::Pose point2 = point1;
//   point2.position.x += 0.1;
//   waypoints.push_back(point2);

//   // Third point
//   geometry_msgs::msg::Pose point3 = point2;
//   point3.position.y += 0.1;
//   waypoints.push_back(point3);

//   // Fourth point
//   geometry_msgs::msg::Pose point4 = point3;
//   point4.position.x -= 0.1;
//   waypoints.push_back(point4);

//   // Return to the starting point 
//   waypoints.push_back(point1);

//   // Iterate over waypoints to execute the square path
//   for (size_t i = 0; i < waypoints.size(); ++i)
//   {
//     arm_group_interface.setPoseTarget(waypoints[i]);

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     auto const ok = arm_group_interface.plan(plan);

//     if (ok == moveit::core::MoveItErrorCode::SUCCESS)
//     {
//       RCLCPP_INFO(logger, "Executing move to waypoint %zu", i + 1);
//       arm_group_interface.execute(plan);
//     }
//     else
//     {
//       RCLCPP_ERROR(logger, "Planning failed for waypoint %zu!", i + 1);
//       break;
//     }
//   }

//   // Draw the path in RViz
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
//   moveit_visual_tools.trigger();

//   // Shut down ROS 2 cleanly when we're done
//   rclcpp::shutdown();
//   return 0;
// }











// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// THE SHIT CODE -the one that works hehe



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
  move_group_interface.setMaxVelocityScalingFactor(0.5);
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
  spinner.join();
  rclcpp::shutdown();
  return 0;
}











// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



