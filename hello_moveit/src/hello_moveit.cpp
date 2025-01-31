// ################################################################
// naj bi delala za vec trajektori in umes stala med njimi 5 sekund
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Function to execute a set of waypoints
void execute_waypoints(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    moveit_visual_tools::MoveItVisualTools& moveit_visual_tools,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const rclcpp::Logger& logger)
{
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        move_group_interface.setPoseTarget(waypoints[i]);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const ok = move_group_interface.plan(plan);

        if (ok == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Executing move to waypoint %zu", i + 1);
            move_group_interface.execute(plan);

            // Draw the path in RViz
            moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
            moveit_visual_tools.trigger();
        }
        else
        {
            RCLCPP_ERROR(logger, "Planning failed for waypoint %zu!", i + 1);
            break;
        }
    }
}

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
    move_group_interface.setPlannerId("PTP"); // Set to 'PTP' for point-to-point movement
    move_group_interface.setPlanningTime(20.0);
    move_group_interface.setMaxVelocityScalingFactor(0.99);
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
    // Define a base pose to use as a reference for all movements
    geometry_msgs::msg::Pose base_pose;
    base_pose.position.x = 0.5;
    base_pose.position.y = 0.1;
    base_pose.position.z = 1.25;
    base_pose.orientation.x = 0.0;
    base_pose.orientation.y = 0.0;
    base_pose.orientation.z = 0.0;
    base_pose.orientation.w = 1.0;

    // --- Define multiple sets of waypoints ---
    std::vector<std::vector<geometry_msgs::msg::Pose>> trajectory_sets;


    // ###############  LIN 10  ############### 
    // Set 1: Lin move x axis
    std::vector<geometry_msgs::msg::Pose> waypoints_set1;
    geometry_msgs::msg::Pose point1 = base_pose;
    waypoints_set1.push_back(point1);

    geometry_msgs::msg::Pose point2 = base_pose;
    point2.position.x += 0.1;
    waypoints_set1.push_back(point2);

    geometry_msgs::msg::Pose point3 = base_pose;
    point3.position.x -= 0.1;
    waypoints_set1.push_back(point3);

    geometry_msgs::msg::Pose point4 = base_pose;
    point4.position.x -= 0.1;
    waypoints_set1.push_back(point4);

    waypoints_set1.push_back(point1); // Return to start

    trajectory_sets.push_back(waypoints_set1); // Add to list of trajectories

    // Set 2: Square movement in Y axis
    std::vector<geometry_msgs::msg::Pose> waypoints_set2;
    geometry_msgs::msg::Pose point1_Y = base_pose;
    waypoints_set2.push_back(point1_Y);

    geometry_msgs::msg::Pose point2_Y = base_pose;
    point2_Y.position.y += 0.1;
    waypoints_set2.push_back(point2_Y);

    geometry_msgs::msg::Pose point3_Y = base_pose;
    point3_Y.position.y -= 0.1;
    waypoints_set2.push_back(point3_Y);

    geometry_msgs::msg::Pose point4_Y = base_pose;
    point4_Y.position.y -= 0.1;
    waypoints_set2.push_back(point4_Y);

    waypoints_set2.push_back(point1_Y);

    trajectory_sets.push_back(waypoints_set2); // Add to list of trajectories

    // Set 3: Movement in Z direction
    std::vector<geometry_msgs::msg::Pose> waypoints_set3;
    geometry_msgs::msg::Pose point1_Z = base_pose;
    waypoints_set3.push_back(point1_Z);

    geometry_msgs::msg::Pose point2_Z = base_pose;
    point2_Z.position.z += 0.1;
    waypoints_set3.push_back(point2_Z);

    geometry_msgs::msg::Pose point3_Z = base_pose;
    point3_Z.position.z -= 0.1;
    waypoints_set3.push_back(point3_Z);

    geometry_msgs::msg::Pose point4_Z = base_pose;
    point4_Z.position.z -= 0.1;
    waypoints_set3.push_back(point4_Z);

    waypoints_set3.push_back(point1_Z);

    trajectory_sets.push_back(waypoints_set3);

    // Set 3: Movement in Z direction
    std::vector<geometry_msgs::msg::Pose> waypoints_set4;
    geometry_msgs::msg::Pose point1_X20 = base_pose;
    waypoints_set4.push_back(point1_X20);

    geometry_msgs::msg::Pose point2_X20 = base_pose;
    point2_X20.position.x += 0.2;
    waypoints_set4.push_back(point2_X20);

    geometry_msgs::msg::Pose point3_X20 = base_pose;
    point3_X20.position.x -= 0.2;
    waypoints_set4.push_back(point3_X20);

    geometry_msgs::msg::Pose point4_X20 = base_pose;
    point4_X20.position.x -= 0.2;
    waypoints_set4.push_back(point4_X20);

    waypoints_set4.push_back(point1_X20);

    trajectory_sets.push_back(waypoints_set4);




    // --- Execute all sets sequentially with pauses ---
    for (size_t i = 0; i < trajectory_sets.size(); ++i)
    {
        RCLCPP_INFO(logger, "Executing trajectory set %zu", i + 1);
        execute_waypoints(move_group_interface, moveit_visual_tools, trajectory_sets[i], logger);

        if (i < trajectory_sets.size() - 1) // No need to wait after the last set
        {
            RCLCPP_INFO(logger, "Waiting 5 seconds before executing the next trajectory set...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // Shut down ROS 2 cleanly when we're done
    spinner.join();
    rclcpp::shutdown();
    return 0;
}






// #################################################################

// #include <memory>
// #include <thread>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// int main(int argc, char *argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//     "hello_moveit",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "manipulator");

//   // Set start state to the current state
//   move_group_interface.setStartStateToCurrentState();

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
//     node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
//     move_group_interface.getRobotModel()};
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   // Get the current pose of the end-effector
//   geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

//   // Define waypoints for Cartesian path
//   std::vector<geometry_msgs::msg::Pose> waypoints;

//   // Set start_pose as the first waypoint
//   waypoints.push_back(start_pose);

//   // Define other waypoints
//   geometry_msgs::msg::Pose target_pose = start_pose;

//   // Move down 20 cm
//   target_pose.position.z -= 0.1;
//   waypoints.push_back(target_pose);

//   // Move up 20 cm back to origin
//   target_pose.position.z += 0.1;
//   waypoints.push_back(target_pose);

//   // Move up another 20 cm
//   target_pose.position.z += 0.1;
//   waypoints.push_back(target_pose);

//   // Move down 20 cm back to origin
//   target_pose.position.z -= 0.1;
//   waypoints.push_back(target_pose);

//   // Move right 20 cm
//   target_pose.position.y -= 0.1;
//   waypoints.push_back(target_pose);

//   // Move up 20 cm and adjust other axes
//   target_pose.position.z += 0.1;
//   target_pose.position.y += 0.1;
//   target_pose.position.x -= 0.1;  // Move left 20 cm
//   waypoints.push_back(target_pose);

//   // Move up 20 cm and adjust other axes
//   target_pose.position.x += 0.1;
//   target_pose.position.z -= 0.1;
//   waypoints.push_back(target_pose);

//   // Compute Cartesian Path
//   moveit_msgs::msg::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group_interface.computeCartesianPath(
//       waypoints, eef_step, jump_threshold, trajectory);
//   RCLCPP_INFO(logger, "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

//   // Visualize Cartesian Path in RViz
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.publishText(start_pose, "Cartesian_Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//   moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
//   for (std::size_t i = 0; i < waypoints.size(); ++i)
//     moveit_visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
//   moveit_visual_tools.trigger();
//   moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute Cartesian Path");

//   // Execute Cartesian Path
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   plan.trajectory_ = trajectory;

//   bool success = (move_group_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   if (!success)
//   {
//     RCLCPP_ERROR(logger, "Failed to execute planned trajectory");
//   }

//   // Shutdown ROS
//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }











// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!









// #include <memory>
// #include <thread>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// int main(int argc, char *argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//     "hello_moveit",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "manipulator");

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
//     node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
//     move_group_interface.getRobotModel()};
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   // Set start state to the current state
//   move_group_interface.setStartStateToCurrentState();

//   // Get the current pose of the end-effector
//   geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

//   // Define waypoints for Cartesian path
//   std::vector<geometry_msgs::msg::Pose> waypoints;

//   // Set start_pose as the first waypoint
//   waypoints.push_back(start_pose);

//   // Define other waypoints
//   geometry_msgs::msg::Pose target_pose = start_pose;

//   // Move down 20 cm
//   target_pose.position.z -= 0.1;
//   waypoints.push_back(target_pose);

//   // Move up 20 cm back to origin
//   target_pose.position.z += 0.1;
//   waypoints.push_back(target_pose);

//   // Move up another 20 cm
//   target_pose.position.z += 0.1;
//   waypoints.push_back(target_pose);

//   // Move down 20 cm back to origin
//   target_pose.position.z -= 0.1;
//   waypoints.push_back(target_pose);

//   // Move right 20 cm
//   target_pose.position.y -= 0.1;
//   waypoints.push_back(target_pose);

//   // Move up 20 cm and adjust other axes
//   target_pose.position.z += 0.1;
//   target_pose.position.y += 0.1;
//   target_pose.position.x -= 0.1;  // Move left 20 cm
//   waypoints.push_back(target_pose);

//   // Move up 20 cm and adjust other axes
//   target_pose.position.z += 0.01;

//   waypoints.push_back(target_pose);

//   // Compute Cartesian Path
//   moveit_msgs::msg::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group_interface.computeCartesianPath(
//       waypoints, eef_step, jump_threshold, trajectory);
//   RCLCPP_INFO(logger, "Visualizing plan (Cartesian path) (%.2f%% achieved)",
//               fraction * 100.0);

//   // Visualize Cartesian Path in RViz
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.publishText(start_pose, "Cartesian_Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//   moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
//   for (std::size_t i = 0; i < waypoints.size(); ++i)
//     moveit_visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
//   moveit_visual_tools.trigger();
//   moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute Cartesian Path");

//   // Execute Cartesian Path
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   plan.trajectory_ = trajectory;

//   bool success = (move_group_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   if (!success)
//   {
//     RCLCPP_ERROR(logger, "Failed to execute planned trajectory");
//   }

//   // Shutdown ROS
//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }





// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!










// #include <memory>
// #include <thread>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// int main(int argc, char *argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//     "hello_moveit",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "manipulator");

//   // Set the start state to the current state
//   move_group_interface.setStartState(*move_group_interface.getCurrentState());

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
//     node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
//     move_group_interface.getRobotModel()};
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   // Create closures for visualization
//   auto const draw_title = [&moveit_visual_tools](auto text) {
//     auto const text_pose = [] {
//       auto msg = Eigen::Isometry3d::Identity();
//       msg.translation().z() = 1.5;  // Place text 1.5m above the base link
//       return msg;
//     }();
//     moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
//                                     rviz_visual_tools::XLARGE);
//   };
//   auto const prompt = [&moveit_visual_tools](auto text) {
//     moveit_visual_tools.prompt(text);
//   };
//   auto const draw_trajectory_tool_path =
//       [&moveit_visual_tools,
//       jmg = move_group_interface.getRobotModel()->getJointModelGroup(
//           "manipulator")](auto const trajectory) {
//         moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
//       };

//   // Example Cartesian Path Integration Section (refer to the previous answer)
//   std::vector<geometry_msgs::msg::Pose> waypoints;
//   geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

//   // Set start_pose as the first waypoint
//   // Initial point (bottom-left corner of the square)
//   start_pose.position.x = 0.5;
//   start_pose.position.y = 0.676;
//   start_pose.position.z = 1.25;
//   start_pose.orientation.x = 0.0;
//   start_pose.orientation.y = 0.0;
//   start_pose.orientation.z = 0.0;
//   start_pose.orientation.w = 1.0;
//   waypoints.push_back(start_pose);

//   // Define other waypoints
//   geometry_msgs::msg::Pose target_pose = start_pose;
//   target_pose.position.z -= 0.05;  // Move down 20 cm
//   waypoints.push_back(target_pose);

//   target_pose.position.z += 0.05;  // Move up 20 cm back to origin
//   waypoints.push_back(target_pose);

//   target_pose.position.z += 0.05;  // Move up 20 cm
//   waypoints.push_back(target_pose);

//   target_pose.position.z -= 0.05;  // Move down 20 cm back to origin
//   waypoints.push_back(target_pose);

//   target_pose.position.y -= 0.05;  // Move right 20 cm
//   waypoints.push_back(target_pose);

//   target_pose.position.z += 0.05;  // Move up 20 cm
//   target_pose.position.y += 0.05;  // Move back to original Y
//   target_pose.position.x -= 0.05;  // Move left 20 cm
//   waypoints.push_back(target_pose);

//   // Compute Cartesian Path
//   moveit_msgs::msg::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group_interface.computeCartesianPath(
//       waypoints, eef_step, jump_threshold, trajectory);
//   RCLCPP_INFO(logger, "Visualizing plan (Cartesian path) (%.2f%% achieved)",
//               fraction * 100.0);

//   // Visualize Cartesian Path in RViz
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.publishText(start_pose, "Cartesian_Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//   moveit_visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
//   for (std::size_t i = 0; i < waypoints.size(); ++i)
//     moveit_visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
//   moveit_visual_tools.trigger();
//   prompt("Press 'next' in the RvizVisualToolsGui window to execute Cartesian Path");

//   // Execute Cartesian Path
//   move_group_interface.execute(trajectory);

//   // Shutdown ROS
//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }
