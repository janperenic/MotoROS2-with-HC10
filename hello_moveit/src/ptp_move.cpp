//  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Function to execute a set of waypoints and calculate average speed
double execute_waypoints(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    moveit_visual_tools::MoveItVisualTools& moveit_visual_tools,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const rclcpp::Logger& logger)
{
    double total_speed = 0.0;
    size_t speed_count = 0; // Number of velocity data points

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        move_group_interface.setPoseTarget(waypoints[i]);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const ok = move_group_interface.plan(plan);

        if (ok == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Executing move to waypoint %zu", i + 1);
            move_group_interface.execute(plan);

            // Extract velocity from trajectory
            if (!plan.trajectory_.joint_trajectory.points.empty())
            {
                for (size_t j = 0; j < plan.trajectory_.joint_trajectory.points.size(); ++j)
                {
                    auto velocities = plan.trajectory_.joint_trajectory.points[j].velocities;
                    double speed = 0.0;

                    // Compute speed as the magnitude of joint velocities
                    for (double v : velocities)
                    {
                        speed += v * v; // Sum of squared velocities
                    }
                    speed = std::sqrt(speed); // Convert to scalar speed

                    total_speed += speed;
                    speed_count++;

                    // RCLCPP_INFO(logger, "Waypoint %zu, Time: %.2f sec, Speed: %.3f rad/s", 
                    //             j, 
                    //             plan.trajectory_.joint_trajectory.points[j].time_from_start.sec + 
                    //             plan.trajectory_.joint_trajectory.points[j].time_from_start.nanosec * 1e-9,
                    //             speed);
                }
            }

            // Visualize trajectory in RViz
            moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
            moveit_visual_tools.trigger();
        }
        else
        {
            RCLCPP_ERROR(logger, "Planning failed for waypoint %zu!", i + 1);
            break;
        }
    }

    // Calculate and return the average speed for this trajectory
    double avg_speed = (speed_count > 0) ? (total_speed / speed_count) : 0.0;
    RCLCPP_INFO(logger, "Average speed for this trajectory: %.3f rad/s", avg_speed);

    return avg_speed;
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
    move_group_interface.setPlannerId("LIN"); // Set to 'PTP' for point-to-point movement
    move_group_interface.setPlanningTime(3.0);
    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setMaxAccelerationScalingFactor(0.2);

    // Construct and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    RCLCPP_INFO(logger, "Planning pipeline: %s", move_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", move_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", move_group_interface.getPlanningTime());

        // Wait for the current robot state to be available
        rclcpp::sleep_for(std::chrono::seconds(2));
    // Define a base pose to use as a reference for all movements
    geometry_msgs::msg::Pose base_pose;
    base_pose.position.x = 1.08;
    base_pose.position.y = 0.0;
    base_pose.position.z = 1.0;
    base_pose.orientation.x = 0.0;
    base_pose.orientation.y = 0.0;
    base_pose.orientation.z = 0.0;
    base_pose.orientation.w = 1.0;

    // --- Define multiple sets of waypoints ---
    std::vector<std::vector<geometry_msgs::msg::Pose>> trajectory_sets;


    // dodaj if ce je ze v tej tocki potem prekipa point 1 = base_pose ker drugace dobimo error

    // ##############################  LIN 10  ############################## 

    // Set 1: Lin move x axis
    std::vector<geometry_msgs::msg::Pose> waypoints_set1;
    geometry_msgs::msg::Pose point1 = base_pose;
    waypoints_set1.push_back(point1);

    geometry_msgs::msg::Pose point2 = point1;
    point2.position.x += 0.1;
    waypoints_set1.push_back(point2);

    geometry_msgs::msg::Pose point3 = point2;
    point3.position.x -= 0.1;
    waypoints_set1.push_back(point3);

    geometry_msgs::msg::Pose point4 = point3;
    point4.position.x -= 0.1;
    waypoints_set1.push_back(point4);

    geometry_msgs::msg::Pose point5 = point4;
    point5.position.x += 0.1;
    waypoints_set1.push_back(point5); // Return to start

    trajectory_sets.push_back(waypoints_set1); // Add to list of trajectories

    // Set 2: Lin move Y axis
    std::vector<geometry_msgs::msg::Pose> waypoints_set2;
    // geometry_msgs::msg::Pose point1_Y = point5;
    // waypoints_set2.push_back(point1_Y);

    geometry_msgs::msg::Pose point2_Y = point5;
    point2_Y.position.y += 0.1;
    waypoints_set2.push_back(point2_Y);

    geometry_msgs::msg::Pose point3_Y = point2_Y;
    point3_Y.position.y -= 0.1;
    waypoints_set2.push_back(point3_Y);

    geometry_msgs::msg::Pose point4_Y = point3_Y;
    point4_Y.position.y -= 0.1;
    waypoints_set2.push_back(point4_Y);

    geometry_msgs::msg::Pose point5_Y = point4_Y;
    point5_Y.position.y += 0.1;
    waypoints_set2.push_back(point5_Y);

    trajectory_sets.push_back(waypoints_set2); // Add to list of trajectories
 
    // Set 3: Lin move Z axis
    std::vector<geometry_msgs::msg::Pose> waypoints_set3;
    // geometry_msgs::msg::Pose point1_Z = point5_Y;
    // waypoints_set3.push_back(point1_Z);

    geometry_msgs::msg::Pose point2_Z = point5_Y;
    point2_Z.position.z += 0.1;
    waypoints_set3.push_back(point2_Z);

    geometry_msgs::msg::Pose point3_Z = point2_Z;
    point3_Z.position.z -= 0.1;
    waypoints_set3.push_back(point3_Z);

    geometry_msgs::msg::Pose point4_Z = point3_Z;
    point4_Z.position.z -= 0.1;
    waypoints_set3.push_back(point4_Z);

    geometry_msgs::msg::Pose point5_Z = point4_Z;
    point5_Z.position.z += 0.1;
    waypoints_set3.push_back(point5_Z);

    trajectory_sets.push_back(waypoints_set3);

    // ##############################  LIN 20  ############################## 

   // Set 4: Lin move X axis (0.2m)
    std::vector<geometry_msgs::msg::Pose> waypoints_set4;

    geometry_msgs::msg::Pose point2_X2 = point5_Z;
    point2_X2.position.x += 0.2;
    waypoints_set4.push_back(point2_X2);

    geometry_msgs::msg::Pose point3_X2 = point2_X2;
    point3_X2.position.x -= 0.2;
    waypoints_set4.push_back(point3_X2);

    geometry_msgs::msg::Pose point4_X2 = point3_X2;
    point4_X2.position.x -= 0.2;
    waypoints_set4.push_back(point4_X2);

    geometry_msgs::msg::Pose point5_X2 = point4_X2;
    point5_X2.position.x += 0.2;
    waypoints_set4.push_back(point5_X2); // Return to start

    trajectory_sets.push_back(waypoints_set4);

    // Set 5: Lin move Y axis (0.2m)
    std::vector<geometry_msgs::msg::Pose> waypoints_set5;

    geometry_msgs::msg::Pose point2_Y2 = point5_X2;
    point2_Y2.position.y += 0.2;
    waypoints_set5.push_back(point2_Y2);

    geometry_msgs::msg::Pose point3_Y2 = point2_Y2;
    point3_Y2.position.y -= 0.2;
    waypoints_set5.push_back(point3_Y2);

    geometry_msgs::msg::Pose point4_Y2 = point3_Y2;
    point4_Y2.position.y -= 0.2;
    waypoints_set5.push_back(point4_Y2);

    geometry_msgs::msg::Pose point5_Y2 = point4_Y2;
    point5_Y2.position.y += 0.2;
    waypoints_set5.push_back(point5_Y2);

    trajectory_sets.push_back(waypoints_set5);

    // Set 6: Lin move Z axis (0.2m)
    std::vector<geometry_msgs::msg::Pose> waypoints_set6;

    geometry_msgs::msg::Pose point2_Z2 = point5_Y2;
    point2_Z2.position.z += 0.2;
    waypoints_set6.push_back(point2_Z2);

    geometry_msgs::msg::Pose point3_Z2 = point2_Z2;
    point3_Z2.position.z -= 0.2;
    waypoints_set6.push_back(point3_Z2);

    geometry_msgs::msg::Pose point4_Z2 = point3_Z2;
    point4_Z2.position.z -= 0.2;
    waypoints_set6.push_back(point4_Z2);

    geometry_msgs::msg::Pose point5_Z2 = point4_Z2;
    point5_Z2.position.z += 0.2;
    waypoints_set6.push_back(point5_Z2);

    trajectory_sets.push_back(waypoints_set6);


    // ##############################  PLANE 10 ##############################

    // ###############  PLANE XY ###############

    // Set 7: Plane XY move (0.1m steps)
    std::vector<geometry_msgs::msg::Pose> waypoints_set7;

    geometry_msgs::msg::Pose point2_XY1 = point5_Z2;
    point2_XY1.position.y += 0.1;
    waypoints_set7.push_back(point2_XY1);

    geometry_msgs::msg::Pose point3_XY1 = point2_XY1;
    point3_XY1.position.x += 0.1;
    waypoints_set7.push_back(point3_XY1);

    geometry_msgs::msg::Pose point4_XY1 = point3_XY1;
    point4_XY1.position.y -= 0.1;
    waypoints_set7.push_back(point4_XY1);

    geometry_msgs::msg::Pose point5_XY1 = point4_XY1;
    point5_XY1.position.x -= 0.1;
    waypoints_set7.push_back(point5_XY1);

    trajectory_sets.push_back(waypoints_set7);


    // ###############  PLANE YZ ###############

    // Set 8: Plane YZ move (0.1m steps)
    std::vector<geometry_msgs::msg::Pose> waypoints_set8;

    geometry_msgs::msg::Pose point2_YZ1 = point5_XY1;
    point2_YZ1.position.z += 0.1;
    waypoints_set8.push_back(point2_YZ1);

    geometry_msgs::msg::Pose point3_YZ1 = point2_YZ1;
    point3_YZ1.position.y += 0.1;
    waypoints_set8.push_back(point3_YZ1);

    geometry_msgs::msg::Pose point4_YZ1 = point3_YZ1;
    point4_YZ1.position.z -= 0.1;
    waypoints_set8.push_back(point4_YZ1);

    geometry_msgs::msg::Pose point5_YZ1 = point4_YZ1;
    point5_YZ1.position.y -= 0.1;
    waypoints_set8.push_back(point5_YZ1);

    trajectory_sets.push_back(waypoints_set8);


    // ###############  PLANE XZ ###############

    // Set 9: Plane XZ move (0.1m steps)
    std::vector<geometry_msgs::msg::Pose> waypoints_set9;

    geometry_msgs::msg::Pose point2_XZ1 = point5_YZ1;
    point2_XZ1.position.z += 0.1;
    waypoints_set9.push_back(point2_XZ1);

    geometry_msgs::msg::Pose point3_XZ1 = point2_XZ1;
    point3_XZ1.position.x += 0.1;
    waypoints_set9.push_back(point3_XZ1);

    geometry_msgs::msg::Pose point4_XZ1 = point3_XZ1;
    point4_XZ1.position.z -= 0.1;
    waypoints_set9.push_back(point4_XZ1);

    geometry_msgs::msg::Pose point5_XZ1 = point4_XZ1;
    point5_XZ1.position.x -= 0.1;
    waypoints_set9.push_back(point5_XZ1);

    trajectory_sets.push_back(waypoints_set9);


    // ##############################  PLANE 20 ##############################

    // ###############  PLANE XY 20 ###############

    // Set 10: Plane XY move (0.2m steps)
    std::vector<geometry_msgs::msg::Pose> waypoints_set10;

    geometry_msgs::msg::Pose point2_XY2 = point5_XZ1;
    point2_XY2.position.y += 0.2;
    waypoints_set10.push_back(point2_XY2);

    geometry_msgs::msg::Pose point3_XY2 = point2_XY2;
    point3_XY2.position.x += 0.2;
    waypoints_set10.push_back(point3_XY2);

    geometry_msgs::msg::Pose point4_XY2 = point3_XY2;
    point4_XY2.position.y -= 0.2;
    waypoints_set10.push_back(point4_XY2);

    geometry_msgs::msg::Pose point5_XY2 = point4_XY2;
    point5_XY2.position.x -= 0.2;
    waypoints_set10.push_back(point5_XY2);

    trajectory_sets.push_back(waypoints_set10);


    // ###############  PLANE YZ 20 ###############

    // Set 11: Plane YZ move (0.2m steps)
    std::vector<geometry_msgs::msg::Pose> waypoints_set11;

    geometry_msgs::msg::Pose point2_YZ2 = point5_XY2;
    point2_YZ2.position.z += 0.2;
    waypoints_set11.push_back(point2_YZ2);

    geometry_msgs::msg::Pose point3_YZ2 = point2_YZ2;
    point3_YZ2.position.y += 0.2;
    waypoints_set11.push_back(point3_YZ2);

    geometry_msgs::msg::Pose point4_YZ2 = point3_YZ2;
    point4_YZ2.position.z -= 0.2;
    waypoints_set11.push_back(point4_YZ2);

    geometry_msgs::msg::Pose point5_YZ2 = point4_YZ2;
    point5_YZ2.position.y -= 0.2;
    waypoints_set11.push_back(point5_YZ2);

    trajectory_sets.push_back(waypoints_set11);


    // ###############  PLANE XZ 20 ###############

    // Set 12: Plane XZ move (0.2m steps)
    std::vector<geometry_msgs::msg::Pose> waypoints_set12;

    geometry_msgs::msg::Pose point2_XZ2 = point5_YZ2;
    point2_XZ2.position.z += 0.2;
    waypoints_set12.push_back(point2_XZ2);

    geometry_msgs::msg::Pose point3_XZ2 = point2_XZ2;
    point3_XZ2.position.x += 0.2;
    waypoints_set12.push_back(point3_XZ2);

    geometry_msgs::msg::Pose point4_XZ2 = point3_XZ2;
    point4_XZ2.position.z -= 0.2;
    waypoints_set12.push_back(point4_XZ2);

    geometry_msgs::msg::Pose point5_XZ2 = point4_XZ2;
    point5_XZ2.position.x -= 0.2;
    waypoints_set12.push_back(point5_XZ2);

    trajectory_sets.push_back(waypoints_set12);






    // Store average speeds for each trajectory set
    std::vector<double> avg_speeds;

    for (size_t i = 0; i < trajectory_sets.size(); ++i)
    {
        RCLCPP_INFO(logger, "Executing trajectory set %zu", i + 1);
        double avg_speed = execute_waypoints(move_group_interface, moveit_visual_tools, trajectory_sets[i], logger);
        avg_speeds.push_back(avg_speed);

        if (i < trajectory_sets.size() - 1) // No need to wait after the last set
        {
            RCLCPP_INFO(logger, "Waiting 5 seconds before executing the next trajectory set...");
            std::this_thread::sleep_for(std::chrono::nanoseconds(500000000));
        }
    }

    // Print the final average speed results
    for (size_t i = 0; i < avg_speeds.size(); ++i)
    {
        RCLCPP_INFO(logger, "Final average speed for trajectory set %zu: %.3f rad/s", i + 1, avg_speeds[i]);
    }


    // Shut down ROS 2 cleanly when we're done
    spinner.join();
    rclcpp::shutdown();
    return 0;
}





//  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


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



// #include <memory>
// #include <thread>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// int main(int argc, char* argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//       "hello_moveit",
//       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   // We spin up a SingleThreadedExecutor so MoveItVisualTools can interact with ROS
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "manipulator");

//   // Set the planning pipeline to Pilz
//   move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
//   move_group_interface.setPlannerId("LIN"); // Set to 'PTP' for point-to-point movement
//   move_group_interface.setPlanningTime(20.0);
//   move_group_interface.setMaxVelocityScalingFactor(0.5);
//   move_group_interface.setMaxAccelerationScalingFactor(0.5);

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC};
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   RCLCPP_INFO(logger, "Planning pipeline: %s", move_group_interface.getPlanningPipelineId().c_str());
//   RCLCPP_INFO(logger, "Planner ID: %s", move_group_interface.getPlannerId().c_str());
//   RCLCPP_INFO(logger, "Planning time: %.2f", move_group_interface.getPlanningTime());

//   // Wait for the current robot state to be available
//   rclcpp::sleep_for(std::chrono::seconds(5));

//   // Define target waypoints for movement
//   std::vector<geometry_msgs::msg::Pose> waypoints;

//   // Initial point (bottom-left corner of the square)
//   geometry_msgs::msg::Pose point1;
//   point1.position.x = 0.5;
//   point1.position.y = 0.1;
//   point1.position.z = 1.25;
//   point1.orientation.x = 0.0;
//   point1.orientation.y = 0.0;
//   point1.orientation.z = 0.0;
//   point1.orientation.w = 1.0;
//   waypoints.push_back(point1);

//   // Other waypoints to form a square
//   geometry_msgs::msg::Pose point2 = point1;
//   point2.position.x += 0.2;
//   waypoints.push_back(point2);

//   geometry_msgs::msg::Pose point3 = point2;
//   point3.position.y += 0.2;
//   waypoints.push_back(point3);

//   geometry_msgs::msg::Pose point4 = point3;
//   point4.position.x -= 0.2;
//   waypoints.push_back(point4);

//   // Return to the starting point
//   waypoints.push_back(point1);

//   // Execute the waypoints
//   for (size_t i = 0; i < waypoints.size(); ++i)
//   {
//     move_group_interface.setPoseTarget(waypoints[i]);

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     auto const ok = move_group_interface.plan(plan);

//     if (ok == moveit::core::MoveItErrorCode::SUCCESS)
//     {
//       RCLCPP_INFO(logger, "Executing move to waypoint %zu", i + 1);
//       move_group_interface.execute(plan);
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
//   spinner.join();
//   rclcpp::shutdown();
//   return 0;
// }











// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// ta je delala



// #include <memory>
// #include <thread>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// int main(int argc, char* argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//       "hello_moveit",
//       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   // We spin up a SingleThreadedExecutor so MoveItVisualTools can interact with ROS
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "manipulator");

//   // Set the planning pipeline to Pilz
//   move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
//   move_group_interface.setPlannerId("LIN"); // Set to 'PTP' for point-to-point movement
//   move_group_interface.setPlanningTime(20.0);
//   move_group_interface.setMaxVelocityScalingFactor(0.5); // sprememba hitrosti
//   move_group_interface.setMaxAccelerationScalingFactor(0.5);

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC};
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   RCLCPP_INFO(logger, "Planning pipeline: %s", move_group_interface.getPlanningPipelineId().c_str());
//   RCLCPP_INFO(logger, "Planner ID: %s", move_group_interface.getPlannerId().c_str());
//   RCLCPP_INFO(logger, "Planning time: %.2f", move_group_interface.getPlanningTime());

//   // Wait for the current robot state to be available
//   rclcpp::sleep_for(std::chrono::seconds(5));

//   // Define target waypoints for movement
//   std::vector<geometry_msgs::msg::Pose> waypoints;

//   // Initial point (bottom-left corner of the square)
//   geometry_msgs::msg::Pose point1;
//   point1.position.x = 0.5;
//   point1.position.y = 0.1;
//   point1.position.z = 1.25;
//   point1.orientation.x = 0.0;
//   point1.orientation.y = 0.0;
//   point1.orientation.z = 0.0;
//   point1.orientation.w = 1.0;
//   waypoints.push_back(point1);

//   // Other waypoints to form a square
//   geometry_msgs::msg::Pose point2 = point1;
//   point2.position.x += 0.2;
//   waypoints.push_back(point2);

//   geometry_msgs::msg::Pose point3 = point2;
//   point3.position.x -= 0.2;
//   waypoints.push_back(point3);

//   geometry_msgs::msg::Pose point4 = point3;
//   point4.position.x += 0.2;
//   waypoints.push_back(point4);

//   // Return to the starting point
//   waypoints.push_back(point1);

//   // Execute the waypoints
//   for (size_t i = 0; i < waypoints.size(); ++i)
//   {
//     move_group_interface.setPoseTarget(waypoints[i]);

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     auto const ok = move_group_interface.plan(plan);

//     if (ok == moveit::core::MoveItErrorCode::SUCCESS)
//     {
//       RCLCPP_INFO(logger, "Executing move to waypoint %zu", i + 1);
//       move_group_interface.execute(plan);

//       // Draw the path in RViz after execution
//       moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
//       moveit_visual_tools.trigger();
//     }
//     else
//     {
//       RCLCPP_ERROR(logger, "Planning failed for waypoint %zu!", i + 1);
//       break;
//     }
//   }

//   // Shut down ROS 2 cleanly when we're done
//   spinner.join();
//   rclcpp::shutdown();
//   return 0;
// }
