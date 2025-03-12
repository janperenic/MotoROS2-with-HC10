// ################################################################
// naj bi delala za vec trajektori in umes stala med njimi 5 sekund
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>  // Include for pose message
#include <fstream>  // For CSV output
#include <chrono>   // For timing
#include <atomic>  // For thread-safe flag
#include <cmath> // Include for rounding
#include <iomanip> // Include for std::fixed and std::setprecision


std::atomic<bool> logging_active(true);

double round_to_decimal(double value, int decimal_places) {
    double factor = std::pow(6, decimal_places);
    return std::round(value * factor) / factor;
}

void log_tool0_position(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    std::ofstream csv_file("MotoROS_SIM_PTP_0.75.csv");
    csv_file << "//MOTOROS_SIM_PTP_0.75;;;;;;;\n";
    csv_file << "Name;X;Y;Z;Rx;Ry;Rz;time;step\n";
    // csv_file << "time;X;Y;Z\n";
    csv_file << std::fixed << std::setprecision(4) << std::showpoint;
    // csv_file << std::fixed << std::setprecision(4); // Set fixed-point notation and precision for all values

    double time_step = 0.005; // Initialize time step
    double name = 1;

    while (logging_active) {
        auto current_pose = move_group_interface.getCurrentPose("tool0");
        auto now = node->get_clock()->now();
        csv_file << name << ";"
                 << round_to_decimal(current_pose.pose.position.x*1000, 6) << ";"
                 << round_to_decimal(current_pose.pose.position.y*1000, 6) << ";"
                 << round_to_decimal(current_pose.pose.position.z*1000, 6) << ";"
                 << ";;;"  // Placeholder for Rx, Ry, Rz (if needed)
                 << now.seconds() << ";"
                 << time_step << "\n";

        // RCLCPP_INFO(node->get_logger(), "Tool0 Position: x=%.4f, y=%.4f, z=%.4f",
        //             round_to_decimal(current_pose.pose.position.x*1000, 6),
        //             round_to_decimal(current_pose.pose.position.y*1000, 6),
        //             round_to_decimal(current_pose.pose.position.z*1000, 6));
        
        time_step += 0.005;
        name += 1;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz
    }

    csv_file.close();
}



// Function to execute a set of waypoints and calculate average speed
double execute_waypoints(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    moveit_visual_tools::MoveItVisualTools& moveit_visual_tools,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const rclcpp::Logger& logger)
{
    double total_speed = 0.0;
    size_t speed_count = 0; // Number of velocity data points
    double max_speed = 0.0;

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

                    // Update maximum speed
                    if (speed > max_speed)
                    {
                        max_speed = speed;
                    }
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
    RCLCPP_INFO(logger, "Maximum speed during the operation: %.3f rad/s", max_speed);
    
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
    move_group_interface.setPlannerId("PTP"); // Set to 'PTP' for point-to-point movement
    move_group_interface.setPlanningTime(3.0);
    move_group_interface.setMaxVelocityScalingFactor(0.0375); 
    move_group_interface.setMaxAccelerationScalingFactor(1.0);


    //0.1 je 200mm/s( pri pospesku 0.2),
    //0.05 je 100mm/s in je isto kot pri motoplusu ( pospesk 1.0)
        //0.05, 0.0375, 0.025, 0.0125
    //0.2 je 400 (pri 1.0 pospesku)


    // Start logging tool0 positions in a separate thread
    std::thread logging_thread(log_tool0_position, node, std::ref(move_group_interface));


    // Construct and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    RCLCPP_INFO(logger, "Planning pipeline: %s", move_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", move_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", move_group_interface.getPlanningTime());
   
    auto current_pose = move_group_interface.getCurrentPose("tool0");  // Get current pose of tool0
    // RCLCPP_INFO(logger, "Tool0 Position: x=%.3f, y=%.3f, z=%.3f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);



    // Wait for the current robot state to be available
    rclcpp::sleep_for(std::chrono::seconds(2));
    // Define a base pose to use as a reference for all movements
    geometry_msgs::msg::Pose base_pose;
    base_pose.position.x = 1.175; // pri meritvah mormo mi dt 1080 ker 95 je se od T maca
    base_pose.position.y = 0.0;
    base_pose.position.z = 0.24; // spremeni une baze da use stima!
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
    point2_YZ1.position.y += 0.1;
    waypoints_set8.push_back(point2_YZ1);

    geometry_msgs::msg::Pose point3_YZ1 = point2_YZ1;
    point3_YZ1.position.z += 0.1;
    waypoints_set8.push_back(point3_YZ1);

    geometry_msgs::msg::Pose point4_YZ1 = point3_YZ1;
    point4_YZ1.position.y -= 0.1;
    waypoints_set8.push_back(point4_YZ1);

    geometry_msgs::msg::Pose point5_YZ1 = point4_YZ1;
    point5_YZ1.position.z -= 0.1;
    waypoints_set8.push_back(point5_YZ1);

    trajectory_sets.push_back(waypoints_set8);


    // ###############  PLANE XZ ###############

    // Set 9: Plane XZ move (0.1m steps)
    std::vector<geometry_msgs::msg::Pose> waypoints_set9;

    geometry_msgs::msg::Pose point2_XZ1 = point5_YZ1;
    point2_XZ1.position.x += 0.1;
    waypoints_set9.push_back(point2_XZ1);

    geometry_msgs::msg::Pose point3_XZ1 = point2_XZ1;
    point3_XZ1.position.z += 0.1;
    waypoints_set9.push_back(point3_XZ1);

    geometry_msgs::msg::Pose point4_XZ1 = point3_XZ1;
    point4_XZ1.position.x -= 0.1;
    waypoints_set9.push_back(point4_XZ1);

    geometry_msgs::msg::Pose point5_XZ1 = point4_XZ1;
    point5_XZ1.position.z -= 0.1;
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
    point2_YZ2.position.y += 0.2;
    waypoints_set11.push_back(point2_YZ2);

    geometry_msgs::msg::Pose point3_YZ2 = point2_YZ2;
    point3_YZ2.position.z += 0.2;
    waypoints_set11.push_back(point3_YZ2);

    geometry_msgs::msg::Pose point4_YZ2 = point3_YZ2;
    point4_YZ2.position.y -= 0.2;
    waypoints_set11.push_back(point4_YZ2);

    geometry_msgs::msg::Pose point5_YZ2 = point4_YZ2;
    point5_YZ2.position.z -= 0.2;
    waypoints_set11.push_back(point5_YZ2);

    trajectory_sets.push_back(waypoints_set11);


    // ###############  PLANE XZ 20 ###############

    // Set 12: Plane XZ move (0.2m steps)
    std::vector<geometry_msgs::msg::Pose> waypoints_set12;

    geometry_msgs::msg::Pose point2_XZ2 = point5_YZ2;
    point2_XZ2.position.x += 0.2;
    waypoints_set12.push_back(point2_XZ2);

    geometry_msgs::msg::Pose point3_XZ2 = point2_XZ2;
    point3_XZ2.position.z += 0.2;
    waypoints_set12.push_back(point3_XZ2);

    geometry_msgs::msg::Pose point4_XZ2 = point3_XZ2;
    point4_XZ2.position.x -= 0.2;
    waypoints_set12.push_back(point4_XZ2);

    geometry_msgs::msg::Pose point5_XZ2 = point4_XZ2;
    point5_XZ2.position.z -= 0.2;
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
            std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        }
        
    }

    // Print the final average speed results
    for (size_t i = 0; i < avg_speeds.size(); ++i)
    {
        RCLCPP_INFO(logger, "Final average speed for trajectory set %zu: %.3f rad/s", i + 1, avg_speeds[i]);
    }

    logging_active = false;  // Stop logging when movements are done
    logging_thread.join();   // Wait for logging to finish

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
