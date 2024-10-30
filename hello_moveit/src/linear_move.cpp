#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
plan_to_pose(moveit::planning_interface::MoveGroupInterface &move_group_interface, 
             const geometry_msgs::msg::Pose &target_pose, 
             moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
             rclcpp::Logger logger) {
  move_group_interface.setPoseTarget(target_pose);

  // Plan to the target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success) {
    // Visualize the plan
    moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
    moveit_visual_tools.trigger();
    return std::make_pair(true, plan);
  } else {
    RCLCPP_ERROR(logger, "Planning to the target pose failed!");
    return std::make_pair(false, moveit::planning_interface::MoveGroupInterface::Plan());  }
}

void execute_plan(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                  const moveit::planning_interface::MoveGroupInterface::Plan &plan,
                  moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
                  rclcpp::Logger logger) {
  moveit_visual_tools.trigger();
  if (!move_group_interface.execute(plan)) {
    RCLCPP_ERROR(logger, "Execution failed!");
  } else {
    moveit_visual_tools.trigger();
  }
}

int main(int argc, char * argv[]) {
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
      msg.translation().z() = 1.5;  // Place text 1m above the base link
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

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;  // Example values
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.5; //rdeca
    msg.position.y = 0.15; //zelena
    msg.position.z = 0.9;
    return msg;
  }();

  // Define the second target pose
  auto const second_target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;  // Example values
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.6;
    msg.position.y = 0.15;
    msg.position.z = 0.9;
    return msg;
  }();

  // Define the third target pose
  auto const third_target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;  // Example values
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.6;  // Adjusting x position
    msg.position.y = 0.05;
    msg.position.z = 0.6;
    return msg;
  }();

  // Define the fourth target pose
  auto const fourth_target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;  // Example values
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.5;  // Moving x position further
    msg.position.y = 0.05;
    msg.position.z = 0.6;
    return msg;
  }();

  // Define the fifth target pose
  auto const fifth_target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;  // Example values
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.5;  // Return to original x position
    msg.position.y = 0.15;
    msg.position.z = 0.6;
    return msg;
  }();

  // Plan and execute to the first pose
  draw_title("Planning to first pose"); 
  moveit_visual_tools.trigger();  // Trigger the visual tool to display the title
  auto [success, plan] = plan_to_pose(move_group_interface, target_pose, moveit_visual_tools, logger);
  if (success) {
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing first pose");  // Add this line before execution
      moveit_visual_tools.trigger();
      execute_plan(move_group_interface, plan, moveit_visual_tools, logger);
  }


  // Plan and execute to the second pose
  auto [success2, plan2] = plan_to_pose(move_group_interface, second_target_pose, moveit_visual_tools, logger);
  if (success2) {
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute second pose");
    execute_plan(move_group_interface, plan2, moveit_visual_tools, logger);
  }

  // Plan and execute to the third pose
  auto [success3, plan3] = plan_to_pose(move_group_interface, third_target_pose, moveit_visual_tools, logger);
  if (success3) {
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute third pose");
    execute_plan(move_group_interface, plan3, moveit_visual_tools, logger);
  }

  // Plan and execute to the fourth pose
  auto [success4, plan4] = plan_to_pose(move_group_interface, fourth_target_pose, moveit_visual_tools, logger);
  if (success4) {
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute fourth pose");
    execute_plan(move_group_interface, plan4, moveit_visual_tools, logger);
  }

  // Plan and execute to the fifth pose
  auto [success5, plan5] = plan_to_pose(move_group_interface, fifth_target_pose, moveit_visual_tools, logger);
  if (success5) {
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute fifth pose");
    execute_plan(move_group_interface, plan5, moveit_visual_tools, logger);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join(); 
  return 0;
}
