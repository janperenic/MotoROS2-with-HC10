#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask() {
  mtc::Task task;
  task.stages()->setName("square_motion_task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "manipulator";//panda_arm!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  // task.setProperty("ik_frame", hand_frame);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // Define the motion to create a square pattern
  // Move 20cm along X-axis
  auto move_20cm_x = std::make_unique<mtc::stages::MoveRelative>("Move 20cm along X", cartesian_planner);
  move_20cm_x->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  move_20cm_x->setMinMaxDistance(0.2, 0.2);
  geometry_msgs::msg::Vector3Stamped direction;
  direction.header.frame_id = "base_link";//world!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  direction.vector.x = 1.0;
  move_20cm_x->setDirection(direction);
  task.add(std::move(move_20cm_x));

  // Move 20cm along Y-axis
  auto move_20cm_y = std::make_unique<mtc::stages::MoveRelative>("Move 20cm along Y", cartesian_planner);
  move_20cm_y->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  move_20cm_y->setMinMaxDistance(0.2, 0.2);
  direction.vector.x = 0.0;  // Reset X movement
  direction.vector.y = 1.0;  // Move along Y-axis
  move_20cm_y->setDirection(direction);
  task.add(std::move(move_20cm_y));

  // Move back 20cm along X-axis (negative direction)
  auto move_back_20cm_x = std::make_unique<mtc::stages::MoveRelative>("Move back 20cm along X", cartesian_planner);
  move_back_20cm_x->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  move_back_20cm_x->setMinMaxDistance(0.2, 0.2);
  direction.vector.y = 0.0;  // Reset Y movement
  direction.vector.x = -1.0;  // Move back along X-axis
  move_back_20cm_x->setDirection(direction);
  task.add(std::move(move_back_20cm_x));

  // Move back 20cm along Y-axis (negative direction)
  auto move_back_20cm_y = std::make_unique<mtc::stages::MoveRelative>("Move back 20cm along Y", cartesian_planner);
  move_back_20cm_y->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  move_back_20cm_y->setMinMaxDistance(0.2, 0.2);
  direction.vector.x = 0.0;  // Reset X movement
  direction.vector.y = -1.0;  // Move back along Y-axis
  move_back_20cm_y->setDirection(direction);
  task.add(std::move(move_back_20cm_y));

  return task;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}