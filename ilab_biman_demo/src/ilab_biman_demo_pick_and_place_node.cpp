#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ilab_biman_demo_pick_and_place");

void move_to_state(moveit::planning_interface::MoveGroupInterface& move_group, std::string group_state) {
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;

  move_group.setNamedTarget(group_state);

  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute the plan
  if (success) move_group.execute(plan);
  else RCLCPP_ERROR(LOGGER, "Planning failed!");
}


void move(moveit::planning_interface::MoveGroupInterface& move_group, std::string planning_group, std::vector<geometry_msgs::msg::Pose> waypoints) {
  moveit_msgs::msg::RobotTrajectory plan_trajectory, exec_trajectory;
  moveit_msgs::msg::RobotState robot_state;
  const double jump_threshold = 0.0;
  const double eef_step = 0.002;
  move_group.setPoseReferenceFrame(move_group.getPlanningFrame());
  RCLCPP_INFO(LOGGER, "pose reference frame set to %s", move_group.getPoseReferenceFrame().c_str());

  trajectory_processing::TimeOptimalTrajectoryGeneration traj_processor;
  robot_trajectory::RobotTrajectory traj(move_group.getRobotModel(), planning_group);
  robot_trajectory::RobotTrajectory traj_plan(move_group.getRobotModel(), planning_group);

  auto start_state = *move_group.getCurrentState();

  double fraction = move_group.computeCartesianPath(
    waypoints, eef_step, jump_threshold,
    plan_trajectory);
  RCLCPP_INFO(LOGGER, "Cartesian path : %.2f%% achieved", fraction * 100.0);
  traj.setRobotTrajectoryMsg(start_state, plan_trajectory);

  traj_processor.computeTimeStamps(traj, 0.2, 0.1);
  for (auto i = 0ul; i < traj.getWayPointDurations().size(); i++) {
    // std::cout<< traj.getWayPointDurationFromPrevious(i) <<std::endl;
    if (traj.getWayPointDurationFromPrevious(i) == 0) {
      traj.setWayPointDurationFromPrevious(i, 0.0001);
    }
  }
  traj.getRobotTrajectoryMsg(exec_trajectory);
  move_group.execute(exec_trajectory);

}

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("ilab_biman_demo_pick_and_place", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto ur_group = MoveGroupInterface(move_group_node, "ur_manipulator");
  auto franka_group = MoveGroupInterface(move_group_node, "franka_arm");
  auto franka_hand_group = MoveGroupInterface(move_group_node, "hand");
  auto ur_gripper_group = MoveGroupInterface(move_group_node, "gripper");


  std::vector<geometry_msgs::msg::Pose> ur_waypoints, franka_waypoints;

  //-----------
  // auto ur_pose_init = ur_group.getCurrentPose("tool0").pose;
  // ur_pose_init.position.z -= 0.2;
  // ur_waypoints.push_back(ur_pose_init);
  // move(ur_group, "ur_manipulator", ur_waypoints);
  // ur_waypoints.clear();

  // move_to_state(ur_gripper_group, "open");

  while(rclcpp::ok())
  {
  
  move_to_state(ur_gripper_group, "close");

  
  auto ur_pose = ur_group.getCurrentPose("tool0").pose;
  ur_pose.position.z += 0.2;
  ur_waypoints.push_back(ur_pose);
  move(ur_group, "ur_manipulator", ur_waypoints);
  ur_waypoints.clear();

  // // ----------
  auto franka_pose = franka_group.getCurrentPose("franka_link8").pose;
  franka_pose.position.z -= 0.0;
  franka_waypoints.push_back(franka_pose);
  move(franka_group, "franka_arm", franka_waypoints);
  franka_waypoints.clear();

  move_to_state(franka_hand_group, "close");
  move_to_state(ur_gripper_group, "open");

  franka_pose = franka_group.getCurrentPose("franka_link8").pose;
  franka_pose.position.z += 0.1;
  franka_waypoints.push_back(franka_pose);
  franka_pose.position.z -= 0.1;
  franka_waypoints.push_back(franka_pose);
  move(franka_group, "franka_arm", franka_waypoints);
  franka_waypoints.clear();

  move_to_state(ur_gripper_group, "close");
  move_to_state(franka_hand_group, "open");
  //--------

  ur_pose.position.z -= 0.2;
  ur_waypoints.push_back(ur_pose);
  move(ur_group, "ur_manipulator", ur_waypoints);
  ur_waypoints.clear();

  move_to_state(ur_gripper_group, "open");

  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}