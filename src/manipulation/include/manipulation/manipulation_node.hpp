#pragma once

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "workflow_interfaces/msg/bin_target.hpp"
#include "workflow_interfaces/srv/execute_stage.hpp"

namespace manipulation
{

class ManipulationNode : public rclcpp::Node
{
public:
  ManipulationNode();
  void initialize();

private:
  void declareParameters();
  void configureMoveGroups();
  void seedPlanningScene();
  void binTargetCallback(const workflow_interfaces::msg::BinTarget::SharedPtr msg);
  void executeStageCallback(
    const std::shared_ptr<workflow_interfaces::srv::ExecuteStage::Request> request,
    std::shared_ptr<workflow_interfaces::srv::ExecuteStage::Response> response);

  bool executeNamedTarget(
    moveit::planning_interface::MoveGroupInterface & group,
    const std::string & target_name,
    const std::string & description);
  bool executeNamedArmTrajectory(const std::string & target_name, const std::string & description);
  bool extractNamedJointTarget(
    moveit::planning_interface::MoveGroupInterface & group,
    const std::string & target_name,
    std::vector<std::string> & joint_names,
    std::vector<double> & positions);
  bool executeJointTrajectory(
    const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr & client,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions,
    double duration_sec,
    const std::string & description);
  bool executePoseTarget(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & description);
  bool executeWorldPoseTarget(
    double x,
    double y,
    double z,
    const std::vector<double> & orientation,
    const std::string & description);
  bool executePositionTarget(double x, double y, double z, const std::string & description);
  bool executeCartesianPoseTarget(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & description);
  bool executeCartesianOffset(double dx, double dy, double dz, const std::string & description);
  bool executeVerticalMoveToHeight(double target_z, const std::string & description);
  bool attachObjectToTool();
  bool detachObjectFromTool();
  bool refreshWorldObject();
  bool placeObjectAtBin(const std::string & bin_id);
  bool isEndEffectorNearObject(
    const std::string & offset_parameter,
    double xy_tolerance,
    double z_tolerance);
  void publishStatus(
    const std::string & stage,
    const std::string & status,
    const std::string & detail);
  std::string namedPlaceTargetForBin(const std::string & bin_id) const;
  std::vector<double> parameterArray(const std::string & name) const;
  bool executeObjectOffsetTarget(const std::string & offset_parameter, const std::string & description);
  bool setObjectPoseInGazebo(const geometry_msgs::msg::Pose & pose);

  geometry_msgs::msg::Pose poseFromArray(const std::vector<double> & values) const;
  shape_msgs::msg::SolidPrimitive boxPrimitive(const std::vector<double> & dimensions) const;

  rclcpp::Service<workflow_interfaces::srv::ExecuteStage>::SharedPtr execute_stage_service_;
  rclcpp::Subscription<workflow_interfaces::msg::BinTarget>::SharedPtr bin_target_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr primary_set_entity_state_client_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr fallback_set_entity_state_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_trajectory_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper_trajectory_client_;

  workflow_interfaces::msg::BinTarget latest_bin_target_;
  bool have_bin_target_{false};
  bool object_attached_{false};
  bool pre_grasp_ready_{false};

  std::string world_frame_;
  std::string base_frame_;
  std::string end_effector_link_;
  std::string object_id_;
  std::string home_pose_name_;
  std::string pre_grasp_pose_name_;
  std::string grasp_pose_name_;
  std::string lift_pose_name_;
  std::string transit_pose_name_;
  std::string scan_pose_name_;
  std::string bin_a_pose_name_;
  std::string bin_b_pose_name_;
  std::string bin_c_pose_name_;
  std::string gripper_open_state_;
  std::string gripper_closed_state_;
  std::string status_topic_;
  std::string primary_entity_state_service_;
  std::string fallback_entity_state_service_;
  std::string arm_action_name_;
  std::string gripper_action_name_;

  int planning_attempts_{10};
  int max_plan_retries_{3};
  double planning_time_{5.0};
  double max_velocity_scaling_factor_{0.25};
  double max_acceleration_scaling_factor_{0.18};
  double cartesian_step_{0.01};
  double jump_threshold_{0.0};
  double grasp_descent_{0.04};
  double place_descent_{0.08};
  double grasp_soft_xy_tolerance_{0.09};
  double grasp_soft_z_tolerance_{0.10};
  double grasp_pre_grasp_xy_tolerance_{0.12};
  double grasp_pre_grasp_z_tolerance_{0.18};
  double named_pose_motion_duration_{3.5};
  double gripper_motion_duration_{1.0};
};

}  // namespace manipulation
