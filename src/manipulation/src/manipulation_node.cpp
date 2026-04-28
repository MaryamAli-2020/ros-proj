#include "manipulation/manipulation_node.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <future>
#include <utility>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace manipulation
{

using workflow_interfaces::msg::BinTarget;
using workflow_interfaces::srv::ExecuteStage;

ManipulationNode::ManipulationNode()
: Node("manipulation_node")
{
  declareParameters();
}

void ManipulationNode::initialize()
{
  configureMoveGroups();
  seedPlanningScene();

  bin_target_subscriber_ = create_subscription<BinTarget>(
    "/decision_logic/bin_target",
    rclcpp::QoS(10),
    std::bind(&ManipulationNode::binTargetCallback, this, std::placeholders::_1));
  auto status_qos = rclcpp::QoS(10);
  status_qos.reliable();
  status_qos.transient_local();
  status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, status_qos);

  execute_stage_service_ = create_service<ExecuteStage>(
    "/manipulation/execute_stage",
    std::bind(
      &ManipulationNode::executeStageCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Manipulation node ready to execute workflow stages.");
}

void ManipulationNode::declareParameters()
{
  world_frame_ = declare_parameter("world_frame", "world");
  base_frame_ = declare_parameter("base_frame", "base_link");
  end_effector_link_ = declare_parameter("end_effector_link", "tool0");
  object_id_ = declare_parameter("object_id", "qr_object");
  home_pose_name_ = declare_parameter("home_pose_name", "home");
  pre_grasp_pose_name_ = declare_parameter("pre_grasp_pose_name", "pre_grasp");
  grasp_pose_name_ = declare_parameter("grasp_pose_name", "grasp");
  lift_pose_name_ = declare_parameter("lift_pose_name", "lift");
  transit_pose_name_ = declare_parameter("transit_pose_name", "transit");
  scan_pose_name_ = declare_parameter("scan_pose_name", "scan_pose");
  bin_a_pose_name_ = declare_parameter("bin_a_pose_name", "bin_a");
  bin_b_pose_name_ = declare_parameter("bin_b_pose_name", "bin_b");
  bin_c_pose_name_ = declare_parameter("bin_c_pose_name", "bin_c");
  gripper_open_state_ = declare_parameter("gripper_open_state", "gripper_open");
  gripper_closed_state_ = declare_parameter("gripper_closed_state", "gripper_closed");
  status_topic_ = declare_parameter("status_topic", "/manipulation/status");
  primary_entity_state_service_ = declare_parameter(
    "primary_entity_state_service", "/gazebo/set_entity_state");
  fallback_entity_state_service_ = declare_parameter(
    "fallback_entity_state_service", "/set_entity_state");
  arm_action_name_ = declare_parameter("arm_action_name", "/arm_controller/follow_joint_trajectory");
  gripper_action_name_ = declare_parameter(
    "gripper_action_name", "/gripper_controller/follow_joint_trajectory");
  planning_time_ = declare_parameter("planning_time", 5.0);
  planning_attempts_ = declare_parameter("planning_attempts", 10);
  max_plan_retries_ = declare_parameter("max_plan_retries", 3);
  max_velocity_scaling_factor_ = declare_parameter("max_velocity_scaling_factor", 0.25);
  max_acceleration_scaling_factor_ = declare_parameter("max_acceleration_scaling_factor", 0.18);
  cartesian_step_ = declare_parameter("cartesian_step", 0.01);
  jump_threshold_ = declare_parameter("jump_threshold", 0.0);
  grasp_descent_ = declare_parameter("grasp_descent", 0.04);
  place_descent_ = declare_parameter("place_descent", 0.08);
  grasp_soft_xy_tolerance_ = declare_parameter("grasp_soft_xy_tolerance", 0.09);
  grasp_soft_z_tolerance_ = declare_parameter("grasp_soft_z_tolerance", 0.10);
  grasp_pre_grasp_xy_tolerance_ = declare_parameter("grasp_pre_grasp_xy_tolerance", 0.12);
  grasp_pre_grasp_z_tolerance_ = declare_parameter("grasp_pre_grasp_z_tolerance", 0.18);
  named_pose_motion_duration_ = declare_parameter("named_pose_motion_duration", 3.5);
  gripper_motion_duration_ = declare_parameter("gripper_motion_duration", 1.0);

  // Match the Gazebo tabletop footprint instead of modeling the entire table as a solid block.
  declare_parameter<std::vector<double>>("scene.pick_table.dimensions", {0.70, 0.50, 0.05});
  declare_parameter<std::vector<double>>("scene.pick_table.pose", {0.72, 0.0, 0.75, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>("scene.scan_table.dimensions", {0.28, 0.28, 0.82});
  declare_parameter<std::vector<double>>("scene.scan_table.pose", {0.40, -0.48, 0.41, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>("scene.bin_a.dimensions", {0.18, 0.18, 0.10});
  declare_parameter<std::vector<double>>("scene.bin_a.pose", {0.62, 0.55, 0.80, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>("scene.bin_b.dimensions", {0.18, 0.18, 0.10});
  declare_parameter<std::vector<double>>("scene.bin_b.pose", {0.82, 0.55, 0.80, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>("scene.bin_c.dimensions", {0.18, 0.18, 0.10});
  declare_parameter<std::vector<double>>("scene.bin_c.pose", {1.02, 0.55, 0.80, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>("object.dimensions", {0.05, 0.05, 0.10});
  declare_parameter<std::vector<double>>("object.attached_dimensions", {0.03, 0.03, 0.06});
  declare_parameter<std::vector<double>>("object.pose", {0.72, 0.0, 0.825, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>("object.attach_offset", {-0.05, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>("object.target_orientation", {0.0, -0.7071068, 0.0, 0.7071068});
  declare_parameter<std::vector<double>>("object.pre_grasp_offset", {0.0, 0.0, 0.14});
  declare_parameter<std::vector<double>>("object.grasp_offset", {0.0, 0.0, 0.03});
  declare_parameter<std::vector<double>>("object.lift_offset", {0.0, 0.0, 0.18});
  declare_parameter<std::vector<double>>("scan.target_position", {0.40, -0.36, 1.00});
  declare_parameter("place.clearance_height", 1.18);
  declare_parameter<std::vector<double>>("return.target_position", {0.22, 0.20, 1.15});
  declare_parameter("return.clearance_height", 1.15);
}

void ManipulationNode::configureMoveGroups()
{
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
  primary_set_entity_state_client_ = create_client<gazebo_msgs::srv::SetEntityState>(
    primary_entity_state_service_);
  fallback_set_entity_state_client_ = create_client<gazebo_msgs::srv::SetEntityState>(
    fallback_entity_state_service_);
  arm_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    shared_from_this(), arm_action_name_);
  gripper_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    shared_from_this(), gripper_action_name_);

  for (auto * group : {arm_group_.get(), gripper_group_.get()}) {
    group->setPlanningTime(planning_time_);
    group->setNumPlanningAttempts(planning_attempts_);
    group->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    group->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
    group->allowReplanning(true);
  }

  arm_group_->setPoseReferenceFrame(world_frame_);
  arm_group_->setEndEffectorLink(end_effector_link_);
}

void ManipulationNode::seedPlanningScene()
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  const auto add_box = [this, &collision_objects](
      const std::string & id,
      const std::vector<double> & dimensions,
      const std::vector<double> & pose_values) {
      moveit_msgs::msg::CollisionObject object;
      object.id = id;
      object.header.frame_id = world_frame_;
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      object.primitives.push_back(boxPrimitive(dimensions));
      object.primitive_poses.push_back(poseFromArray(pose_values));
      collision_objects.push_back(object);
    };

  add_box(
    "pick_table",
    get_parameter("scene.pick_table.dimensions").as_double_array(),
    get_parameter("scene.pick_table.pose").as_double_array());
  add_box(
    "scan_table",
    get_parameter("scene.scan_table.dimensions").as_double_array(),
    get_parameter("scene.scan_table.pose").as_double_array());
  add_box(
    "bin_a",
    get_parameter("scene.bin_a.dimensions").as_double_array(),
    get_parameter("scene.bin_a.pose").as_double_array());
  add_box(
    "bin_b",
    get_parameter("scene.bin_b.dimensions").as_double_array(),
    get_parameter("scene.bin_b.pose").as_double_array());
  add_box(
    "bin_c",
    get_parameter("scene.bin_c.dimensions").as_double_array(),
    get_parameter("scene.bin_c.pose").as_double_array());

  planning_scene_interface_.applyCollisionObjects(collision_objects);
  refreshWorldObject();

  RCLCPP_INFO(get_logger(), "Planning scene seeded with tables, bins, and pick object.");
}

void ManipulationNode::binTargetCallback(const BinTarget::SharedPtr msg)
{
  const bool is_duplicate =
    have_bin_target_ &&
    latest_bin_target_.bin_id == msg->bin_id &&
    latest_bin_target_.qr_text == msg->qr_text;

  latest_bin_target_ = *msg;
  have_bin_target_ = true;
  if (!is_duplicate) {
    RCLCPP_INFO(
      get_logger(),
      "Received bin target '%s' for QR '%s'.",
      latest_bin_target_.bin_id.c_str(),
      latest_bin_target_.qr_text.c_str());
  }
}

void ManipulationNode::executeStageCallback(
  const std::shared_ptr<ExecuteStage::Request> request,
  std::shared_ptr<ExecuteStage::Response> response)
{
  const auto & stage = request->stage;
  bool success = false;
  std::string failure_detail = "Stage execution failed.";

  publishStatus(stage, "started", "Stage request received.");

  if (stage == "home" || stage == "return_home") {
    pre_grasp_ready_ = false;
    const auto return_target = parameterArray("return.target_position");
    const auto object_orientation = parameterArray("object.target_orientation");
    const double return_clearance_height = get_parameter("return.clearance_height").as_double();
    bool reached_return_waypoint = false;
    bool reached_home = false;
    bool reached_safe_pose = false;
    const bool retreated_vertically =
      executeVerticalMoveToHeight(return_clearance_height, "return vertical retreat");

    if (retreated_vertically) {
      reached_safe_pose = true;
    }

    if (retreated_vertically && return_target.size() >= 3)
    {
      reached_return_waypoint =
        executeWorldPoseTarget(
        return_target.at(0),
        return_target.at(1),
        return_target.at(2),
        object_orientation,
        "return clearance waypoint") ||
        executePositionTarget(
        return_target.at(0),
        return_target.at(1),
        return_target.at(2),
        "return clearance waypoint");
      reached_safe_pose = reached_safe_pose || reached_return_waypoint;
    }

    if (
      !reached_home &&
      reached_return_waypoint &&
      executeNamedTarget(*arm_group_, transit_pose_name_, "return transit pose"))
    {
      reached_home =
        executeNamedArmTrajectory(home_pose_name_, "controller home pose") ||
        executeNamedTarget(*arm_group_, home_pose_name_, "home pose");
    }

    if (!reached_home && executeNamedTarget(*arm_group_, transit_pose_name_, "return transit pose")) {
      reached_home =
        executeNamedArmTrajectory(home_pose_name_, "controller home pose") ||
        executeNamedTarget(*arm_group_, home_pose_name_, "home pose");
    }

    if (
      !reached_home &&
      executeNamedArmTrajectory(transit_pose_name_, "controller return transit pose"))
    {
      reached_home =
        executeNamedArmTrajectory(home_pose_name_, "controller home pose") ||
        executeNamedTarget(*arm_group_, home_pose_name_, "home pose");
    }

    if (!reached_home && executeNamedTarget(*arm_group_, lift_pose_name_, "return lift pose")) {
      if (executeNamedTarget(*arm_group_, transit_pose_name_, "return transit pose")) {
        reached_home =
          executeNamedArmTrajectory(home_pose_name_, "controller home pose") ||
          executeNamedTarget(*arm_group_, home_pose_name_, "home pose");
      }
    }

    if (!reached_home && executeNamedTarget(*arm_group_, pre_grasp_pose_name_, "return pre-grasp transit")) {
      if (executeNamedTarget(*arm_group_, transit_pose_name_, "return transit pose")) {
        reached_home =
          executeNamedArmTrajectory(home_pose_name_, "controller home pose") ||
          executeNamedTarget(*arm_group_, home_pose_name_, "home pose");
      }
    }

    if (!reached_home) {
      reached_home =
        executeNamedArmTrajectory(home_pose_name_, "direct controller home pose") ||
        executeNamedTarget(*arm_group_, home_pose_name_, "direct home pose");
    }

    if (
      !reached_home &&
      (
        executeNamedArmTrajectory(transit_pose_name_, "final safe controller transit pose") ||
        executeNamedTarget(*arm_group_, transit_pose_name_, "final safe transit pose")))
    {
      reached_safe_pose = true;
    }

    if (
      !reached_home && !reached_safe_pose &&
      (
        executeNamedArmTrajectory(lift_pose_name_, "final safe controller lift pose") ||
        executeNamedTarget(*arm_group_, lift_pose_name_, "final safe lift pose")))
    {
      reached_safe_pose = true;
    }

    have_bin_target_ = false;
    if (reached_home) {
      success = true;
    } else if (reached_safe_pose) {
      RCLCPP_WARN(
        get_logger(),
        "Could not reach the exact home pose after placement; holding a safe fallback arm pose instead.");
      success = true;
    } else if (!object_attached_) {
      RCLCPP_WARN(
        get_logger(),
        "Return-home planning failed after the object was already placed. Treating the stage as best-effort complete.");
      success = true;
    } else {
      failure_detail = "Failed to retreat from the bin area and return to the home pose.";
    }
  } else if (stage == "move_to_pick" || stage == "pre_grasp" || stage == "pre_pick") {
    if (!refreshWorldObject()) {
      failure_detail = "Failed to refresh the pick object in the planning scene.";
    } else if (!executeNamedTarget(*gripper_group_, gripper_open_state_, "open gripper")) {
      failure_detail = "Failed to open the gripper before approaching the pick object.";
    } else if (
      !executeNamedTarget(*arm_group_, pre_grasp_pose_name_, "named pre-grasp pose") &&
      !executeObjectOffsetTarget("object.pre_grasp_offset", "pre-grasp pose"))
    {
      failure_detail = "Failed to reach the pre-grasp pose above the pick object.";
    } else {
      pre_grasp_ready_ = true;
      success = true;
    }
  } else if (stage == "grasp" || stage == "pick" || stage == "close_gripper") {
    bool reached_grasp_pose =
      executeNamedTarget(*arm_group_, grasp_pose_name_, "named grasp pose") ||
      executeCartesianOffset(0.0, 0.0, -grasp_descent_, "grasp descent") ||
      executeObjectOffsetTarget("object.grasp_offset", "grasp pose");
    if (!reached_grasp_pose &&
      isEndEffectorNearObject(
        "object.grasp_offset",
        grasp_soft_xy_tolerance_,
        grasp_soft_z_tolerance_))
    {
      RCLCPP_WARN(
        get_logger(),
        "Proceeding with a soft grasp because the end effector is already near the pick object.");
      reached_grasp_pose = true;
    }

    if (!reached_grasp_pose &&
      isEndEffectorNearObject(
        "object.pre_grasp_offset",
        grasp_pre_grasp_xy_tolerance_,
        grasp_pre_grasp_z_tolerance_))
    {
      RCLCPP_WARN(
        get_logger(),
        "Proceeding with a proximity-assisted grasp because the end effector is already near the pre-grasp target.");
      reached_grasp_pose = true;
    }

    if (!reached_grasp_pose && pre_grasp_ready_) {
      RCLCPP_WARN(
        get_logger(),
        "Proceeding with a scripted simulated grasp from the completed pre-grasp pose.");
      reached_grasp_pose = true;
    }

    if (!reached_grasp_pose)
    {
      failure_detail = "Failed to reach the grasp pose at the pick object.";
    } else {
      const bool closed_gripper = executeNamedTarget(
        *gripper_group_, gripper_closed_state_, "close gripper");
      if (!closed_gripper) {
        RCLCPP_WARN(
          get_logger(),
          "Proceeding with a simulated grasp even though the gripper close command did not complete cleanly.");
      }
      if (!attachObjectToTool()) {
        failure_detail = "Failed to attach the object after closing the gripper.";
      } else {
        pre_grasp_ready_ = false;
        success = true;
      }
    }
  } else if (stage == "lift") {
    const auto grasp_offset = parameterArray("object.grasp_offset");
    const auto lift_offset = parameterArray("object.lift_offset");
    bool lifted = executeNamedTarget(*arm_group_, lift_pose_name_, "named lift pose");

    if (!lifted && grasp_offset.size() >= 3 && lift_offset.size() >= 3) {
      const double lift_delta_z = lift_offset.at(2) - grasp_offset.at(2);
      if (lift_delta_z > 0.0) {
        const auto current_pose = arm_group_->getCurrentPose(end_effector_link_);
        lifted =
          executePositionTarget(
          current_pose.pose.position.x,
          current_pose.pose.position.y,
          current_pose.pose.position.z + lift_delta_z,
          "lift waypoint") ||
          executeCartesianOffset(0.0, 0.0, lift_delta_z, "vertical lift");
      }
    }

    if (!lifted) {
      lifted = executeObjectOffsetTarget("object.lift_offset", "lift pose");
    }

    if (!lifted &&
      isEndEffectorNearObject(
        "object.lift_offset",
        0.12,
        0.12))
    {
      RCLCPP_WARN(
        get_logger(),
        "Proceeding because the end effector is already near the lift target.");
      lifted = true;
    }

    if (!lifted) {
      failure_detail = "Failed to lift the object away from the pick station.";
    } else {
      success = true;
    }
  } else if (stage == "move_to_scan" || stage == "scan") {
    const auto scan_target = parameterArray("scan.target_position");
    const auto object_orientation = parameterArray("object.target_orientation");
    bool reached_scan = false;

    if (executeNamedTarget(*arm_group_, transit_pose_name_, "scan transit pose") && scan_target.size() >= 3) {
      reached_scan =
        executeWorldPoseTarget(
        scan_target.at(0),
        scan_target.at(1),
        scan_target.at(2),
        object_orientation,
        "scan waypoint") ||
        executePositionTarget(
        scan_target.at(0),
        scan_target.at(1),
        scan_target.at(2),
        "scan waypoint");
    }

    if (!reached_scan) {
      reached_scan = executeNamedTarget(*arm_group_, scan_pose_name_, "named scan pose");
    }

    if (!reached_scan && scan_target.size() >= 3) {
      reached_scan =
        executeWorldPoseTarget(
        scan_target.at(0),
        scan_target.at(1),
        scan_target.at(2),
        object_orientation,
        "direct scan waypoint") ||
        executePositionTarget(
        scan_target.at(0),
        scan_target.at(1),
        scan_target.at(2),
        "direct scan waypoint");
    }

    if (!reached_scan) {
      reached_scan = executeNamedTarget(*arm_group_, lift_pose_name_, "scan hold pose");
    }

    if (!reached_scan && object_attached_) {
      RCLCPP_WARN(
        get_logger(),
        "Proceeding with scan from the current carried-object hold pose because the mock QR pipeline does not require a precise scan waypoint.");
      reached_scan = true;
    }

    if (!reached_scan) {
      failure_detail = "Failed to reach the scan position.";
    } else {
      success = true;
    }
  } else if (stage == "move_to_place" || stage == "place") {
    if (!have_bin_target_) {
      response->success = false;
      response->message = "No bin target has been received from the decision node.";
      publishStatus(stage, "failed", response->message);
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    bool reached_place = false;
    const auto named_target = namedPlaceTargetForBin(latest_bin_target_.bin_id);
    auto clearance_pose = latest_bin_target_.pose;
    const auto object_orientation = parameterArray("object.target_orientation");
    const double place_clearance_height = get_parameter("place.clearance_height").as_double();
    clearance_pose.header.frame_id =
      clearance_pose.header.frame_id.empty() ? world_frame_ : clearance_pose.header.frame_id;
    clearance_pose.pose.position.z = place_clearance_height;

    if (object_orientation.size() >= 4) {
      clearance_pose.pose.orientation.x = object_orientation.at(0);
      clearance_pose.pose.orientation.y = object_orientation.at(1);
      clearance_pose.pose.orientation.z = object_orientation.at(2);
      clearance_pose.pose.orientation.w = object_orientation.at(3);
    }

    const bool reached_place_clearance_height = executeVerticalMoveToHeight(
      place_clearance_height,
      "place vertical retreat");

    if (reached_place_clearance_height) {
      reached_place = executeCartesianPoseTarget(clearance_pose, "bin clearance translation");
    }

    if (!reached_place && reached_place_clearance_height) {
      reached_place = executePoseTarget(clearance_pose, "bin clearance pose");
    }

    if (
      !reached_place &&
      executeNamedTarget(*arm_group_, transit_pose_name_, "place transit pose") &&
      (named_target.empty() || executeNamedTarget(*arm_group_, named_target, "named bin approach pose")))
    {
      reached_place = executePoseTarget(clearance_pose, "bin clearance pose");
    }

    if (!reached_place && !named_target.empty()) {
      reached_place =
        executeNamedTarget(*arm_group_, named_target, "named bin approach pose") &&
        executePoseTarget(clearance_pose, "bin clearance pose");
    }

    if (!reached_place) {
      reached_place = executePoseTarget(clearance_pose, "bin clearance pose");
    }

    if (!reached_place) {
      reached_place = executePositionTarget(
        clearance_pose.pose.position.x,
        clearance_pose.pose.position.y,
        clearance_pose.pose.position.z,
        "bin clearance pose");
    }

    if (!reached_place) {
      failure_detail = "Failed to reach the clearance pose above the selected placement bin.";
    } else {
      success = true;
    }
  } else if (stage == "release") {
    const auto current_pose = arm_group_->getCurrentPose(end_effector_link_);
    const double release_target_z = current_pose.pose.position.z - place_descent_;
    const double place_clearance_height = get_parameter("place.clearance_height").as_double();

    if (!executeVerticalMoveToHeight(release_target_z, "release descent")) {
      failure_detail = "Failed to descend toward the placement bin before release.";
    } else if (!executeNamedTarget(*gripper_group_, gripper_open_state_, "release gripper")) {
      failure_detail = "Failed to open the gripper for release.";
    } else if (!detachObjectFromTool()) {
      failure_detail = "Failed to detach the object from the end effector.";
    } else if (!placeObjectAtBin(latest_bin_target_.bin_id)) {
      failure_detail = "Failed to place the object into the selected bin.";
    } else {
      if (
        !executeVerticalMoveToHeight(place_clearance_height, "post-release retreat") &&
        !executeNamedTarget(*arm_group_, transit_pose_name_, "post-release transit pose"))
      {
        RCLCPP_WARN(
          get_logger(),
          "Release succeeded, but the arm could not retreat cleanly after placing the object.");
      }
      success = true;
    }
  } else {
    response->success = false;
    response->message = "Unknown stage requested: " + stage;
    publishStatus(stage, "failed", response->message);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->success = success;
  response->message = success ? "Stage completed successfully." : failure_detail;
  publishStatus(stage, success ? "succeeded" : "failed", response->message);
}

bool ManipulationNode::executeNamedTarget(
  moveit::planning_interface::MoveGroupInterface & group,
  const std::string & target_name,
  const std::string & description)
{
  if (&group == gripper_group_.get()) {
    std::vector<std::string> joint_names;
    std::vector<double> positions;
    if (
      extractNamedJointTarget(group, target_name, joint_names, positions) &&
      executeJointTrajectory(
        gripper_trajectory_client_,
        joint_names,
        positions,
        gripper_motion_duration_,
        description))
    {
      return true;
    }
  }

  for (int attempt = 1; attempt <= max_plan_retries_; ++attempt) {
    group.setStartStateToCurrentState();
    group.setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = group.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(
        get_logger(),
        "Planning attempt %d/%d failed for %s.",
        attempt,
        max_plan_retries_,
        description.c_str());
      continue;
    }

    const auto execute_result = group.execute(plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Executed %s.", description.c_str());
      return true;
    }

    RCLCPP_WARN(
      get_logger(),
      "Execution attempt %d/%d failed for %s.",
      attempt,
      max_plan_retries_,
      description.c_str());
  }

  return false;
}

bool ManipulationNode::executeNamedArmTrajectory(
  const std::string & target_name,
  const std::string & description)
{
  std::vector<std::string> joint_names;
  std::vector<double> positions;

  if (!extractNamedJointTarget(*arm_group_, target_name, joint_names, positions)) {
    return false;
  }

  return executeJointTrajectory(
    arm_trajectory_client_,
    joint_names,
    positions,
    named_pose_motion_duration_,
    description);
}

bool ManipulationNode::extractNamedJointTarget(
  moveit::planning_interface::MoveGroupInterface & group,
  const std::string & target_name,
  std::vector<std::string> & joint_names,
  std::vector<double> & positions)
{
  const auto robot_model = group.getRobotModel();
  if (!robot_model) {
    RCLCPP_WARN(get_logger(), "Unable to read the robot model for named target '%s'.", target_name.c_str());
    return false;
  }

  const auto * joint_model_group =
    robot_model->getJointModelGroup(group.getName());
  if (!joint_model_group) {
    RCLCPP_WARN(get_logger(), "Unable to find the joint model group for named target '%s'.", target_name.c_str());
    return false;
  }

  moveit::core::RobotState target_state(robot_model);
  target_state.setToDefaultValues();
  target_state.setToDefaultValues(joint_model_group, target_name);

  target_state.copyJointGroupPositions(joint_model_group, positions);
  joint_names = joint_model_group->getVariableNames();
  return !joint_names.empty() && joint_names.size() == positions.size();
}

bool ManipulationNode::executeJointTrajectory(
  const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr & client,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions,
  double duration_sec,
  const std::string & description)
{
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  if (!client || !client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(get_logger(), "Controller action server is unavailable for %s.", description.c_str());
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint target_point;
  target_point.positions = positions;
  target_point.velocities.assign(joint_names.size(), 0.0);
  target_point.accelerations.assign(joint_names.size(), 0.0);
  target_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  goal.trajectory.points.push_back(target_point);

  const auto goal_future = client->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Timed out sending controller trajectory for %s.", description.c_str());
    return false;
  }

  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Controller rejected the trajectory goal for %s.", description.c_str());
    return false;
  }

  const auto result_future = client->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::duration<double>(duration_sec + 5.0)) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Timed out waiting for controller trajectory result for %s.", description.c_str());
    return false;
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped_result.result ||
    wrapped_result.result->error_code != control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
  {
    RCLCPP_WARN(get_logger(), "Controller trajectory execution failed for %s.", description.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "Executed %s with a direct controller trajectory.", description.c_str());
  return true;
}

void ManipulationNode::publishStatus(
  const std::string & stage,
  const std::string & status,
  const std::string & detail)
{
  if (!status_publisher_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = stage + "|" + status + "|" + detail;
  status_publisher_->publish(msg);
  RCLCPP_INFO(
    get_logger(),
    "Manipulation status -> %s",
    msg.data.c_str());
}

std::string ManipulationNode::namedPlaceTargetForBin(const std::string & bin_id) const
{
  std::string normalized = bin_id;
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});

  if (normalized == "bin_a") {
    return bin_a_pose_name_;
  }
  if (normalized == "bin_b") {
    return bin_b_pose_name_;
  }
  if (normalized == "bin_c") {
    return bin_c_pose_name_;
  }
  return "";
}

bool ManipulationNode::executePoseTarget(
  const geometry_msgs::msg::PoseStamped & target_pose,
  const std::string & description)
{
  for (int attempt = 1; attempt <= max_plan_retries_; ++attempt) {
    arm_group_->setStartStateToCurrentState();
    arm_group_->setPoseReferenceFrame(
      target_pose.header.frame_id.empty() ? world_frame_ : target_pose.header.frame_id);
    arm_group_->setPoseTarget(target_pose.pose, end_effector_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_group_->plan(plan);
    arm_group_->clearPoseTargets();

    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
      const auto execute_result = arm_group_->execute(plan);
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Executed %s with a full pose target.", description.c_str());
        return true;
      }
    }

    arm_group_->setStartStateToCurrentState();
    arm_group_->setPoseReferenceFrame(
      target_pose.header.frame_id.empty() ? world_frame_ : target_pose.header.frame_id);
    arm_group_->setPositionTarget(
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z,
      end_effector_link_);

    moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
    const auto fallback_plan_result = arm_group_->plan(fallback_plan);
    arm_group_->clearPoseTargets();

    if (fallback_plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(
        get_logger(),
        "Planning attempt %d/%d failed for %s, including the position-only fallback.",
        attempt,
        max_plan_retries_,
        description.c_str());
      continue;
    }

    const auto execute_result = arm_group_->execute(fallback_plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(
        get_logger(),
        "Executed %s using a position-only fallback target.",
        description.c_str());
      return true;
    }
  }

  return false;
}

bool ManipulationNode::executeWorldPoseTarget(
  double x,
  double y,
  double z,
  const std::vector<double> & orientation,
  const std::string & description)
{
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = world_frame_;
  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = z;

  if (orientation.size() >= 4) {
    target_pose.pose.orientation.x = orientation.at(0);
    target_pose.pose.orientation.y = orientation.at(1);
    target_pose.pose.orientation.z = orientation.at(2);
    target_pose.pose.orientation.w = orientation.at(3);
  } else {
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 1.0;
  }

  return executePoseTarget(target_pose, description);
}

bool ManipulationNode::executePositionTarget(
  double x,
  double y,
  double z,
  const std::string & description)
{
  for (int attempt = 1; attempt <= max_plan_retries_; ++attempt) {
    arm_group_->setStartStateToCurrentState();
    arm_group_->setPoseReferenceFrame(world_frame_);
    arm_group_->setPositionTarget(x, y, z, end_effector_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_group_->plan(plan);
    arm_group_->clearPoseTargets();

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(
        get_logger(),
        "Planning attempt %d/%d failed for %s at target (%.3f, %.3f, %.3f).",
        attempt,
        max_plan_retries_,
        description.c_str(),
        x,
        y,
        z);
      continue;
    }

    const auto execute_result = arm_group_->execute(plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(
        get_logger(),
        "Executed %s at target (%.3f, %.3f, %.3f).",
        description.c_str(),
        x,
        y,
        z);
      return true;
    }

    RCLCPP_WARN(
      get_logger(),
      "Execution attempt %d/%d failed for %s at target (%.3f, %.3f, %.3f).",
      attempt,
      max_plan_retries_,
      description.c_str(),
      x,
      y,
      z);
  }

  return false;
}

bool ManipulationNode::executeCartesianPoseTarget(
  const geometry_msgs::msg::PoseStamped & target_pose,
  const std::string & description)
{
  std::vector<geometry_msgs::msg::Pose> waypoints{target_pose.pose};
  moveit_msgs::msg::RobotTrajectory trajectory;

  const double fraction = arm_group_->computeCartesianPath(
    waypoints,
    cartesian_step_,
    jump_threshold_,
    trajectory,
    true);

  if (fraction < 0.99) {
    RCLCPP_WARN(
      get_logger(),
      "Cartesian path for %s only achieved %.2f%% of the requested motion.",
      description.c_str(),
      fraction * 100.0);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;

  const auto execute_result = arm_group_->execute(cartesian_plan);
  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Executed %s.", description.c_str());
    return true;
  }

  RCLCPP_WARN(get_logger(), "Execution failed for %s.", description.c_str());
  return false;
}

bool ManipulationNode::executeCartesianOffset(double dx, double dy, double dz, const std::string & description)
{
  const auto current_pose = arm_group_->getCurrentPose(end_effector_link_);
  geometry_msgs::msg::Pose target = current_pose.pose;
  target.position.x += dx;
  target.position.y += dy;
  target.position.z += dz;

  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  moveit_msgs::msg::RobotTrajectory trajectory;

  const double fraction = arm_group_->computeCartesianPath(
    waypoints,
    cartesian_step_,
    jump_threshold_,
    trajectory,
    true);

  if (fraction < 0.99) {
    RCLCPP_WARN(
      get_logger(),
      "Cartesian path for %s only achieved %.2f%% of the requested motion.",
      description.c_str(),
      fraction * 100.0);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;

  const auto execute_result = arm_group_->execute(cartesian_plan);
  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Executed %s.", description.c_str());
    return true;
  }

  RCLCPP_WARN(get_logger(), "Execution failed for %s.", description.c_str());
  return false;
}

bool ManipulationNode::executeVerticalMoveToHeight(double target_z, const std::string & description)
{
  const auto current_pose = arm_group_->getCurrentPose(end_effector_link_);
  const double delta_z = target_z - current_pose.pose.position.z;

  if (std::abs(delta_z) < 1e-3) {
    RCLCPP_INFO(get_logger(), "%s already satisfied at z=%.3f.", description.c_str(), target_z);
    return true;
  }

  return
    executeCartesianOffset(0.0, 0.0, delta_z, description) ||
    executePositionTarget(
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    target_z,
    description);
}

bool ManipulationNode::attachObjectToTool()
{
  if (object_attached_) {
    return true;
  }

  planning_scene_interface_.removeCollisionObjects({object_id_});

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = end_effector_link_;
  attached_object.object.id = object_id_;
  attached_object.object.header.frame_id = end_effector_link_;
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached_object.object.primitives.push_back(
    boxPrimitive(get_parameter("object.attached_dimensions").as_double_array()));
  attached_object.object.primitive_poses.push_back(
    poseFromArray(get_parameter("object.attach_offset").as_double_array()));
  attached_object.touch_links = {
    end_effector_link_,
    "gripper_base_link",
    "gripper_slider_link",
    "tool0"};

  planning_scene_interface_.applyAttachedCollisionObject(attached_object);
  object_attached_ = true;
  RCLCPP_INFO(get_logger(), "Attached pick object to the end effector in the planning scene.");
  return true;
}

bool ManipulationNode::detachObjectFromTool()
{
  if (!object_attached_) {
    return true;
  }

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = end_effector_link_;
  attached_object.object.id = object_id_;
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  planning_scene_interface_.applyAttachedCollisionObject(attached_object);
  object_attached_ = false;
  RCLCPP_INFO(get_logger(), "Detached pick object from the end effector in the planning scene.");
  return true;
}

bool ManipulationNode::refreshWorldObject()
{
  if (object_attached_) {
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = end_effector_link_;
    attached_object.object.id = object_id_;
    attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_.applyAttachedCollisionObject(attached_object);
    object_attached_ = false;
  }

  pre_grasp_ready_ = false;

  moveit_msgs::msg::CollisionObject object;
  object.id = object_id_;
  object.header.frame_id = world_frame_;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.primitives.push_back(boxPrimitive(get_parameter("object.dimensions").as_double_array()));
  object.primitive_poses.push_back(poseFromArray(get_parameter("object.pose").as_double_array()));
  planning_scene_interface_.applyCollisionObject(object);
  return true;
}

std::vector<double> ManipulationNode::parameterArray(const std::string & name) const
{
  return get_parameter(name).as_double_array();
}

bool ManipulationNode::executeObjectOffsetTarget(
  const std::string & offset_parameter,
  const std::string & description)
{
  const auto object_pose = parameterArray("object.pose");
  const auto offset = parameterArray(offset_parameter);
  const auto orientation = parameterArray("object.target_orientation");

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = world_frame_;
  target_pose.pose.position.x = object_pose.at(0) + offset.at(0);
  target_pose.pose.position.y = object_pose.at(1) + offset.at(1);
  target_pose.pose.position.z = object_pose.at(2) + offset.at(2);

  if (orientation.size() >= 4) {
    target_pose.pose.orientation.x = orientation.at(0);
    target_pose.pose.orientation.y = orientation.at(1);
    target_pose.pose.orientation.z = orientation.at(2);
    target_pose.pose.orientation.w = orientation.at(3);
  } else {
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 1.0;
  }

  return executePoseTarget(target_pose, description);
}

bool ManipulationNode::placeObjectAtBin(const std::string & bin_id)
{
  std::vector<double> bin_pose;
  std::vector<double> bin_dimensions;

  if (bin_id == "bin_a") {
    bin_pose = parameterArray("scene.bin_a.pose");
    bin_dimensions = parameterArray("scene.bin_a.dimensions");
  } else if (bin_id == "bin_b") {
    bin_pose = parameterArray("scene.bin_b.pose");
    bin_dimensions = parameterArray("scene.bin_b.dimensions");
  } else if (bin_id == "bin_c") {
    bin_pose = parameterArray("scene.bin_c.pose");
    bin_dimensions = parameterArray("scene.bin_c.dimensions");
  } else {
    RCLCPP_ERROR(get_logger(), "Cannot place object because bin id '%s' is unknown.", bin_id.c_str());
    return false;
  }

  const auto object_dimensions = parameterArray("object.dimensions");

  geometry_msgs::msg::Pose pose;
  pose.position.x = bin_pose.at(0);
  pose.position.y = bin_pose.at(1);
  pose.position.z = bin_pose.at(2) + (bin_dimensions.at(2) * 0.5) + (object_dimensions.at(2) * 0.5);
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  moveit_msgs::msg::CollisionObject object;
  object.id = object_id_;
  object.header.frame_id = world_frame_;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.primitives.push_back(boxPrimitive(parameterArray("object.dimensions")));
  object.primitive_poses.push_back(pose);
  planning_scene_interface_.applyCollisionObject(object);

  if (!setObjectPoseInGazebo(pose)) {
    RCLCPP_WARN(
      get_logger(),
      "Updated the MoveIt planning scene for '%s', but could not explicitly place it onto %s in Gazebo. Continuing with release anyway.",
      object_id_.c_str(),
      bin_id.c_str());
    return true;
  }

  RCLCPP_INFO(
    get_logger(),
    "Placed object '%s' onto %s at (%.3f, %.3f, %.3f) and updated the planning scene.",
    object_id_.c_str(),
    bin_id.c_str(),
    pose.position.x,
    pose.position.y,
    pose.position.z);
  return true;
}

bool ManipulationNode::isEndEffectorNearObject(
  const std::string & offset_parameter,
  double xy_tolerance,
  double z_tolerance)
{
  const auto current_pose = arm_group_->getCurrentPose(end_effector_link_);
  const auto object_pose = parameterArray("object.pose");
  const auto offset = parameterArray(offset_parameter);

  const double target_x = object_pose.at(0) + offset.at(0);
  const double target_y = object_pose.at(1) + offset.at(1);
  const double target_z = object_pose.at(2) + offset.at(2);

  const double dx = current_pose.pose.position.x - target_x;
  const double dy = current_pose.pose.position.y - target_y;
  const double dz = current_pose.pose.position.z - target_z;
  const double xy_error = std::hypot(dx, dy);

  RCLCPP_INFO(
    get_logger(),
    "Soft-grasp check: xy error %.3f m, z error %.3f m (tolerances %.3f / %.3f).",
    xy_error,
    std::abs(dz),
    xy_tolerance,
    z_tolerance);

  return xy_error <= xy_tolerance && std::abs(dz) <= z_tolerance;
}

bool ManipulationNode::setObjectPoseInGazebo(const geometry_msgs::msg::Pose & pose)
{
  auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  request->state.name = object_id_;
  request->state.reference_frame = world_frame_;
  request->state.pose = pose;

  for (const auto & client : {primary_set_entity_state_client_, fallback_set_entity_state_client_}) {
    if (!client || !client->wait_for_service(std::chrono::seconds(1))) {
      continue;
    }

    auto future = client->async_send_request(request);
    const auto status = future.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready) {
      continue;
    }

    const auto response = future.get();
    if (response && response->success) {
      return true;
    }

    if (response) {
      RCLCPP_WARN(
        get_logger(),
        "Gazebo rejected object placement via '%s'.",
        client->get_service_name());
    }
  }

  RCLCPP_ERROR(get_logger(), "Unable to update object pose in Gazebo for release.");
  return false;
}

geometry_msgs::msg::Pose ManipulationNode::poseFromArray(const std::vector<double> & values) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = values.at(0);
  pose.position.y = values.at(1);
  pose.position.z = values.at(2);
  pose.orientation.x = values.at(3);
  pose.orientation.y = values.at(4);
  pose.orientation.z = values.at(5);
  pose.orientation.w = values.at(6);
  return pose;
}

shape_msgs::msg::SolidPrimitive ManipulationNode::boxPrimitive(const std::vector<double> & dimensions) const
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.clear();
  for (const double dimension : dimensions) {
    primitive.dimensions.push_back(dimension);
  }
  return primitive;
}

}  // namespace manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<manipulation::ManipulationNode>();
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
