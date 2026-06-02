#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cgn_flexbe_utilities/srv/move_to_pose.hpp"
using MoveToPoseSrv = cgn_flexbe_utilities::srv::MoveToPose;

#include <std_srvs/srv/trigger.hpp>

class ReachToGraspNode : public rclcpp::Node
{
public:
  ReachToGraspNode()
  : Node("reach_to_grasp_service")
  {
    // Original Panda defaults:
    // this->declare_parameter<std::string>("planning_group", "panda_arm");
    // this->declare_parameter<std::string>("gripper_group", "panda_hand");

    // GEN3/real-robot defaults verified from the Kinova SRDF:
    //   manipulator: base_link -> end_effector_link
    //   gripper: Robotiq 2F-85 group with named states Open / Close
    this->declare_parameter<std::string>("planning_group", "manipulator");
    this->declare_parameter<std::string>("gripper_group", "gripper");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("ee_link", "end_effector_link");

    // GEN3/Robotiq: prefer SRDF named states instead of manually filling all gripper joints.
    this->declare_parameter<bool>("use_named_gripper_targets", true);
    this->declare_parameter<std::string>("open_named_target", "Open");
    this->declare_parameter<std::string>("close_named_target", "Close");

    // Fallback joint-value mode. For the Kinova Robotiq SRDF, the useful actuated joint is
    // typically robotiq_85_left_knuckle_joint: Open=0.0, Close=0.8.
    this->declare_parameter<double>("open_gripper_joint_value", 0.0);
    this->declare_parameter<double>("close_gripper_joint_value", 0.8);

    // Motion sequence parameters. Defaults are conservative for your current pipeline where
    // move_to_pose already places the gripper at the CGN grasp pose.
    this->declare_parameter<bool>("open_before_grasp", true);
    this->declare_parameter<bool>("move_to_grasp_pose_first", true);
    this->declare_parameter<double>("pregrasp_base_z_offset", 0.0);     // Original temporary value was -0.05
    this->declare_parameter<double>("approach_ee_z_distance", 0.0);     // Original temporary value was 0.12
    this->declare_parameter<double>("lift_base_z_distance", 0.10);      // Lift after closing
    this->declare_parameter<bool>("reopen_after_lift", false);          // Original code reopened/dropped after lift

    this->declare_parameter<double>("arm_planning_time", 5.0);
    this->declare_parameter<double>("gripper_planning_time", 5.0);
    this->declare_parameter<double>("arm_velocity_scaling", 0.2);
    this->declare_parameter<double>("arm_acceleration_scaling", 0.2);
    this->declare_parameter<double>("gripper_velocity_scaling", 0.5);
    this->declare_parameter<double>("gripper_acceleration_scaling", 0.5);

    std::string arm_group_name, gripper_group_name;
    this->get_parameter("planning_group", arm_group_name);
    this->get_parameter("gripper_group", gripper_group_name);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("ee_link", ee_link_);
    this->get_parameter("use_named_gripper_targets", use_named_gripper_targets_);
    this->get_parameter("open_named_target", open_named_target_);
    this->get_parameter("close_named_target", close_named_target_);
    this->get_parameter("open_gripper_joint_value", open_gripper_joint_value_);
    this->get_parameter("close_gripper_joint_value", close_gripper_joint_value_);
    this->get_parameter("open_before_grasp", open_before_grasp_);
    this->get_parameter("move_to_grasp_pose_first", move_to_grasp_pose_first_);
    this->get_parameter("pregrasp_base_z_offset", pregrasp_base_z_offset_);
    this->get_parameter("approach_ee_z_distance", approach_ee_z_distance_);
    this->get_parameter("lift_base_z_distance", lift_base_z_distance_);
    this->get_parameter("reopen_after_lift", reopen_after_lift_);
    this->get_parameter("arm_planning_time", arm_planning_time_);
    this->get_parameter("gripper_planning_time", gripper_planning_time_);
    this->get_parameter("arm_velocity_scaling", arm_velocity_scaling_);
    this->get_parameter("arm_acceleration_scaling", arm_acceleration_scaling_);
    this->get_parameter("gripper_velocity_scaling", gripper_velocity_scaling_);
    this->get_parameter("gripper_acceleration_scaling", gripper_acceleration_scaling_);

    RCLCPP_INFO(this->get_logger(), "ReachToGraspNode using arm group: %s, hand group: %s",
                arm_group_name.c_str(), gripper_group_name.c_str());
    RCLCPP_INFO(this->get_logger(), "base_frame: %s, requested ee_link: %s",
                base_frame_.c_str(), ee_link_.c_str());

    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), arm_group_name);

    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), gripper_group_name);

    if (!ee_link_.empty())
    {
      // GEN3: explicitly set end_effector_link to avoid Panda defaults / empty EE link.
      arm_group_->setEndEffectorLink(ee_link_);
    }

    arm_group_->setPlanningTime(arm_planning_time_);
    arm_group_->setMaxVelocityScalingFactor(arm_velocity_scaling_);
    arm_group_->setMaxAccelerationScalingFactor(arm_acceleration_scaling_);

    gripper_group_->setPlanningTime(gripper_planning_time_);
    gripper_group_->setMaxVelocityScalingFactor(gripper_velocity_scaling_);
    gripper_group_->setMaxAccelerationScalingFactor(gripper_acceleration_scaling_);

    RCLCPP_INFO(this->get_logger(), "MoveIt planning frame: %s", arm_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "MoveIt end-effector link: %s", arm_group_->getEndEffectorLink().c_str());

    // Original Panda hand joint-value setup:
    // open_gripper_joint_values_.assign(joint_names.size(), 0.04);
    // close_gripper_joint_values_.assign(joint_names.size(), 0.015);

    // GEN3 fallback joint-value setup. Named targets are used by default, so these are only
    // used if use_named_gripper_targets:=False.
    open_gripper_joint_values_.clear();
    close_gripper_joint_values_.clear();
    const auto& joint_names = gripper_group_->getJointNames();
    open_gripper_joint_values_.assign(joint_names.size(), open_gripper_joint_value_);
    close_gripper_joint_values_.assign(joint_names.size(), close_gripper_joint_value_);

    RCLCPP_INFO(this->get_logger(), "Gripper group has %zu joint names. Named target mode: %s",
                joint_names.size(), use_named_gripper_targets_ ? "true" : "false");
    for (const auto& j : joint_names)
    {
      RCLCPP_INFO(this->get_logger(), "  gripper joint: %s", j.c_str());
    }

    service_ = this->create_service<MoveToPoseSrv>(
      "/reach_to_grasp",
      std::bind(&ReachToGraspNode::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ReachToGrasp service '/reach_to_grasp' is ready.");
  }

private:
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Service<MoveToPoseSrv>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  std::vector<double> open_gripper_joint_values_;
  std::vector<double> close_gripper_joint_values_;

  std::string base_frame_;
  std::string ee_link_;
  bool use_named_gripper_targets_ = true;
  std::string open_named_target_ = "Open";
  std::string close_named_target_ = "Close";
  double open_gripper_joint_value_ = 0.0;
  double close_gripper_joint_value_ = 0.8;

  bool open_before_grasp_ = true;
  bool move_to_grasp_pose_first_ = true;
  double pregrasp_base_z_offset_ = 0.0;
  double approach_ee_z_distance_ = 0.0;
  double lift_base_z_distance_ = 0.10;
  bool reopen_after_lift_ = false;

  double arm_planning_time_ = 5.0;
  double gripper_planning_time_ = 5.0;
  double arm_velocity_scaling_ = 0.2;
  double arm_acceleration_scaling_ = 0.2;
  double gripper_velocity_scaling_ = 0.5;
  double gripper_acceleration_scaling_ = 0.5;

  geometry_msgs::msg::PoseStamped target_ps;

  void handle_request(const std::shared_ptr<MoveToPoseSrv::Request> req,
                      std::shared_ptr<MoveToPoseSrv::Response> res)
  {
    try
    {
      // Use the requested pose as grasp pose in base frame.
      const geometry_msgs::msg::Pose& grasp_pose = req->target_pose;

      geometry_msgs::msg::PoseStamped grasp_ps;
      // Original Panda/simulation line:
      // grasp_ps.header.frame_id = "panda_link0";
      // Earlier generic line:
      // grasp_ps.header.frame_id = arm_group_->getPlanningFrame();
      // GEN3/real robot: make this explicit and launch-configurable.
      grasp_ps.header.frame_id = base_frame_;
      grasp_ps.pose = grasp_pose;

      RCLCPP_INFO(this->get_logger(),
                  "Reach-to-grasp request in frame '%s': pos=(%.3f, %.3f, %.3f)",
                  grasp_ps.header.frame_id.c_str(),
                  grasp_ps.pose.position.x, grasp_ps.pose.position.y, grasp_ps.pose.position.z);

      // 1) Open gripper before approaching/closing, if requested.
      if (open_before_grasp_)
      {
        if (!setGripperOpen())
        {
          res->success = false;
          return;
        }
      }

      // Optional pregrasp base-Z offset from requested pose. Default 0.0 keeps your current
      // working move_to_pose behavior unchanged.
      geometry_msgs::msg::PoseStamped close_pose = grasp_ps;
      if (std::abs(pregrasp_base_z_offset_) > 1e-6)
      {
        // Original temporary line:
        // if (!moveInBaseZ(grasp_ps, -0.05)) { ... }
        geometry_msgs::msg::PoseStamped pregrasp_ps = grasp_ps;
        pregrasp_ps.pose.position.z += pregrasp_base_z_offset_;
        if (!moveToPose(pregrasp_ps))
        {
          res->success = false;
          return;
        }
      }

      // Move to the actual CGN grasp pose. If you already called /move_to_pose first, this
      // is usually a near-zero/no-op motion, but it keeps /reach_to_grasp self-contained.
      if (move_to_grasp_pose_first_)
      {
        if (!moveToPose(grasp_ps))
        {
          res->success = false;
          return;
        }
        close_pose = grasp_ps;
      }

      // Optional approach along the end-effector Z axis. Default 0.0 because your CGN pose
      // already works with convert/apply offsets disabled.
      if (std::abs(approach_ee_z_distance_) > 1e-6)
      {
        // Original temporary line:
        // if (!moveAlongEndEffectorZ(target_ps, 0.12)) { ... }
        if (!moveAlongEndEffectorZ(close_pose, approach_ee_z_distance_))
        {
          res->success = false;
          return;
        }
        close_pose = target_ps;
      }

      // 2) Close gripper on object.
      if (!setGripperClose())
      {
        res->success = false;
        return;
      }

      // 3) Lift vertically in base frame while keeping the grasp orientation.
      if (std::abs(lift_base_z_distance_) > 1e-6)
      {
        if (!moveInBaseZ(close_pose, lift_base_z_distance_))
        {
          res->success = false;
          return;
        }
      }

      // Original code reopened the gripper after lift, which drops the object:
      // if (!setGripper(open_gripper_joint_values_)) { ... }
      // GEN3 default keeps the object grasped. Enable reopen_after_lift:=True only for drop tests.
      if (reopen_after_lift_)
      {
        if (!setGripperOpen())
        {
          res->success = false;
          return;
        }
      }

      res->success = true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception in grasp sequence: %s", e.what());
      res->success = false;
    }
  }

  bool setCurrentStateForGroup(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& group,
                               const std::string& label)
  {
    moveit::core::RobotStatePtr current_state = group->getCurrentState(1.0);
    if (current_state)
    {
      current_state->enforceBounds();
      group->setStartState(*current_state);
      RCLCPP_INFO(this->get_logger(), "%s: Using current robot state as start state.", label.c_str());
      return true;
    }

    RCLCPP_WARN(this->get_logger(),
                "%s: No current robot state available within 1.0s; using default robot state.",
                label.c_str());
    moveit::core::RobotState default_state(group->getRobotModel());
    default_state.setToDefaultValues();
    default_state.enforceBounds();
    group->setStartState(default_state);
    return false;
  }

  bool setGripperOpen()
  {
    if (use_named_gripper_targets_)
    {
      return setGripperNamed(open_named_target_, "openGripper");
    }
    return setGripper(open_gripper_joint_values_);
  }

  bool setGripperClose()
  {
    if (use_named_gripper_targets_)
    {
      return setGripperNamed(close_named_target_, "closeGripper");
    }
    return setGripper(close_gripper_joint_values_);
  }

  bool setGripperNamed(const std::string& named_target, const std::string& label)
  {
    setCurrentStateForGroup(gripper_group_, label);

    gripper_group_->setPlanningTime(gripper_planning_time_);
    gripper_group_->setMaxVelocityScalingFactor(gripper_velocity_scaling_);
    gripper_group_->setMaxAccelerationScalingFactor(gripper_acceleration_scaling_);

    // GEN3/Robotiq: use SRDF group_state names, e.g., Open and Close.
    if (!gripper_group_->setNamedTarget(named_target))
    {
      RCLCPP_WARN(this->get_logger(), "%s: Failed to set named target '%s'.", label.c_str(), named_target.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = gripper_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "%s: Gripper planning failed for named target '%s'.",
                  label.c_str(), named_target.c_str());
      return false;
    }

    auto exec = gripper_group_->execute(plan);
    if (exec == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "%s: Gripper execution succeeded for named target '%s'.",
                  label.c_str(), named_target.c_str());
      return true;
    }

    // Closing on a real object may report timeout/control failure after contact. Treat this as success
    // only for close, not for open.
    if (named_target == close_named_target_ &&
        (exec == moveit::core::MoveItErrorCode::TIMED_OUT ||
         exec == moveit::core::MoveItErrorCode::CONTROL_FAILED))
    {
      RCLCPP_WARN(this->get_logger(),
                  "%s: gripper execution returned %d while closing; assuming object contact and treating as success.",
                  label.c_str(), static_cast<int>(exec.val));
      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "%s: Gripper execution failed with code %d.",
                 label.c_str(), static_cast<int>(exec.val));
    return false;
  }

  bool setGripper(const std::vector<double>& joint_values)
  {
    // Original joint-value function kept as fallback for simulation/Panda or manual testing.
    setCurrentStateForGroup(gripper_group_, "setGripper");

    gripper_group_->setPlanningTime(gripper_planning_time_);
    gripper_group_->setMaxVelocityScalingFactor(gripper_velocity_scaling_);
    gripper_group_->setMaxAccelerationScalingFactor(gripper_acceleration_scaling_);

    gripper_group_->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = gripper_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "Gripper planning failed.");
      return false;
    }

    auto exec = gripper_group_->execute(plan);
    if (exec == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Gripper execution succeeded.");
      return true;
    }

    if (exec == moveit::core::MoveItErrorCode::TIMED_OUT ||
        exec == moveit::core::MoveItErrorCode::CONTROL_FAILED)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Gripper: execution returned %d (TIMED_OUT / CONTROL_FAILED). Assuming contact and treating as success.",
                  static_cast<int>(exec.val));
      return true;
    }

    return false;
  }

  bool moveToPose(const geometry_msgs::msg::PoseStamped& pose)
  {
    setCurrentStateForGroup(arm_group_, "moveToPose");

    arm_group_->setPlanningTime(arm_planning_time_);
    arm_group_->setMaxVelocityScalingFactor(arm_velocity_scaling_);
    arm_group_->setMaxAccelerationScalingFactor(arm_acceleration_scaling_);

    const std::string ee_link = ee_link_.empty() ? arm_group_->getEndEffectorLink() : ee_link_;
    arm_group_->setPoseTarget(pose, ee_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = arm_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "Planning to grasp pose failed.");
      arm_group_->clearPoseTargets();
      return false;
    }

    auto exec = arm_group_->execute(plan);
    arm_group_->clearPoseTargets();
    if (exec != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Execution to grasp pose failed with code %d.", static_cast<int>(exec.val));
      return false;
    }

    target_ps = pose;
    return true;
  }

  bool moveAlongEndEffectorZ(const geometry_msgs::msg::PoseStamped& ref_pose, double distance)
  {
    target_ps = ref_pose;

    tf2::Quaternion q;
    tf2::fromMsg(target_ps.pose.orientation, q);
    tf2::Matrix3x3 R(q);

    // Move along +Z_EE or -Z_EE depending on distance sign.
    tf2::Vector3 dz_ee(0.0, 0.0, distance);
    tf2::Vector3 dz_world = R * dz_ee;

    target_ps.pose.position.x += dz_world.x();
    target_ps.pose.position.y += dz_world.y();
    target_ps.pose.position.z += dz_world.z();

    return moveToPose(target_ps);
  }

  bool moveInBaseZ(const geometry_msgs::msg::PoseStamped& ref_pose, double dz)
  {
    target_ps = ref_pose;
    target_ps.pose.position.z += dz;  // base-frame Z

    RCLCPP_INFO(this->get_logger(), "Lift target z = %f", target_ps.pose.position.z);
    return moveToPose(target_ps);
  }

  bool moveInBaseY(const geometry_msgs::msg::PoseStamped& ref_pose, double dy)
  {
    target_ps = ref_pose;
    target_ps.pose.position.y += dy;  // base-frame Y

    RCLCPP_INFO(this->get_logger(), "Base-Y target y = %f", target_ps.pose.position.y);
    return moveToPose(target_ps);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReachToGraspNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
