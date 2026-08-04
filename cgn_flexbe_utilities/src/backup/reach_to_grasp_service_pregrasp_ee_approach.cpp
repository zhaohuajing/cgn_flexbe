#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <cmath>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cgn_flexbe_utilities/srv/move_to_pose.hpp"
using MoveToPoseSrv = cgn_flexbe_utilities::srv::MoveToPose;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

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

    // GEN3/Robotiq real-hardware default: use the GripperActionController directly.
    // This matches the manually verified action:
    //   /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand
    // Original MoveIt/SRDF named-target gripper method is preserved as fallback by setting
    // use_gripper_action:=False.
    this->declare_parameter<bool>("use_gripper_action", true);
    this->declare_parameter<std::string>("gripper_action_name", "/robotiq_gripper_controller/gripper_cmd");
    this->declare_parameter<double>("open_gripper_position", 0.0);
    this->declare_parameter<double>("close_gripper_position", 0.8);
    this->declare_parameter<double>("gripper_max_effort", 100.0);
    this->declare_parameter<double>("gripper_action_timeout", 10.0);

    // Motion sequence parameters. Defaults are conservative for your current pipeline where
    // move_to_pose already places the gripper at the CGN grasp pose.
    this->declare_parameter<bool>("open_before_grasp", true);
    this->declare_parameter<bool>("move_to_grasp_pose_first", false);
    this->declare_parameter<double>("pregrasp_base_z_offset", 0.0);     // Original temporary value was -0.05
    this->declare_parameter<double>("approach_ee_z_distance", 0.10);    // Move from pregrasp to actual grasp along +Z_EE
    this->declare_parameter<double>("lift_base_z_distance", 0.10);      // Lift after closing

    // After lifting, optionally move to a fixed base-frame drop/place pose,
    // then open the gripper.  This keeps the earlier lift behavior but makes
    // the actual release happen at a safer, known pose.
    this->declare_parameter<bool>("move_to_drop_pose_after_lift", true);
    this->declare_parameter<double>("drop_pose_x", 0.12054);
    this->declare_parameter<double>("drop_pose_y", -0.23436);
    this->declare_parameter<double>("drop_pose_z", 0.39196);
    this->declare_parameter<double>("drop_pose_qx", 0.706244);
    this->declare_parameter<double>("drop_pose_qy", 0.706793);
    this->declare_parameter<double>("drop_pose_qz", 0.0287094);
    this->declare_parameter<double>("drop_pose_qw", 0.0289661);
    this->declare_parameter<bool>("reopen_after_lift", true);           // Drop/release after optional drop-pose motion

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
    this->get_parameter("use_gripper_action", use_gripper_action_);
    this->get_parameter("gripper_action_name", gripper_action_name_);
    this->get_parameter("open_gripper_position", open_gripper_position_);
    this->get_parameter("close_gripper_position", close_gripper_position_);
    this->get_parameter("gripper_max_effort", gripper_max_effort_);
    this->get_parameter("gripper_action_timeout", gripper_action_timeout_);
    this->get_parameter("open_before_grasp", open_before_grasp_);
    this->get_parameter("move_to_grasp_pose_first", move_to_grasp_pose_first_);
    this->get_parameter("pregrasp_base_z_offset", pregrasp_base_z_offset_);
    this->get_parameter("approach_ee_z_distance", approach_ee_z_distance_);
    this->get_parameter("lift_base_z_distance", lift_base_z_distance_);
    this->get_parameter("move_to_drop_pose_after_lift", move_to_drop_pose_after_lift_);
    this->get_parameter("drop_pose_x", drop_pose_x_);
    this->get_parameter("drop_pose_y", drop_pose_y_);
    this->get_parameter("drop_pose_z", drop_pose_z_);
    this->get_parameter("drop_pose_qx", drop_pose_qx_);
    this->get_parameter("drop_pose_qy", drop_pose_qy_);
    this->get_parameter("drop_pose_qz", drop_pose_qz_);
    this->get_parameter("drop_pose_qw", drop_pose_qw_);
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

    // GEN3/Robotiq action client. The old MoveIt gripper group remains available as fallback.
    // IMPORTANT: use a separate Reentrant callback group for the action client.
    // The /reach_to_grasp service callback waits for the action result; if the action
    // client shares the default mutually-exclusive callback group, the goal-response
    // callback can be blocked and the service reports "timed out waiting for goal response".
    gripper_action_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      gripper_action_name_,
      gripper_action_callback_group_,
      rcl_action_client_get_default_options());

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
    RCLCPP_INFO(this->get_logger(),
                "Post-lift drop pose: enabled=%s, frame=%s, pos=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f), reopen_after_lift=%s",
                move_to_drop_pose_after_lift_ ? "true" : "false",
                base_frame_.c_str(),
                drop_pose_x_, drop_pose_y_, drop_pose_z_,
                drop_pose_qx_, drop_pose_qy_, drop_pose_qz_, drop_pose_qw_,
                reopen_after_lift_ ? "true" : "false");

    RCLCPP_INFO(this->get_logger(),
                "Pregrasp approach: move_to_grasp_pose_first=%s, approach_ee_z_distance=%.3f m. "
                "Assuming /move_to_pose first moved to actual_grasp - distance * Z_EE.",
                move_to_grasp_pose_first_ ? "true" : "false",
                approach_ee_z_distance_);

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

    RCLCPP_INFO(this->get_logger(), "Gripper group has %zu joint names. Named target mode: %s, action mode: %s, action: %s",
                joint_names.size(),
                use_named_gripper_targets_ ? "true" : "false",
                use_gripper_action_ ? "true" : "false",
                gripper_action_name_.c_str());
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

  bool use_gripper_action_ = true;
  std::string gripper_action_name_ = "/robotiq_gripper_controller/gripper_cmd";
  double open_gripper_position_ = 0.0;
  double close_gripper_position_ = 0.8;
  double gripper_max_effort_ = 100.0;
  double gripper_action_timeout_ = 10.0;
  rclcpp::CallbackGroup::SharedPtr gripper_action_callback_group_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;


  bool open_before_grasp_ = true;
  bool move_to_grasp_pose_first_ = false;
  double pregrasp_base_z_offset_ = 0.0;
  double approach_ee_z_distance_ = 0.10;
  double lift_base_z_distance_ = 0.15;
  bool move_to_drop_pose_after_lift_ = true;
  double drop_pose_x_ = 0.12054;
  double drop_pose_y_ = -0.23436;
  double drop_pose_z_ = 0.39196;
  double drop_pose_qx_ = 0.706244;
  double drop_pose_qy_ = 0.706793;
  double drop_pose_qz_ = 0.0287094;
  double drop_pose_qw_ = 0.0289661;
  bool reopen_after_lift_ = true;

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

      // The request pose is the actual grasp pose.  In the updated pipeline,
      // /move_to_pose first moves to a pregrasp pose:
      //   pregrasp = actual_grasp - approach_ee_z_distance * Z_EE
      // This state starts from that pregrasp pose and moves by
      // +approach_ee_z_distance along Z_EE to reach the actual grasp pose.
      // Use +0.10 for the normal +Z_EE approach convention. If the gripper's
      // approach direction is opposite, use -0.10 in both services.
      const double approach_distance = approach_ee_z_distance_;

      geometry_msgs::msg::PoseStamped pregrasp_ps = grasp_ps;
      if (std::abs(approach_distance) > 1e-6)
      {
        pregrasp_ps = offsetPoseAlongEndEffectorZ(grasp_ps, -approach_distance);
      }

      // Optional additional base-Z offset, preserved for compatibility.  Keep
      // this at 0.0 for the normal UOC/GraspSAM pregrasp workflow; otherwise
      // the final approach target will also be shifted by this base-frame amount.
      geometry_msgs::msg::PoseStamped approach_start_ps = pregrasp_ps;
      if (std::abs(pregrasp_base_z_offset_) > 1e-6)
      {
        approach_start_ps.pose.position.z += pregrasp_base_z_offset_;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Computed pregrasp/start pose in frame '%s': pos=(%.3f, %.3f, %.3f), approach_distance=%.3f",
                  approach_start_ps.header.frame_id.c_str(),
                  approach_start_ps.pose.position.x,
                  approach_start_ps.pose.position.y,
                  approach_start_ps.pose.position.z,
                  approach_distance);

      // Optional self-contained mode: move to the pregrasp pose first.  In the
      // normal FlexBE sequence, /move_to_pose has already done this, so keep
      // move_to_grasp_pose_first:=false.
      if (move_to_grasp_pose_first_)
      {
        if (!moveToPose(approach_start_ps))
        {
          res->success = false;
          return;
        }
      }

      geometry_msgs::msg::PoseStamped close_pose = approach_start_ps;

      // Approach from pregrasp to actual grasp along the end-effector +Z axis.
      if (std::abs(approach_distance) > 1e-6)
      {
        if (!moveAlongEndEffectorZ(close_pose, approach_distance))
        {
          res->success = false;
          return;
        }
        close_pose = target_ps;
      }
      else
      {
        close_pose = grasp_ps;
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

      // 4) Move to a fixed base-frame drop/place pose before releasing.
      // This pose is independent of the grasp orientation and is specified in base_frame_.
      if (move_to_drop_pose_after_lift_)
      {
        geometry_msgs::msg::PoseStamped drop_ps;
        drop_ps.header.frame_id = base_frame_;
        drop_ps.header.stamp = this->now();
        drop_ps.pose.position.x = drop_pose_x_;
        drop_ps.pose.position.y = drop_pose_y_;
        drop_ps.pose.position.z = drop_pose_z_;
        drop_ps.pose.orientation.x = drop_pose_qx_;
        drop_ps.pose.orientation.y = drop_pose_qy_;
        drop_ps.pose.orientation.z = drop_pose_qz_;
        drop_ps.pose.orientation.w = drop_pose_qw_;

        RCLCPP_INFO(this->get_logger(),
                    "Moving to post-lift drop pose in frame '%s': pos=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
                    drop_ps.header.frame_id.c_str(),
                    drop_ps.pose.position.x, drop_ps.pose.position.y, drop_ps.pose.position.z,
                    drop_ps.pose.orientation.x, drop_ps.pose.orientation.y,
                    drop_ps.pose.orientation.z, drop_ps.pose.orientation.w);

        if (!moveToPose(drop_ps))
        {
          res->success = false;
          return;
        }
      }

      // 5) Drop/release the object after the optional drop-pose motion.
      // Set reopen_after_lift:=false if you want to keep holding the object.
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
    if (use_gripper_action_)
    {
      return sendGripperAction(open_gripper_position_, gripper_max_effort_, "openGripperAction", false);
    }

    // Original MoveIt/SRDF named-target path preserved as fallback.
    if (use_named_gripper_targets_)
    {
      return setGripperNamed(open_named_target_, "openGripper");
    }
    return setGripper(open_gripper_joint_values_);
  }

  bool setGripperClose()
  {
    if (use_gripper_action_)
    {
      return sendGripperAction(close_gripper_position_, gripper_max_effort_, "closeGripperAction", true);
    }

    // Original MoveIt/SRDF named-target path preserved as fallback.
    if (use_named_gripper_targets_)
    {
      return setGripperNamed(close_named_target_, "closeGripper");
    }
    return setGripper(close_gripper_joint_values_);
  }

  bool sendGripperAction(double position, double max_effort, const std::string& label, bool closing)
  {
    if (!gripper_action_client_)
    {
      RCLCPP_ERROR(this->get_logger(), "%s: gripper action client is not initialized.", label.c_str());
      return false;
    }

    const auto timeout = std::chrono::duration<double>(gripper_action_timeout_);
    if (!gripper_action_client_->wait_for_action_server(timeout))
    {
      RCLCPP_ERROR(this->get_logger(), "%s: action server '%s' not available after %.2f s.",
                   label.c_str(), gripper_action_name_.c_str(), gripper_action_timeout_);
      return false;
    }

    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = max_effort;

    RCLCPP_INFO(this->get_logger(), "%s: sending Robotiq goal position=%.3f, max_effort=%.3f to %s",
                label.c_str(), position, max_effort, gripper_action_name_.c_str());

    std::mutex mutex;
    std::condition_variable cv;
    bool goal_response_received = false;
    bool result_received = false;
    bool goal_accepted = false;
    bool result_success = false;
    int result_code = -999;
    double result_position = 0.0;
    double result_effort = 0.0;
    bool result_stalled = false;
    bool result_reached_goal = false;

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [&](const GoalHandleGripperCommand::SharedPtr& goal_handle)
      {
        std::lock_guard<std::mutex> lock(mutex);
        goal_response_received = true;
        goal_accepted = static_cast<bool>(goal_handle);
        cv.notify_all();
      };

    send_goal_options.result_callback =
      [&](const GoalHandleGripperCommand::WrappedResult& result)
      {
        std::lock_guard<std::mutex> lock(mutex);
        result_received = true;
        result_code = static_cast<int>(result.code);
        if (result.result)
        {
          result_position = result.result->position;
          result_effort = result.result->effort;
          result_stalled = result.result->stalled;
          result_reached_goal = result.result->reached_goal;
        }
        result_success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        cv.notify_all();
      };

    gripper_action_client_->async_send_goal(goal, send_goal_options);

    // This wait assumes main() uses MultiThreadedExecutor AND that the action client is
    // in a callback group that can run while this service callback is blocked.
    // See gripper_action_callback_group_ in the constructor and main() at the bottom.
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (!cv.wait_for(lock, timeout, [&] { return goal_response_received; }))
      {
        RCLCPP_ERROR(this->get_logger(), "%s: timed out waiting for goal response.", label.c_str());
        return false;
      }
      if (!goal_accepted)
      {
        RCLCPP_ERROR(this->get_logger(), "%s: gripper goal was rejected.", label.c_str());
        return false;
      }
      if (!cv.wait_for(lock, timeout, [&] { return result_received; }))
      {
        RCLCPP_ERROR(this->get_logger(), "%s: timed out waiting for action result.", label.c_str());
        return false;
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "%s: result_code=%d, position=%.4f, effort=%.4f, stalled=%s, reached_goal=%s",
                label.c_str(), result_code, result_position, result_effort,
                result_stalled ? "true" : "false", result_reached_goal ? "true" : "false");

    if (result_success)
    {
      return true;
    }

    // Closing on a real object may stall or not fully reach the target after contact. Treat this
    // as success only for close, not for open, if the controller returned a result.
    if (closing && result_received && result_stalled)
    {
      RCLCPP_WARN(this->get_logger(), "%s: gripper stalled while closing; assuming object contact.", label.c_str());
      return true;
    }

    return false;
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

  // bool moveToPose(const geometry_msgs::msg::PoseStamped& pose)
  // {
  //   setCurrentStateForGroup(arm_group_, "moveToPose");

  //   arm_group_->setPlanningTime(arm_planning_time_);
  //   arm_group_->setMaxVelocityScalingFactor(arm_velocity_scaling_);
  //   arm_group_->setMaxAccelerationScalingFactor(arm_acceleration_scaling_);

  //   const std::string ee_link = ee_link_.empty() ? arm_group_->getEndEffectorLink() : ee_link_;
  //   arm_group_->setPoseTarget(pose, ee_link);

  //   moveit::planning_interface::MoveGroupInterface::Plan plan;
  //   auto result = arm_group_->plan(plan);
  //   if (result != moveit::core::MoveItErrorCode::SUCCESS)
  //   {
  //     RCLCPP_WARN(this->get_logger(), "Planning to grasp pose failed.");
  //     arm_group_->clearPoseTargets();
  //     return false;
  //   }

  //   auto exec = arm_group_->execute(plan);
  //   arm_group_->clearPoseTargets();
  //   if (exec != moveit::core::MoveItErrorCode::SUCCESS)
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "Execution to grasp pose failed with code %d.", static_cast<int>(exec.val));
  //     return false;
  //   }

  //   target_ps = pose;
  //   return true;
  // }

  bool moveToPose(const geometry_msgs::msg::PoseStamped& pose)
  {
    // GEN3/real robot:
    // Do NOT force a default start state if getCurrentState() fails.
    // The Kortex/ros2_control joint_states can have stamp 0.0, which makes
    // MoveIt CurrentStateMonitor complain. Falling back to a default state
    // causes execution aborts because the planned start does not match the
    // physical robot.
    //
    // Original line:
    // setCurrentStateForGroup(arm_group_, "moveToPose");

    arm_group_->setStartStateToCurrentState();

    arm_group_->setPlanningTime(arm_planning_time_);
    arm_group_->setMaxVelocityScalingFactor(arm_velocity_scaling_);
    arm_group_->setMaxAccelerationScalingFactor(arm_acceleration_scaling_);

    const std::string ee_link = ee_link_.empty() ? arm_group_->getEndEffectorLink() : ee_link_;
    arm_group_->setPoseTarget(pose, ee_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = arm_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "Planning to pose failed.");
      arm_group_->clearPoseTargets();
      return false;
    }

    auto exec = arm_group_->execute(plan);
    arm_group_->clearPoseTargets();

    if (exec != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Execution to pose failed with code %d.", static_cast<int>(exec.val));
      return false;
    }

    target_ps = pose;
    return true;
  }

  geometry_msgs::msg::PoseStamped offsetPoseAlongEndEffectorZ(
    const geometry_msgs::msg::PoseStamped& ref_pose,
    double distance)
  {
    geometry_msgs::msg::PoseStamped out = ref_pose;

    tf2::Quaternion q;
    tf2::fromMsg(out.pose.orientation, q);
    q.normalize();

    tf2::Matrix3x3 R(q);

    // Move along +Z_EE or -Z_EE depending on distance sign.
    tf2::Vector3 dz_ee(0.0, 0.0, distance);
    tf2::Vector3 dz_world = R * dz_ee;

    out.pose.position.x += dz_world.x();
    out.pose.position.y += dz_world.y();
    out.pose.position.z += dz_world.z();

    return out;
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

  // GEN3/Robotiq action mode: use a MultiThreadedExecutor so the gripper action
  // callbacks can be processed while /reach_to_grasp service callback is waiting.
  // Original line:
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
