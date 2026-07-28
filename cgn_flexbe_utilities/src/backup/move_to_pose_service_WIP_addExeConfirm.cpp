#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "cgn_flexbe_utilities/srv/move_to_pose.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <cctype>
#include <iostream>
#include <string>

class PosePlanner : public rclcpp::Node
{
public:
  PosePlanner()
    : Node("move_to_pose_service")
  {
    // Original Panda/old generic default was:
    // this->declare_parameter<std::string>("planning_group", "arm");
    // GEN3/real-robot default verified from SRDF: group name is "manipulator".
    this->declare_parameter<std::string>("planning_group", "manipulator");

    // GEN3/real-robot additions:
    // These let the same service run with the old Panda/Gazebo frames or the
    // Kinova Gen3 frames from your kinova_gen3_7dof_robotiq_2f_85 MoveIt config.
    this->declare_parameter<std::string>("base_frame", "base_link");
    // Original/no-override behavior was an empty ee_link.
    // GEN3 default verified from SRDF/view_frames: end_effector_link exists.
    this->declare_parameter<std::string>("ee_link", "end_effector_link");

    // Safety / supervision option. If true, planning succeeds first, the
    // planned trajectory is published for RViz visualization, and the node
    // waits for terminal input before executing. Type the execute_key and
    // press Enter to execute. Any other input rejects this plan.
    this->declare_parameter<bool>("confirm_before_execute", true);
    this->declare_parameter<std::string>("execute_key", "e");

    std::string group_name;
    this->get_parameter("planning_group", group_name);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("ee_link", ee_link_);
    this->get_parameter("confirm_before_execute", confirm_before_execute_);
    this->get_parameter("execute_key", execute_key_);

    if (execute_key_.empty())
    {
      execute_key_ = "e";
    }

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), group_name);

    // GEN3/real-robot addition: optionally force the end-effector link if the
    // MoveIt group default is empty or different from the grasp frame you use.
    if (!ee_link_.empty())
    {
      move_group_->setEndEffectorLink(ee_link_);
    }

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "debug_goal_marker", rclcpp::QoS(1).transient_local().reliable());

    display_trajectory_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "display_planned_path", rclcpp::QoS(1).transient_local().reliable());

    service_ = this->create_service<cgn_flexbe_utilities::srv::MoveToPose>(
      "move_to_pose",
      std::bind(&PosePlanner::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "MoveToPose service ready (group: %s)", group_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Planning frame reported by MoveIt: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End-effector link reported by MoveIt: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Debug marker/base frame parameter: %s", base_frame_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Execution confirmation: confirm_before_execute=%s, execute_key='%s'",
      confirm_before_execute_ ? "true" : "false",
      execute_key_.c_str());
  }

private:
  rclcpp::Service<cgn_flexbe_utilities::srv::MoveToPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;

  // GEN3/real-robot additions:
  std::string base_frame_;
  std::string ee_link_;

  // Human-in-the-loop execution control.
  bool confirm_before_execute_;
  std::string execute_key_;

  void publish_plan_for_review(const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    moveit_msgs::msg::DisplayTrajectory display_msg;
    display_msg.trajectory_start = plan.start_state;
    display_msg.trajectory.push_back(plan.trajectory);
    display_trajectory_pub_->publish(display_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Published planned trajectory to /display_planned_path for RViz review.");
  }

  bool user_confirm_execution()
  {
    if (!confirm_before_execute_)
    {
      return true;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Planning succeeded. Review the planned path in RViz before execution.");

    std::cout << "\n[move_to_pose_service] Type '" << execute_key_
              << "' then Enter to EXECUTE this plan. "
              << "Type anything else then Enter to reject/try next grasp: "
              << std::flush;

    std::string input;
    if (!std::getline(std::cin, input))
    {
      RCLCPP_ERROR(this->get_logger(), "Could not read terminal input. Rejecting execution.");
      return false;
    }

    // Trim simple leading/trailing whitespace.
    const auto first = input.find_first_not_of(" \t\r\n");
    const auto last = input.find_last_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
      input.clear();
    }
    else
    {
      input = input.substr(first, last - first + 1);
    }

    if (input == execute_key_)
    {
      RCLCPP_WARN(this->get_logger(), "User confirmed execution with '%s'.", execute_key_.c_str());
      return true;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "User rejected execution with input '%s'. No robot motion will be executed for this plan.",
      input.c_str());
    return false;
  }

  void handle_request(
    const std::shared_ptr<cgn_flexbe_utilities::srv::MoveToPose::Request> req,
    std::shared_ptr<cgn_flexbe_utilities::srv::MoveToPose::Response> res)
  {
    try
    {
      // Original line:
      // move_group_->setPoseTarget(req->target_pose);
      // GEN3-safe version: use explicit ee_link if provided; otherwise keep original behavior.
      if (!ee_link_.empty())
      {
        move_group_->setPoseTarget(req->target_pose, ee_link_);
      }
      else
      {
        move_group_->setPoseTarget(req->target_pose);
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Received target pose: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
        req->target_pose.position.x,
        req->target_pose.position.y,
        req->target_pose.position.z,
        req->target_pose.orientation.x,
        req->target_pose.orientation.y,
        req->target_pose.orientation.z,
        req->target_pose.orientation.w);

      visualization_msgs::msg::Marker m;
      m.header.stamp = this->now();
      // Original simulation line:
      // m.header.frame_id = "simple_pedestal";
      // m.header.frame_id = "panda_link0";
      // GEN3/real robot: make the marker frame configurable. Use base_link for Kinova.
      m.header.frame_id = base_frame_;
      m.ns = "move_to_pose_goal";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = req->target_pose;              // exactly what you're planning to
      m.scale.x = m.scale.y = m.scale.z = 0.03;
      m.color.a = 1.0; m.color.r = 0.1; m.color.g = 0.8; m.color.b = 0.2;
      marker_pub_->publish(m);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode planning_result = move_group_->plan(plan);

      if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
        res->success = false;
        return;
      }

      publish_plan_for_review(plan);

      if (!user_confirm_execution())
      {
        res->success = false;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Executing user-approved plan...");

      moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);

      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Motion to pose succeeded.");
        res->success = true; // comment out if adding additional pick and place later in this script
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed.");
        res->success = false;
      }

      // Additional step test: Lift vertically in base frame (planning frame)
    //   bool flg_liftUp = moveInBaseZ(req, 0.05);
    //   if (flg_liftUp)
    //   {
    //     res->success = true;
    //     RCLCPP_INFO(this->get_logger(), "Motion to lift object succeeded.");
    //     return;
    //   }
    //   else
    //   {
    //     res->success = false;
    //     RCLCPP_ERROR(this->get_logger(),"Failed to lift object.");
    //     return;
    //   }

    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during planning or execution: %s", e.what());
      res->success = false;
    }
  }

  bool moveInBaseZ(const std::shared_ptr<cgn_flexbe_utilities::srv::MoveToPose::Request> req,  double dz)
  {
    const std::string ee_link = move_group_->getEndEffectorLink();
    // geometry_msgs::msg::PoseStamped current = move_group_->getCurrentPose(ee_link);

    geometry_msgs::msg::Pose target = req->target_pose; // current.pose;
    target.position.z += dz;  // base frame Z

    RCLCPP_INFO(this->get_logger(), "target.position.z = %f", target.position.z);

    move_group_->setPoseTarget(target, ee_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "Planning base-Z lift failed.");
      return false;
    }

    auto exec = move_group_->execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Execution of base-Z lift failed.");
      return false;
    }

    return true;
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
