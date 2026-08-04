#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "cgn_flexbe_utilities/srv/move_to_pose.hpp"

#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cmath>
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

    // Human-safe pregrasp behavior:
    // The service request still contains the actual grasp pose.  This node
    // moves only to a pregrasp pose that is offset backward along the requested
    // end-effector +Z axis.  /reach_to_grasp then moves forward by the same
    // distance to the actual grasp pose.
    this->declare_parameter<bool>("move_to_pregrasp_pose", true);
    this->declare_parameter<double>("pregrasp_ee_z_distance", 0.10);

    std::string group_name;
    this->get_parameter("planning_group", group_name);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("ee_link", ee_link_);
    this->get_parameter("move_to_pregrasp_pose", move_to_pregrasp_pose_);
    this->get_parameter("pregrasp_ee_z_distance", pregrasp_ee_z_distance_);

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

    service_ = this->create_service<cgn_flexbe_utilities::srv::MoveToPose>(
      "/move_to_pose",
      std::bind(&PosePlanner::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "MoveToPose service ready (group: %s)", group_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Planning frame reported by MoveIt: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End-effector link reported by MoveIt: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Debug marker/base frame parameter: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Pregrasp mode: move_to_pregrasp_pose=%s, pregrasp_ee_z_distance=%.3f m",
                move_to_pregrasp_pose_ ? "true" : "false",
                pregrasp_ee_z_distance_);
  }

private:
  rclcpp::Service<cgn_flexbe_utilities::srv::MoveToPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // GEN3/real-robot additions:
  std::string base_frame_;
  std::string ee_link_;

  // Pregrasp parameters.
  bool move_to_pregrasp_pose_ = true;
  double pregrasp_ee_z_distance_ = 0.10;

  geometry_msgs::msg::Pose offsetAlongEndEffectorZ(const geometry_msgs::msg::Pose& pose,
                                                   double distance) const
  {
    geometry_msgs::msg::Pose out = pose;

    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;

    const double norm = std::sqrt(x*x + y*y + z*z + w*w);
    if (norm > 1e-9)
    {
      x /= norm;
      y /= norm;
      z /= norm;
      w /= norm;
    }
    else
    {
      // Invalid quaternion; leave orientation untouched and use base-frame +Z.
      x = 0.0;
      y = 0.0;
      z = 0.0;
      w = 1.0;
    }

    // Third column of the quaternion rotation matrix: local +Z_EE expressed
    // in the pose/base frame.  This avoids adding extra tf2 dependencies to
    // this small service.
    const double zx = 2.0 * (x*z + w*y);
    const double zy = 2.0 * (y*z - w*x);
    const double zz = 1.0 - 2.0 * (x*x + y*y);

    out.position.x += distance * zx;
    out.position.y += distance * zy;
    out.position.z += distance * zz;

    return out;
  }

  void publishPoseMarker(const geometry_msgs::msg::Pose& pose,
                         int id,
                         const std::string& ns,
                         float r, float g, float b)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = base_frame_;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = pose;
    m.scale.x = m.scale.y = m.scale.z = 0.03;
    m.color.a = 1.0;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    marker_pub_->publish(m);
  }

  void handle_request(
    const std::shared_ptr<cgn_flexbe_utilities::srv::MoveToPose::Request> req,
    std::shared_ptr<cgn_flexbe_utilities::srv::MoveToPose::Response> res)
  {
    try
    {
      const geometry_msgs::msg::Pose grasp_pose = req->target_pose;

      geometry_msgs::msg::Pose plan_pose = grasp_pose;
      if (move_to_pregrasp_pose_ && std::abs(pregrasp_ee_z_distance_) > 1e-6)
      {
        // Back away from the requested actual grasp pose along local -Z_EE
        // when pregrasp_ee_z_distance is positive.  If the approach direction
        // is opposite on your gripper, use a negative distance in both
        // /move_to_pose and /reach_to_grasp.
        plan_pose = offsetAlongEndEffectorZ(grasp_pose, -pregrasp_ee_z_distance_);
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Received actual grasp pose: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
        grasp_pose.position.x,
        grasp_pose.position.y,
        grasp_pose.position.z,
        grasp_pose.orientation.x,
        grasp_pose.orientation.y,
        grasp_pose.orientation.z,
        grasp_pose.orientation.w);

      RCLCPP_INFO(
        this->get_logger(),
        "Planning to %s pose: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
        (move_to_pregrasp_pose_ && std::abs(pregrasp_ee_z_distance_) > 1e-6) ? "pregrasp" : "requested",
        plan_pose.position.x,
        plan_pose.position.y,
        plan_pose.position.z,
        plan_pose.orientation.x,
        plan_pose.orientation.y,
        plan_pose.orientation.z,
        plan_pose.orientation.w);

      // Publish the actual requested grasp pose and the commanded pregrasp pose.
      publishPoseMarker(grasp_pose, 0, "move_to_pose_actual_grasp", 0.1f, 0.8f, 0.2f);
      publishPoseMarker(plan_pose, 1, "move_to_pose_pregrasp", 0.2f, 0.2f, 1.0f);

      // GEN3-safe version: use explicit ee_link if provided; otherwise keep original behavior.
      if (!ee_link_.empty())
      {
        move_group_->setPoseTarget(plan_pose, ee_link_);
      }
      else
      {
        move_group_->setPoseTarget(plan_pose);
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode planning_result = move_group_->plan(plan);

      if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
        move_group_->clearPoseTargets();
        res->success = false;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");

      moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);
      move_group_->clearPoseTargets();

      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Motion to pregrasp/requested pose succeeded.");
        res->success = true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed.");
        res->success = false;
      }
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

    geometry_msgs::msg::Pose target = req->target_pose;
    target.position.z += dz;  // base frame Z

    RCLCPP_INFO(this->get_logger(), "target.position.z = %f", target.position.z);

    move_group_->setPoseTarget(target, ee_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "Planning base-Z lift failed.");
      move_group_->clearPoseTargets();
      return false;
    }

    auto exec = move_group_->execute(plan);
    move_group_->clearPoseTargets();
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
