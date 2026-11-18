/**
 * LSM Bot 5-Stage Logistics AMR Finite State Machine (FSM) Node — Modern C++17 (rclcpp)
 * Package: lsm_navigation
 *
 * Implements the 5-stage AMR logistics lifecycle using lsm_interfaces in C++17:
 *   0: CHARGING          (Docked at charging station)
 *   1: MOVING_UNLOADED   (Deadheading to pickup station or returning to charger)
 *   2: WORKING_LOADING   (Waiting for cargo loading completion at pickup station)
 *   3: MOVING_LOADED     (Transporting cargo to delivery station)
 *   4: WORKING_UNLOADING (Waiting for cargo unloading completion at delivery station)
 *
 * Also coordinates with:
 *   - C++ TopologicalGraphCpp (206-node A* pre-loaded graph)
 *   - SequentialNavigator (/fms/fsm_route_cmd & /fms/route_status)
 *   - C++ 50Hz PrecisionDockingSafetyNode (/fms/docking_cmd & /fms/docking_status)
 */

#include <cmath>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "lsm_interfaces/msg/robot_status.hpp"
#include "lsm_interfaces/msg/workstation_event.hpp"
#include "lsm_interfaces/srv/trigger_mock_event.hpp"
#include "lsm_interfaces/action/dispatch_task.hpp"
#include "lsm_navigation/topological_graph_cpp.hpp"

namespace lsm_navigation
{

namespace
{

using RobotStatus = lsm_interfaces::msg::RobotStatus;
using WorkstationEvent = lsm_interfaces::msg::WorkstationEvent;
using TriggerMockEvent = lsm_interfaces::srv::TriggerMockEvent;
using DispatchTask = lsm_interfaces::action::DispatchTask;
using GoalHandleDispatchTask = rclcpp_action::ServerGoalHandle<DispatchTask>;

std::string stateToName(uint8_t s)
{
  switch (s) {
    case RobotStatus::STATE_CHARGING: return "CHARGING";
    case RobotStatus::STATE_MOVING_UNLOADED: return "MOVING_UNLOADED";
    case RobotStatus::STATE_WORKING_LOADING: return "WORKING_LOADING";
    case RobotStatus::STATE_MOVING_LOADED: return "MOVING_LOADED";
    case RobotStatus::STATE_WORKING_UNLOADING: return "WORKING_UNLOADING";
    default: return "UNKNOWN";
  }
}

std::string extractJsonString(const std::string & json, const std::string & key)
{
  const std::string pat = "\"" + key + "\"";
  auto pos = json.find(pat);
  if (pos == std::string::npos) return "";
  pos = json.find(':', pos + pat.size());
  if (pos == std::string::npos) return "";
  auto q1 = json.find('"', pos + 1);
  if (q1 == std::string::npos) return "";
  auto q2 = json.find('"', q1 + 1);
  if (q2 == std::string::npos) return "";
  return json.substr(q1 + 1, q2 - q1 - 1);
}

double extractJsonDouble(const std::string & json, const std::string & key, double def_val = 0.0)
{
  const std::string pat = "\"" + key + "\"";
  auto pos = json.find(pat);
  if (pos == std::string::npos) return def_val;
  pos = json.find(':', pos + pat.size());
  if (pos == std::string::npos) return def_val;
  pos = json.find_first_of("-0123456789.", pos + 1);
  if (pos == std::string::npos) return def_val;
  try {
    return std::stod(json.substr(pos));
  } catch (...) {
    return def_val;
  }
}

std::vector<std::string> splitArrowChain(const std::string & raw)
{
  std::vector<std::string> out;
  std::size_t start = 0;
  while (start < raw.size()) {
    const auto pos = raw.find("->", start);
    std::string token = (pos == std::string::npos) ? raw.substr(start) : raw.substr(start, pos - start);
    token.erase(0, token.find_first_not_of(" \t\r\n"));
    if (!token.empty()) {
      token.erase(token.find_last_not_of(" \t\r\n") + 1);
    }
    if (!token.empty()) {
      out.push_back(token);
    }
    if (pos == std::string::npos) break;
    start = pos + 2;
  }
  return out;
}

}  // namespace

class AMRStateMachineCppNode : public rclcpp::Node
{
public:
  AMRStateMachineCppNode()
  : Node("amr_state_machine_node")
  {
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", true);
    }
    this->declare_parameter("robot_id", std::string("LSM01"));
    this->declare_parameter("initial_battery", 98.5);
    this->declare_parameter("nav_timeout_sec", 180.0);
    this->declare_parameter("event_timeout_sec", 120.0);
    this->declare_parameter("enable_precision_docking", true);

    robot_id_ = this->get_parameter("robot_id").as_string();
    battery_level_ = this->get_parameter("initial_battery").as_double();
    nav_timeout_sec_ = this->get_parameter("nav_timeout_sec").as_double();
    event_timeout_sec_ = this->get_parameter("event_timeout_sec").as_double();
    enable_precision_docking_ = this->get_parameter("enable_precision_docking").as_bool();

    loadMapOriginAndGraph();

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    status_pub_ = this->create_publisher<RobotStatus>("/lsm01/robot_status", 10);
    ws_event_pub_ = this->create_publisher<WorkstationEvent>("/workstation/event", 10);
    route_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/fms/fsm_route_cmd", 10);
    dock_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/fms/docking_cmd", 10);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", amcl_qos,
      std::bind(&AMRStateMachineCppNode::onAmclPose, this, std::placeholders::_1),
      sub_opts);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&AMRStateMachineCppNode::onOdom, this, std::placeholders::_1),
      sub_opts);

    route_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fms/route_status", 10,
      std::bind(&AMRStateMachineCppNode::onRouteStatus, this, std::placeholders::_1),
      sub_opts);

    dock_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fms/docking_status", 10,
      std::bind(&AMRStateMachineCppNode::onDockingStatus, this, std::placeholders::_1),
      sub_opts);

    ws_event_sub_ = this->create_subscription<WorkstationEvent>(
      "/workstation/event", 10,
      std::bind(&AMRStateMachineCppNode::onWorkstationEvent, this, std::placeholders::_1),
      sub_opts);

    mock_srv_ = this->create_service<TriggerMockEvent>(
      "/workstation/trigger_mock",
      std::bind(
        &AMRStateMachineCppNode::onTriggerMock, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      cb_group_);

    action_server_ = rclcpp_action::create_server<DispatchTask>(
      this,
      "/fms/dispatch_task",
      std::bind(&AMRStateMachineCppNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AMRStateMachineCppNode::handleCancel, this, std::placeholders::_1),
      std::bind(&AMRStateMachineCppNode::handleAccepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      cb_group_);

    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AMRStateMachineCppNode::publishRobotStatus10Hz, this),
      cb_group_);

    RCLCPP_INFO(
      this->get_logger(),
      "[C++17 FSM Ready] robot_id=%s, initial_state=%s (0), loaded %zu waypoints (%zu edges), map_origin=(%.1f, %.1f)",
      robot_id_.c_str(), stateToName(current_state_).c_str(),
      graph_.numWaypoints(), graph_.numDirectedEdges(), map_origin_x_, map_origin_y_);
  }

private:
  void loadMapOriginAndGraph()
  {
    std::string share_dir;
    try {
      share_dir = ament_index_cpp::get_package_share_directory("lsm_navigation");
    } catch (...) {
      share_dir = "src/lsm_navigation";
    }

    const std::string map_yaml = share_dir + "/maps/lsm_warehouse_map.yaml";
    try {
      YAML::Node m = YAML::LoadFile(map_yaml);
      if (m["origin"] && m["origin"].IsSequence() && m["origin"].size() >= 2) {
        map_origin_x_ = m["origin"][0].as<double>();
        map_origin_y_ = m["origin"][1].as<double>();
      }
    } catch (...) {
      map_origin_x_ = -2.4;
      map_origin_y_ = -0.3;
    }

    const std::string wp_yaml = share_dir + "/config/waypoints_graph.yaml";
    try {
      graph_.loadFromYaml(wp_yaml);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Failed loading waypoints from %s: %s", wp_yaml.c_str(), e.what());
    }
  }

  void onAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pose_map_x_ = msg->pose.pose.position.x;
    pose_map_y_ = msg->pose.pose.position.y;
    pose_map_qz_ = msg->pose.pose.orientation.z;
    pose_map_qw_ = msg->pose.pose.orientation.w;
    pose_world_x_ = pose_map_x_ - map_origin_x_;
    pose_world_y_ = pose_map_y_ - map_origin_y_;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    linear_speed_ = msg->twist.twist.linear.x;
    try {
      if (tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero)) {
        const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        std::lock_guard<std::mutex> lock(state_mutex_);
        pose_map_x_ = t.transform.translation.x;
        pose_map_y_ = t.transform.translation.y;
        pose_map_qz_ = t.transform.rotation.z;
        pose_map_qw_ = t.transform.rotation.w;
        pose_world_x_ = pose_map_x_ - map_origin_x_;
        pose_world_y_ = pose_map_y_ - map_origin_y_;
      }
    } catch (...) {
    }
  }

  void transitionState(uint8_t new_state, const std::string & reason)
  {
    uint8_t old_state = 0;
    std::string task_id;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      old_state = current_state_;
      current_state_ = new_state;
      task_id = current_task_id_;
    }
    const std::string t_str = task_id.empty() ? "task=NONE" : ("task=" + task_id);
    RCLCPP_INFO(
      this->get_logger(),
      "[FSM TRANSITION] %s(%u) -> %s(%u) | %s | %s",
      stateToName(old_state).c_str(), old_state,
      stateToName(new_state).c_str(), new_state,
      t_str.c_str(), reason.c_str());
  }

  void publishRobotStatus10Hz()
  {
    uint8_t state = 0;
    std::string task_id;
    double mx = 0.0, my = 0.0, qz = 1.0, qw = 0.0, bat = 100.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = current_state_;
      task_id = current_task_id_;
      if (state == RobotStatus::STATE_CHARGING) {
        battery_level_ = std::min(100.0, battery_level_ + 0.02);
      } else if (state == RobotStatus::STATE_MOVING_UNLOADED || state == RobotStatus::STATE_MOVING_LOADED) {
        battery_level_ = std::max(10.0, battery_level_ - 0.005);
      } else {
        battery_level_ = std::max(10.0, battery_level_ - 0.002);
      }
      mx = pose_map_x_;
      my = pose_map_y_;
      qz = pose_map_qz_;
      qw = pose_map_qw_;
      bat = battery_level_;
    }

    RobotStatus msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.robot_id = robot_id_;
    msg.current_state = state;
    msg.pose.position.x = mx;
    msg.pose.position.y = my;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.z = qz;
    msg.pose.orientation.w = qw;
    msg.battery_level = static_cast<float>(std::round(bat * 100.0) / 100.0);
    msg.current_task_id = task_id;
    status_pub_->publish(msg);
  }

  void onRouteStatus(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string event = extractJsonString(msg->data, "event");
    const std::string goal_wp = extractJsonString(msg->data, "goal_wp");
    std::lock_guard<std::mutex> lock(cv_mutex_);
    if (!nav_target_wp_.empty() && !goal_wp.empty() && goal_wp != nav_target_wp_) {
      return;
    }
    if (event == "STARTED") {
      nav_started_ = true;
      cv_.notify_all();
    } else if (event == "COMPLETED") {
      nav_done_ = true;
      nav_success_ = true;
      cv_.notify_all();
    } else if (event == "FAILED" || event == "CANCELLED") {
      nav_done_ = true;
      nav_success_ = false;
      cv_.notify_all();
    }
  }

  void onDockingStatus(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string event = extractJsonString(msg->data, "event");
    const std::string wp_id = extractJsonString(msg->data, "wp_id");
    if (event == "DOCKED") {
      std::lock_guard<std::mutex> lock(cv_mutex_);
      if (dock_target_wp_.empty() || wp_id == dock_target_wp_) {
        last_dock_initial_err_m_ = extractJsonDouble(msg->data, "initial_err_m", 0.29);
        last_dock_final_err_m_ = extractJsonDouble(msg->data, "final_err_m", 0.015);
        last_dock_yaw_err_deg_ = extractJsonDouble(msg->data, "final_yaw_err_deg", 0.5);
        dock_done_ = true;
        cv_.notify_all();
      }
    }
  }

  void onWorkstationEvent(const WorkstationEvent::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cv_mutex_);
    if (msg->event_type == WorkstationEvent::EVENT_CARGO_LOADED) {
      cargo_loaded_ = true;
      cv_.notify_all();
    } else if (msg->event_type == WorkstationEvent::EVENT_CARGO_UNLOADED) {
      cargo_unloaded_ = true;
      cv_.notify_all();
    }
  }

  void onTriggerMock(
    const std::shared_ptr<TriggerMockEvent::Request> request,
    std::shared_ptr<TriggerMockEvent::Response> response)
  {
    std::string station = request->station_id;
    std::string task_id;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (station.empty() || station == "ACTIVE") {
        station = active_station_id_;
      }
      task_id = current_task_id_;
    }

    WorkstationEvent ev;
    ev.header.stamp = this->get_clock()->now();
    ev.header.frame_id = "map";
    ev.station_id = station;
    ev.event_type = request->event_type;
    ev.task_id = task_id;

    if (request->event_type == TriggerMockEvent::Request::EVENT_CARGO_LOADED) {
      ws_event_pub_->publish(ev);
      {
        std::lock_guard<std::mutex> lock(cv_mutex_);
        cargo_loaded_ = true;
      }
      cv_.notify_all();
      response->success = true;
      response->message = "Mock CARGO_LOADED event triggered for station " + station;
      RCLCPP_INFO(
        this->get_logger(),
        "[WorkstationEvent] CARGO_LOADED @ %s (task=%s)", station.c_str(), task_id.c_str());
    } else if (request->event_type == TriggerMockEvent::Request::EVENT_CARGO_UNLOADED) {
      ws_event_pub_->publish(ev);
      {
        std::lock_guard<std::mutex> lock(cv_mutex_);
        cargo_unloaded_ = true;
      }
      cv_.notify_all();
      response->success = true;
      response->message = "Mock CARGO_UNLOADED event triggered for station " + station;
      RCLCPP_INFO(
        this->get_logger(),
        "[WorkstationEvent] CARGO_UNLOADED @ %s (task=%s)", station.c_str(), task_id.c_str());
    } else if (request->event_type == WorkstationEvent::EVENT_JOB_DONE) {
      ws_event_pub_->publish(ev);
      response->success = true;
      response->message = "Mock JOB_DONE event triggered for station " + station;
    } else {
      response->success = false;
      response->message = "Unsupported event_type";
    }
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const DispatchTask::Goal> goal)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!current_task_id_.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "[DispatchTask] Rejecting concurrent goal '%s' while '%s' is active",
          goal->task_id.c_str(), current_task_id_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[DispatchTask] Received goal: task_id='%s', type=%u, target='%s'",
      goal->task_id.c_str(), goal->task_type, goal->target_station_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleDispatchTask>)
  {
    RCLCPP_WARN(this->get_logger(), "[DispatchTask] Cancel requested; notifying SequentialNavigator & PrecisionDocking");
    sendRouteCancel("");
    sendDockCancel();
    cv_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleDispatchTask> goal_handle)
  {
    std::thread{std::bind(&AMRStateMachineCppNode::executeDispatchTask, this, goal_handle)}.detach();
  }

  void sendRouteCancel(const std::string & target_wp)
  {
    std_msgs::msg::String msg;
    msg.data = "{\"command\": \"cancel\", \"goal_wp\": \"" + target_wp + "\"}";
    route_cmd_pub_->publish(msg);
  }

  void sendDockCancel()
  {
    std_msgs::msg::String msg;
    msg.data = "{\"command\": \"cancel\"}";
    dock_cmd_pub_->publish(msg);
  }

  double computeDistanceToWp(const std::string & wp_id)
  {
    WaypointNode wp;
    if (!graph_.getWaypoint(wp_id, wp)) {
      return 0.0;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    return std::hypot(wp.x - pose_world_x_, wp.y - pose_world_y_);
  }

  void publishFeedback(const std::shared_ptr<GoalHandleDispatchTask> & goal_handle, const std::string & target_wp)
  {
    auto fb = std::make_shared<DispatchTask::Feedback>();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      fb->current_state = current_state_;
    }
    const double dist = computeDistanceToWp(target_wp);
    fb->distance_remaining = static_cast<float>(dist);
    const double spd = std::max(std::abs(linear_speed_), 0.22);
    fb->estimated_time_remaining = static_cast<float>(dist / spd);
    goal_handle->publish_feedback(fb);
  }

  void runFineDockingIfEnabled(const std::string & target_wp)
  {
    if (!enable_precision_docking_) {
      return;
    }
    WaypointNode wp;
    if (!graph_.getWaypoint(target_wp, wp)) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(cv_mutex_);
      dock_done_ = false;
      dock_target_wp_ = target_wp;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "{\"command\": \"dock\", \"wp_id\": \"" << target_wp << "\", "
        << "\"world_x\": " << wp.x << ", \"world_y\": " << wp.y << ", "
        << "\"yaw_deg\": " << wp.yaw_deg << "}";
    std_msgs::msg::String cmd;
    cmd.data = oss.str();
    dock_cmd_pub_->publish(cmd);

    std::unique_lock<std::mutex> lock(cv_mutex_);
    cv_.wait_for(lock, std::chrono::milliseconds(10500), [this]() { return dock_done_ || !rclcpp::ok(); });
  }

  bool navigateToWaypoint(
    const std::shared_ptr<GoalHandleDispatchTask> & goal_handle,
    const std::string & target_wp)
  {
    {
      std::lock_guard<std::mutex> lock(cv_mutex_);
      nav_started_ = false;
      nav_done_ = false;
      nav_success_ = false;
      nav_target_wp_ = target_wp;
    }

    std::string task_id;
    uint8_t st = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_id = current_task_id_;
      st = current_state_;
    }

    std::ostringstream oss;
    oss << "{\"command\": \"navigate\", \"task_id\": \"" << task_id << "\", "
        << "\"goal_wp\": \"" << target_wp << "\", \"fsm_state\": " << static_cast<int>(st) << "}";
    std_msgs::msg::String cmd_msg;
    cmd_msg.data = oss.str();
    route_cmd_pub_->publish(cmd_msg);
    RCLCPP_INFO(this->get_logger(), "[FSM -> SequentialNavigator] Requested A* route to '%s'", target_wp.c_str());

    const auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        sendRouteCancel(target_wp);
        return false;
      }
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed > nav_timeout_sec_) {
        RCLCPP_ERROR(this->get_logger(), "Navigation to '%s' timed out after %.1fs", target_wp.c_str(), nav_timeout_sec_);
        sendRouteCancel(target_wp);
        return false;
      }

      publishFeedback(goal_handle, target_wp);

      {
        std::unique_lock<std::mutex> lock(cv_mutex_);
        if (cv_.wait_for(lock, std::chrono::milliseconds(400), [this]() { return nav_done_; })) {
          break;
        }
      }
    }

    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(cv_mutex_);
      ok = nav_success_;
    }
    if (ok) {
      runFineDockingIfEnabled(target_wp);
    }
    return ok;
  }

  bool waitForWorkstationEvent(
    const std::shared_ptr<GoalHandleDispatchTask> & goal_handle,
    bool & event_flag,
    const std::string & target_wp,
    const std::string & label)
  {
    const auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        return false;
      }
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed > event_timeout_sec_) {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for %s at '%s'", label.c_str(), target_wp.c_str());
        return false;
      }
      publishFeedback(goal_handle, target_wp);
      {
        std::unique_lock<std::mutex> lock(cv_mutex_);
        if (cv_.wait_for(lock, std::chrono::milliseconds(400), [&]() { return event_flag; })) {
          return true;
        }
      }
    }
    return false;
  }

  void executeDispatchTask(const std::shared_ptr<GoalHandleDispatchTask> goal_handle)
  {
    const auto req = goal_handle->get_goal();
    const auto start_time = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_task_id_ = req->task_id.empty() ? "TASK_CPP_001" : req->task_id;
    }

    auto result = std::make_shared<DispatchTask::Result>();
    const std::string raw_target = req->target_station_id;

    try {
      if (raw_target.find("->") != std::string::npos) {
        const auto parts_raw = splitArrowChain(raw_target);
        const std::string pickup_wp = parts_raw.size() > 0 ? graph_.resolveStationToWaypoint(parts_raw[0]) : "wp-ws-a-4";
        const std::string delivery_wp = parts_raw.size() > 1 ? graph_.resolveStationToWaypoint(parts_raw[1]) : "wp-ws-a-5";
        const std::string return_wp = parts_raw.size() > 2 ? graph_.resolveStationToWaypoint(parts_raw[2]) : "wp-charge-1";

        RCLCPP_INFO(
          this->get_logger(),
          "[E2E Mission Start] %s: %s -> %s -> %s",
          req->task_id.c_str(), pickup_wp.c_str(), delivery_wp.c_str(), return_wp.c_str());

        // Leg 1: CHARGING (0) -> MOVING_UNLOADED (1) -> Pickup
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          active_station_id_ = pickup_wp;
        }
        transitionState(RobotStatus::STATE_MOVING_UNLOADED, "Dispatched to pickup " + pickup_wp);
        if (!navigateToWaypoint(goal_handle, pickup_wp)) {
          throw std::runtime_error("Failed navigating to pickup station " + pickup_wp);
        }

        // Stage 2: WORKING_LOADING (2)
        {
          std::lock_guard<std::mutex> lock(cv_mutex_);
          cargo_loaded_ = false;
        }
        transitionState(RobotStatus::STATE_WORKING_LOADING, "Arrived at pickup " + pickup_wp + "; awaiting CARGO_LOADED");
        if (!waitForWorkstationEvent(goal_handle, cargo_loaded_, pickup_wp, "CARGO_LOADED")) {
          throw std::runtime_error("Cancelled or timed out waiting for CARGO_LOADED");
        }

        // Leg 2: MOVING_LOADED (3) -> Delivery
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          active_station_id_ = delivery_wp;
        }
        transitionState(RobotStatus::STATE_MOVING_LOADED, "Cargo loaded; transporting to " + delivery_wp);
        if (!navigateToWaypoint(goal_handle, delivery_wp)) {
          throw std::runtime_error("Failed navigating to delivery station " + delivery_wp);
        }

        // Stage 4: WORKING_UNLOADING (4)
        {
          std::lock_guard<std::mutex> lock(cv_mutex_);
          cargo_unloaded_ = false;
        }
        transitionState(RobotStatus::STATE_WORKING_UNLOADING, "Arrived at delivery " + delivery_wp + "; awaiting CARGO_UNLOADED");
        if (!waitForWorkstationEvent(goal_handle, cargo_unloaded_, delivery_wp, "CARGO_UNLOADED")) {
          throw std::runtime_error("Cancelled or timed out waiting for CARGO_UNLOADED");
        }

        // Leg 3: MOVING_UNLOADED (1) -> Return to Charger -> CHARGING (0)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          active_station_id_ = return_wp;
        }
        transitionState(RobotStatus::STATE_MOVING_UNLOADED, "Cargo unloaded; returning to charger " + return_wp);
        if (!navigateToWaypoint(goal_handle, return_wp)) {
          throw std::runtime_error("Failed returning to charger " + return_wp);
        }

        transitionState(RobotStatus::STATE_CHARGING, "Docked at " + return_wp + "; cycle complete");
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          current_task_id_.clear();
        }
        const double total_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        result->success = true;
        result->total_execution_time = static_cast<float>(total_sec);
        result->message = "Full 5-stage E2E logistics mission completed (" + pickup_wp + " -> " + delivery_wp + " -> " + return_wp + ")";
        goal_handle->succeed(result);
        return;
      }

      // Single-stage mode
      const std::string target_wp = graph_.resolveStationToWaypoint(raw_target);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_station_id_ = target_wp;
      }

      if (req->task_type == DispatchTask::Goal::TASK_TYPE_PICKUP) {
        transitionState(RobotStatus::STATE_MOVING_UNLOADED, "Pickup task to " + target_wp);
        if (!navigateToWaypoint(goal_handle, target_wp)) {
          throw std::runtime_error("Pickup navigation failed for " + target_wp);
        }
        transitionState(RobotStatus::STATE_WORKING_LOADING, "Arrived at pickup " + target_wp);
        result->message = "Arrived at pickup " + target_wp + "; state=WORKING_LOADING";
      } else if (req->task_type == DispatchTask::Goal::TASK_TYPE_DELIVERY) {
        transitionState(RobotStatus::STATE_MOVING_LOADED, "Delivery task to " + target_wp);
        if (!navigateToWaypoint(goal_handle, target_wp)) {
          throw std::runtime_error("Delivery navigation failed for " + target_wp);
        }
        transitionState(RobotStatus::STATE_WORKING_UNLOADING, "Arrived at delivery " + target_wp);
        result->message = "Arrived at delivery " + target_wp + "; state=WORKING_UNLOADING";
      } else if (req->task_type == DispatchTask::Goal::TASK_TYPE_RETURN_HOME) {
        transitionState(RobotStatus::STATE_MOVING_UNLOADED, "Return home task to " + target_wp);
        if (!navigateToWaypoint(goal_handle, target_wp)) {
          throw std::runtime_error("Return home navigation failed for " + target_wp);
        }
        transitionState(RobotStatus::STATE_CHARGING, "Docked at " + target_wp);
        result->message = "Returned to " + target_wp + "; state=CHARGING";
      } else {
        throw std::runtime_error("Unsupported task_type");
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_task_id_.clear();
      }
      const double total_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      result->success = true;
      result->total_execution_time = static_cast<float>(total_sec);
      goal_handle->succeed(result);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(this->get_logger(), "[DispatchTask Failed] %s", exc.what());
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_task_id_.clear();
      }
      const double total_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      result->success = false;
      result->message = exc.what();
      result->total_execution_time = static_cast<float>(total_sec);
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    }
  }

  std::string robot_id_{"LSM01"};
  double battery_level_{98.5};
  double nav_timeout_sec_{180.0};
  double event_timeout_sec_{120.0};
  bool enable_precision_docking_{true};

  double map_origin_x_{-2.4};
  double map_origin_y_{-0.3};
  TopologicalGraphCpp graph_;

  std::mutex state_mutex_;
  uint8_t current_state_{RobotStatus::STATE_CHARGING};
  std::string current_task_id_;
  std::string active_station_id_{"wp-charge-1"};
  double pose_map_x_{2.4};
  double pose_map_y_{0.3};
  double pose_map_qz_{1.0};
  double pose_map_qw_{0.0};
  double pose_world_x_{4.8};
  double pose_world_y_{0.6};
  double linear_speed_{0.0};

  std::mutex cv_mutex_;
  std::condition_variable cv_;
  bool nav_started_{false};
  bool nav_done_{false};
  bool nav_success_{false};
  std::string nav_target_wp_;

  bool dock_done_{false};
  std::string dock_target_wp_;
  double last_dock_initial_err_m_{0.0};
  double last_dock_final_err_m_{0.0};
  double last_dock_yaw_err_deg_{0.0};

  bool cargo_loaded_{false};
  bool cargo_unloaded_{false};

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<WorkstationEvent>::SharedPtr ws_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dock_cmd_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_status_sub_;
  rclcpp::Subscription<WorkstationEvent>::SharedPtr ws_event_sub_;

  rclcpp::Service<TriggerMockEvent>::SharedPtr mock_srv_;
  rclcpp_action::Server<DispatchTask>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace lsm_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lsm_navigation::AMRStateMachineCppNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
