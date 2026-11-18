/**
 * LSM Bot C++17 Topological A* Route Planner & Eigen3 EKF / LiDAR Benchmark Node
 * Package: lsm_navigation
 *
 * Role:
 *   1. High-speed C++17 206-waypoint / 208-edge Topological A* Route Planner Node
 *      using contiguous integer-indexed adjacency lists and std::priority_queue.
 *   2. Supports --benchmark CLI flag to run microsecond-level C++17 benchmarks for:
 *      - 206-Node A* Shortest Path Search (1,000 iterations)
 *      - 2D EKF Matrix Predict + Update (10,000 iterations using Eigen3)
 *      - 360-Point LaserScan Safety Sector Check (10,000 iterations)
 *      - 50Hz / 10Hz High-Resolution Timer Jitter & RSS Memory Footprint
 */

#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>

#include "lsm_navigation/topological_graph_cpp.hpp"

namespace lsm_navigation
{

namespace
{

std::string resolveGraphYamlPath(const std::string & custom_path = "")
{
  if (!custom_path.empty()) {
    std::ifstream f(custom_path);
    if (f.good()) return custom_path;
  }
  try {
    const std::string share = ament_index_cpp::get_package_share_directory("lsm_navigation");
    const std::string cand = share + "/config/waypoints_graph.yaml";
    std::ifstream f(cand);
    if (f.good()) return cand;
  } catch (...) {
  }
  return "src/lsm_navigation/config/waypoints_graph.yaml";
}

double getProcessRssMb()
{
  std::ifstream statm("/proc/self/statm");
  long total_pages = 0;
  long rss_pages = 0;
  if (statm >> total_pages >> rss_pages) {
    return static_cast<double>(rss_pages) * 4096.0 / (1024.0 * 1024.0);
  }
  return 0.0;
}

struct StatsSummary
{
  double mean_us{0.0};
  double p50_us{0.0};
  double p99_us{0.0};
  double max_us{0.0};
  double min_us{0.0};
};

StatsSummary computeStats(std::vector<double> samples)
{
  StatsSummary s;
  if (samples.empty()) return s;
  std::sort(samples.begin(), samples.end());
  const double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
  s.mean_us = sum / static_cast<double>(samples.size());
  s.min_us = samples.front();
  s.max_us = samples.back();
  s.p50_us = samples[samples.size() / 2];
  s.p99_us = samples[static_cast<std::size_t>(samples.size() * 0.99)];
  return s;
}

}  // namespace

class TopologicalRoutePlannerNode : public rclcpp::Node
{
public:
  explicit TopologicalRoutePlannerNode(const std::string & yaml_path)
  : Node("topological_route_planner_node"),
    graph_(yaml_path)
  {
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", true);
    }

    route_resp_pub_ = this->create_publisher<std_msgs::msg::String>("/fms/cpp_route_response", 10);
    route_req_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fms/cpp_route_request", 10,
      std::bind(&TopologicalRoutePlannerNode::onRouteRequest, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "[C++17 TopologicalRoutePlanner] Pre-loaded %zu waypoints and %zu directed edges from %s",
      graph_.numWaypoints(), graph_.numDirectedEdges(), yaml_path.c_str());
  }

  const TopologicalGraphCpp & graph() const { return graph_; }

private:
  void onRouteRequest(const std_msgs::msg::String::SharedPtr msg)
  {
    // Format: "start_wp->goal_wp"
    const std::string s = msg->data;
    const auto sep = s.find("->");
    if (sep == std::string::npos) return;
    const std::string start_wp = graph_.resolveStationToWaypoint(s.substr(0, sep));
    const std::string goal_wp = graph_.resolveStationToWaypoint(s.substr(sep + 2));

    const auto res = graph_.findShortestPath(start_wp, goal_wp);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "{\"start_wp\": \"" << start_wp << "\", \"goal_wp\": \"" << goal_wp << "\", "
        << "\"found\": " << (res.found ? "true" : "false") << ", "
        << "\"distance_m\": " << res.total_distance_m << ", "
        << "\"compute_time_us\": " << res.compute_time_us << ", \"path\": [";
    for (std::size_t i = 0; i < res.path.size(); ++i) {
      if (i > 0) oss << ", ";
      oss << "\"" << res.path[i] << "\"";
    }
    oss << "]}";

    std_msgs::msg::String resp;
    resp.data = oss.str();
    route_resp_pub_->publish(resp);
  }

  TopologicalGraphCpp graph_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_resp_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_req_sub_;
};

int runCppBenchmarkSuite(const std::string & yaml_path, const std::string & json_out_path)
{
  TopologicalGraphCpp graph(yaml_path);

  const std::vector<std::pair<std::string, std::string>> pairs = {
    {"wp-charge-1", "wp-ws-a-4"},
    {"wp-ws-a-4", "wp-ws-a-5"},
    {"wp-ws-a-5", "wp-charge-1"},
    {"wp-charge-1", "wp-ws-a-1"},
    {"wp-ws-a-1", "wp-ws-b-2-2"},
    {"wp-ws-b-2-2", "wp-ws-c-2"},
    {"wp-ws-c-2", "wp-charge-1"},
    {"wp-charge-1", "wp-ws-c-4"},
    {"wp-ws-c-4", "wp-ws-a-6"},
    {"wp-ws-a-6", "wp-charge-1"},
  };

  // 1. 206-Node A* Path Search Benchmark (1,000 iterations)
  std::vector<double> astar_samples_us;
  astar_samples_us.reserve(1000);
  std::size_t total_path_nodes = 0;
  for (int i = 0; i < 1000; ++i) {
    const auto & pr = pairs[i % pairs.size()];
    const auto res = graph.findShortestPath(pr.first, pr.second);
    astar_samples_us.push_back(res.compute_time_us);
    total_path_nodes += res.path.size();
  }
  const auto astar_stats = computeStats(astar_samples_us);

  // 2. 2D EKF Eigen3 Matrix Predict + Update Benchmark (10,000 iterations)
  EigenEKFCore2D ekf;
  std::vector<double> ekf_samples_us;
  ekf_samples_us.reserve(10000);
  for (int i = 0; i < 10000; ++i) {
    const double vx = 0.25 + 0.05 * std::sin(0.01 * i);
    const double vyaw = 0.40 * std::cos(0.02 * i);
    const double ax = 0.05 * std::cos(0.01 * i);
    const auto t0 = std::chrono::steady_clock::now();
    ekf.predict(0.0333);
    ekf.updateWheelAndImu(vx, vyaw, ax);
    const auto t1 = std::chrono::steady_clock::now();
    ekf_samples_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
  }
  const auto ekf_stats = computeStats(ekf_samples_us);

  // 3. 360-Point LiDAR Safety Sector Evaluation Benchmark (10,000 iterations)
  std::vector<float> mock_ranges(360, 2.5f);
  mock_ranges[10] = 0.28f;
  mock_ranges[350] = 0.31f;
  std::vector<double> scan_samples_us;
  scan_samples_us.reserve(10000);
  volatile double min_sink = 0.0;
  for (int iter = 0; iter < 10000; ++iter) {
    const auto t0 = std::chrono::steady_clock::now();
    double min_front = 10.0;
    double min_rear = 10.0;
    const double angle_min = -M_PI;
    const double angle_inc = (2.0 * M_PI) / 360.0;
    for (std::size_t i = 0; i < 360; ++i) {
      const float r = mock_ranges[i];
      if (!std::isfinite(r) || r < 0.12f || r > 8.0f) continue;
      double angle = angle_min + static_cast<double>(i) * angle_inc;
      while (angle > M_PI) angle -= 2.0 * M_PI;
      while (angle < -M_PI) angle += 2.0 * M_PI;
      if (std::abs(angle) <= 0.38) {
        min_front = std::min(min_front, static_cast<double>(r));
      } else if (std::abs(angle) >= (M_PI - 0.38)) {
        min_rear = std::min(min_rear, static_cast<double>(r));
      }
    }
    min_sink = min_front + min_rear;
    const auto t1 = std::chrono::steady_clock::now();
    scan_samples_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
  }
  (void)min_sink;
  const auto scan_stats = computeStats(scan_samples_us);

  // 4. 50Hz (20ms) rclcpp WallTimer Jitter Benchmark (150 ticks = 3.0s, event-driven spin)
  auto bench_node = std::make_shared<rclcpp::Node>("cpp_timer_jitter_bench_node");
  std::vector<double> timer_jitter_us;
  timer_jitter_us.reserve(160);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto prev_tick = std::chrono::steady_clock::now();
  bool first_tick = true;
  auto timer = bench_node->create_wall_timer(
    std::chrono::milliseconds(20),
    [&]() {
      const auto now = std::chrono::steady_clock::now();
      if (first_tick) {
        first_tick = false;
      } else {
        const double dt_us = std::chrono::duration<double, std::micro>(now - prev_tick).count();
        timer_jitter_us.push_back(std::abs(dt_us - 20000.0));
        if (timer_jitter_us.size() >= 150) {
          exec.cancel();
        }
      }
      prev_tick = now;
    });

  exec.add_node(bench_node);
  exec.spin();
  const auto jitter_stats = computeStats(timer_jitter_us);
  const double rss_mb = getProcessRssMb();

  std::ostringstream json;
  json << std::fixed << std::setprecision(3)
       << "{\n"
       << "  \"language\": \"C++17 (rclcpp + Eigen3 + STL)\",\n"
       << "  \"waypoints_loaded\": " << graph.numWaypoints() << ",\n"
       << "  \"directed_edges_loaded\": " << graph.numDirectedEdges() << ",\n"
       << "  \"total_path_nodes_verified\": " << total_path_nodes << ",\n"
       << "  \"ekf_final_x\": " << ekf.state()(0) << ",\n"
       << "  \"rss_memory_mb\": " << rss_mb << ",\n"
       << "  \"astar_1000\": {\"mean_us\": " << astar_stats.mean_us << ", \"p50_us\": " << astar_stats.p50_us
       << ", \"p99_us\": " << astar_stats.p99_us << ", \"max_us\": " << astar_stats.max_us << "},\n"
       << "  \"ekf_10000\": {\"mean_us\": " << ekf_stats.mean_us << ", \"p50_us\": " << ekf_stats.p50_us
       << ", \"p99_us\": " << ekf_stats.p99_us << ", \"max_us\": " << ekf_stats.max_us << "},\n"
       << "  \"lidar_scan_10000\": {\"mean_us\": " << scan_stats.mean_us << ", \"p50_us\": " << scan_stats.p50_us
       << ", \"p99_us\": " << scan_stats.p99_us << ", \"max_us\": " << scan_stats.max_us << "},\n"
       << "  \"timer_50hz_jitter\": {\"mean_us\": " << jitter_stats.mean_us << ", \"p50_us\": " << jitter_stats.p50_us
       << ", \"p99_us\": " << jitter_stats.p99_us << ", \"max_us\": " << jitter_stats.max_us << "}\n"
       << "}\n";

  std::cout << json.str();
  if (!json_out_path.empty()) {
    std::ofstream out(json_out_path);
    out << json.str();
  }
  return 0;
}

}  // namespace lsm_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  bool benchmark_mode = false;
  std::string yaml_path;
  std::string json_out;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--benchmark") {
      benchmark_mode = true;
    } else if (arg == "--yaml" && i + 1 < argc) {
      yaml_path = argv[++i];
    } else if (arg == "--json-out" && i + 1 < argc) {
      json_out = argv[++i];
    }
  }

  const std::string resolved_yaml = lsm_navigation::resolveGraphYamlPath(yaml_path);

  if (benchmark_mode) {
    const int rc = lsm_navigation::runCppBenchmarkSuite(resolved_yaml, json_out);
    rclcpp::shutdown();
    return rc;
  }

  auto node = std::make_shared<lsm_navigation::TopologicalRoutePlannerNode>(resolved_yaml);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
