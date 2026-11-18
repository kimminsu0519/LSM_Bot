#ifndef LSM_NAVIGATION__TOPOLOGICAL_GRAPH_CPP_HPP_
#define LSM_NAVIGATION__TOPOLOGICAL_GRAPH_CPP_HPP_

#include <cmath>
#include <chrono>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace lsm_navigation
{

struct WaypointNode
{
  int index{0};
  std::string id;
  std::string type{"passage"};
  std::string status{"active"};
  double x{0.0};
  double y{0.0};
  double yaw_deg{0.0};
};

struct GraphEdge
{
  int target_idx{0};
  double distance_m{1.0};
  std::string direction{"one_way"};
};

struct RouteResult
{
  bool found{false};
  std::vector<std::string> path;
  double total_distance_m{0.0};
  double compute_time_us{0.0};
};

class TopologicalGraphCpp
{
public:
  TopologicalGraphCpp() = default;

  explicit TopologicalGraphCpp(const std::string & yaml_path)
  {
    loadFromYaml(yaml_path);
  }

  void loadFromYaml(const std::string & yaml_path)
  {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node wps = root["waypoints"];
    if (!wps || !wps.IsMap()) {
      throw std::runtime_error("Invalid waypoints_graph.yaml: missing 'waypoints' map");
    }

    nodes_.clear();
    adj_.clear();
    id_to_idx_.clear();

    int idx = 0;
    for (auto it = wps.begin(); it != wps.end(); ++it) {
      WaypointNode node;
      node.index = idx;
      node.id = it->first.as<std::string>();
      const YAML::Node & wp = it->second;
      if (wp["type"]) {
        node.type = wp["type"].as<std::string>();
      }
      if (wp["status"]) {
        node.status = wp["status"].as<std::string>();
      }
      if (wp["x"]) {
        node.x = wp["x"].as<double>();
      } else if (wp["user_pose"] && wp["user_pose"]["x"]) {
        node.x = wp["user_pose"]["x"].as<double>();
      }
      if (wp["y"]) {
        node.y = wp["y"].as<double>();
      } else if (wp["user_pose"] && wp["user_pose"]["y"]) {
        node.y = wp["user_pose"]["y"].as<double>();
      }
      if (wp["yaw_deg"]) {
        node.yaw_deg = wp["yaw_deg"].as<double>();
      } else if (wp["user_pose"] && wp["user_pose"]["yaw_deg"]) {
        node.yaw_deg = wp["user_pose"]["yaw_deg"].as<double>();
      }

      id_to_idx_[node.id] = idx;
      nodes_.push_back(node);
      ++idx;
    }

    adj_.resize(nodes_.size());
    total_directed_edges_ = 0;

    for (auto it = wps.begin(); it != wps.end(); ++it) {
      const std::string src_id = it->first.as<std::string>();
      const int src_idx = id_to_idx_.at(src_id);
      const YAML::Node & conn = it->second["connected_to"];
      if (!conn || !conn.IsSequence()) {
        continue;
      }
      for (std::size_t i = 0; i < conn.size(); ++i) {
        const YAML::Node & e = conn[i];
        if (!e["target"]) {
          continue;
        }
        const std::string tgt_id = e["target"].as<std::string>();
        auto tgt_it = id_to_idx_.find(tgt_id);
        if (tgt_it == id_to_idx_.end()) {
          continue;
        }
        const int tgt_idx = tgt_it->second;
        const double dist = e["distance_m"] ? e["distance_m"].as<double>() : 1.0;
        std::string dir = e["direction"] ? e["direction"].as<std::string>() : "one_way";
        std::transform(dir.begin(), dir.end(), dir.begin(), ::tolower);

        addDirectedEdgeIfAbsent(src_idx, tgt_idx, dist, dir);
        if (dir == "bidirectional" || dir == "two_way" || dir == "both" || dir == "bi") {
          addDirectedEdgeIfAbsent(tgt_idx, src_idx, dist, dir);
        }
      }
    }

    total_directed_edges_ = 0;
    for (const auto & list : adj_) {
      total_directed_edges_ += list.size();
    }
  }

  RouteResult findShortestPath(const std::string & start_id, const std::string & goal_id) const
  {
    const auto t0 = std::chrono::steady_clock::now();
    RouteResult res;

    auto it_s = id_to_idx_.find(start_id);
    auto it_g = id_to_idx_.find(goal_id);
    if (it_s == id_to_idx_.end() || it_g == id_to_idx_.end()) {
      return res;
    }

    const int start_idx = it_s->second;
    const int goal_idx = it_g->second;
    const std::size_t n = nodes_.size();
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<double> g_score(n, INF);
    std::vector<int> came_from(n, -1);
    std::vector<uint8_t> closed(n, 0);

    using PQItem = std::pair<double, int>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open_set;

    g_score[start_idx] = 0.0;
    open_set.push({heuristic(start_idx, goal_idx), start_idx});

    while (!open_set.empty()) {
      const auto [f_curr, curr] = open_set.top();
      (void)f_curr;
      open_set.pop();

      if (curr == goal_idx) {
        res.found = true;
        res.total_distance_m = g_score[goal_idx];
        std::vector<std::string> rev_path;
        int cur = goal_idx;
        while (cur != -1) {
          rev_path.push_back(nodes_[cur].id);
          cur = came_from[cur];
        }
        std::reverse(rev_path.begin(), rev_path.end());
        res.path = std::move(rev_path);
        break;
      }

      if (closed[curr]) {
        continue;
      }
      closed[curr] = 1;

      for (const auto & edge : adj_[curr]) {
        if (closed[edge.target_idx]) {
          continue;
        }
        const double tentative_g = g_score[curr] + edge.distance_m;
        if (tentative_g < g_score[edge.target_idx]) {
          came_from[edge.target_idx] = curr;
          g_score[edge.target_idx] = tentative_g;
          const double f_next = tentative_g + heuristic(edge.target_idx, goal_idx);
          open_set.push({f_next, edge.target_idx});
        }
      }
    }

    const auto t1 = std::chrono::steady_clock::now();
    res.compute_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    return res;
  }

  std::string findNearestWaypoint(double world_x, double world_y) const
  {
    std::string best_id = "wp-charge-1";
    double best_dist = std::numeric_limits<double>::infinity();
    for (const auto & n : nodes_) {
      const double d = std::hypot(n.x - world_x, n.y - world_y);
      if (d < best_dist) {
        best_dist = d;
        best_id = n.id;
      }
    }
    return best_id;
  }

  std::string resolveStationToWaypoint(const std::string & raw_station) const
  {
    std::string s = raw_station;
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    if (!s.empty()) {
      s.erase(s.find_last_not_of(" \t\r\n") + 1);
    }
    if (s.empty()) {
      return "wp-charge-1";
    }
    if (id_to_idx_.count(s)) {
      return s;
    }

    std::string upper = s;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    std::string lower = s;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (upper == "CHARGE-1" || upper == "CHARGE" || upper == "CHARGER" || upper == "HOME") {
      return "wp-charge-1";
    }
    if (upper.rfind("A-", 0) == 0 && upper.size() == 3) {
      std::string cand = "wp-ws-a-" + lower.substr(2);
      if (id_to_idx_.count(cand)) return cand;
    }
    if (upper.rfind("C-", 0) == 0 && upper.size() == 3) {
      std::string cand = "wp-ws-c-" + lower.substr(2);
      if (id_to_idx_.count(cand)) return cand;
    }
    if (upper.rfind("B-", 0) == 0) {
      std::string cand = "wp-ws-" + lower;
      if (id_to_idx_.count(cand)) return cand;
    }
    if (upper.rfind("WS-", 0) == 0) {
      std::string cand = "wp-" + lower;
      if (id_to_idx_.count(cand)) return cand;
    }
    return s;
  }

  bool getWaypoint(const std::string & id, WaypointNode & out) const
  {
    auto it = id_to_idx_.find(id);
    if (it == id_to_idx_.end()) {
      return false;
    }
    out = nodes_[it->second];
    return true;
  }

  std::size_t numWaypoints() const { return nodes_.size(); }
  std::size_t numDirectedEdges() const { return total_directed_edges_; }
  const std::vector<WaypointNode> & nodes() const { return nodes_; }

private:
  void addDirectedEdgeIfAbsent(int u, int v, double dist, const std::string & dir)
  {
    for (const auto & e : adj_[u]) {
      if (e.target_idx == v) {
        return;
      }
    }
    adj_[u].push_back(GraphEdge{v, dist, dir});
  }

  double heuristic(int idx_a, int idx_b) const
  {
    return std::hypot(nodes_[idx_a].x - nodes_[idx_b].x, nodes_[idx_a].y - nodes_[idx_b].y);
  }

  std::vector<WaypointNode> nodes_;
  std::vector<std::vector<GraphEdge>> adj_;
  std::unordered_map<std::string, int> id_to_idx_;
  std::size_t total_directed_edges_{0};
};

/**
 * Fixed-size Eigen3 2D Extended Kalman Filter Core (State: [x, y, yaw, vx, vyaw, ax]^T)
 * Demonstrates high-frequency C++17 Eigen matrix prediction & Joseph-form covariance update.
 */
class EigenEKFCore2D
{
public:
  using StateVec = Eigen::Matrix<double, 6, 1>;
  using CovMat = Eigen::Matrix<double, 6, 6>;

  EigenEKFCore2D()
  {
    x_.setZero();
    P_ = CovMat::Identity() * 1e-6;
    Q_ = CovMat::Zero();
    Q_.diagonal() << 0.05, 0.05, 0.02, 0.05, 0.04, 0.001;
  }

  void predict(double dt)
  {
    const double yaw = x_(2);
    const double vx = x_(3);
    const double vyaw = x_(4);
    const double ax = x_(5);

    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double v_eff = vx + 0.5 * ax * dt;

    x_(0) += v_eff * c * dt;
    x_(1) += v_eff * s * dt;
    x_(2) += vyaw * dt;
    x_(3) += ax * dt;

    CovMat F = CovMat::Identity();
    F(0, 2) = -v_eff * s * dt;
    F(0, 3) = c * dt;
    F(0, 5) = 0.5 * c * dt * dt;
    F(1, 2) = v_eff * c * dt;
    F(1, 3) = s * dt;
    F(1, 5) = 0.5 * s * dt * dt;
    F(2, 4) = dt;
    F(3, 5) = dt;

    P_ = F * P_ * F.transpose() + Q_ * dt;
  }

  void updateWheelAndImu(double meas_vx, double meas_vyaw, double meas_ax)
  {
    // Measurement vector z = [vx, vyaw, ax]^T (6-axis gyro-rate + wheel vx fusion, yaw=false)
    Eigen::Matrix<double, 3, 1> z;
    z << meas_vx, meas_vyaw, meas_ax;

    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 3) = 1.0;  // vx
    H(1, 4) = 1.0;  // vyaw
    H(2, 5) = 1.0;  // ax

    Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();
    R(0, 0) = 2.5e-3;
    R(1, 1) = 6.4e-5;
    R(2, 2) = 2.89e-4;

    const Eigen::Matrix<double, 3, 1> y = z - H * x_;
    const Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    x_ += K * y;
    const CovMat I = CovMat::Identity();
    const CovMat I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  }

  const StateVec & state() const { return x_; }
  const CovMat & covariance() const { return P_; }

private:
  StateVec x_;
  CovMat P_;
  CovMat Q_;
};

}  // namespace lsm_navigation

#endif  // LSM_NAVIGATION__TOPOLOGICAL_GRAPH_CPP_HPP_
