#!/usr/bin/env python3
"""
LSM FMS Topological Route Server & A* Path Planner
Author: Antigravity AI
Package: lsm_fms_web

Pre-loads single-source 'waypoints_graph2.yaml' map data and builds an in-memory A* graph.
Calculates the optimal topological route sequence between any start_wp and goal_wp.
"""

import os
import sys
import yaml
import math
import heapq
import argparse

class TopologicalGraph:
    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self.waypoints = {}
        self.adj = {}  # node_id -> list of (target_id, distance_m, direction)
        self.load_graph()

    def load_graph(self):
        if not os.path.exists(self.yaml_path):
            raise FileNotFoundError(f"YAML graph file not found: {self.yaml_path}")
        
        with open(self.yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        self.waypoints = data.get('waypoints', {})
        self.adj = {}

        for wp_id, wp in self.waypoints.items():
            self.adj[wp_id] = []
            pos = wp.get('user_pose', {})
            wp['x'] = float(wp.get('x', pos.get('x', 0.0)))
            wp['y'] = float(wp.get('y', pos.get('y', 0.0)))
            wp['yaw_deg'] = float(wp.get('yaw_deg', pos.get('yaw_deg', 0.0)))

        # Build adjacency graph
        for src_id, wp in self.waypoints.items():
            for edge in wp.get('connected_to', []):
                tgt_id = edge['target']
                dist = edge.get('distance_m', 1.0)
                direction = (edge.get('direction') or 'one_way').lower()

                if tgt_id in self.waypoints:
                    if not any(t == tgt_id for t, _, _ in self.adj[src_id]):
                        self.adj[src_id].append((tgt_id, dist, direction))
                    if direction in ['bidirectional', 'two_way', 'both', 'bi']:
                        if not any(t == src_id for t, _, _ in self.adj[tgt_id]):
                            self.adj[tgt_id].append((src_id, dist, direction))

        total_edges = sum(len(v) for v in self.adj.values())
        print(f"[TopologicalGraph] Successfully pre-loaded {len(self.waypoints)} waypoints and {total_edges} directed edges into memory graph.")

    def heuristic(self, node_a, node_b):
        """Euclidean distance heuristic for A*"""
        wp_a = self.waypoints[node_a]
        wp_b = self.waypoints[node_b]
        return math.hypot(wp_b['x'] - wp_a['x'], wp_b['y'] - wp_a['y'])

    def find_shortest_path(self, start_wp, goal_wp):
        """
        A* Algorithm to find the shortest topological route from start_wp to goal_wp.
        Returns:
            path: list of waypoint IDs [start_wp, ..., goal_wp]
            total_distance: float in meters
        """
        if start_wp not in self.waypoints:
            print(f"[Error] Start waypoint '{start_wp}' not found in graph.")
            return None, 0.0
        if goal_wp not in self.waypoints:
            print(f"[Error] Goal waypoint '{goal_wp}' not found in graph.")
            return None, 0.0

        if start_wp == goal_wp:
            return [start_wp], 0.0

        pq = [(self.heuristic(start_wp, goal_wp), start_wp)]
        g_score = {node: float('inf') for node in self.waypoints}
        g_score[start_wp] = 0.0
        came_from = {}
        visited = set()

        while pq:
            current_f, current = heapq.heappop(pq)

            if current in visited:
                continue
            visited.add(current)

            if current == goal_wp:
                # Reconstruct path
                path = []
                curr = goal_wp
                while curr in came_from:
                    path.append(curr)
                    curr = came_from[curr]
                path.append(start_wp)
                path.reverse()
                return path, round(g_score[goal_wp], 2)

            for neighbor, weight, _ in self.adj[current]:
                if neighbor in visited:
                    continue
                tentative_g = g_score[current] + weight
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_wp)
                    heapq.heappush(pq, (f_score, neighbor))

        print(f"[A* Planner] No path found between '{start_wp}' and '{goal_wp}'.")
        return None, 0.0

    def get_route_details(self, path):
        """Extract full route telemetry for each step in path"""
        if not path:
            return []
        
        details = []
        for i, wp_id in enumerate(path):
            wp = self.waypoints[wp_id]
            next_wp = path[i+1] if i + 1 < len(path) else None
            
            # Find next choices (branches) at this node
            branches = [edge[0] for edge in self.adj[wp_id]]
            
            details.append({
                'step': i,
                'waypoint_id': wp_id,
                'type': wp['type'],
                'x': wp['x'],
                'y': wp['y'],
                'yaw_deg': wp['yaw_deg'],
                'next_waypoint': next_wp,
                'branch_choices': branches
            })
        return details

def main():
    parser = argparse.ArgumentParser(description="LSM FMS Topological Route Server")
    parser.add_argument('--start', type=str, default='wp-charge-1', help="Start Waypoint ID")
    parser.add_argument('--goal', type=str, default='wp-ws-a-1', help="Goal Waypoint ID")
    parser.add_argument('--yaml', type=str, default='', help="Path to waypoints_graph.yaml")

    args = parser.parse_args()

    # Default YAML path if not specified
    if not args.yaml:
        default_yaml = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../lsm_navigation/config/waypoints_graph.yaml'))
        if not os.path.exists(default_yaml):
            default_yaml = os.path.abspath(os.path.join(os.path.dirname(__file__), '../web/config/waypoints_graph.yaml'))
        args.yaml = default_yaml

    print(f"=== LSM FMS Topological Route Server ===")
    print(f"Using Graph Data: {args.yaml}")

    graph = TopologicalGraph(args.yaml)

    print(f"\n[Route Request] Start: '{args.start}' --> Goal: '{args.goal}'")
    path, dist = graph.find_shortest_path(args.start, args.goal)

    if path:
        print(f"\n[A* Path Found] Total Distance: {dist:.2f} meters ({len(path)} waypoints)")
        print(f"Path Sequence: {' -> '.join(path)}")

        print("\n--- Step-by-Step Route Telemetry ---")
        details = graph.get_route_details(path)
        for d in details:
            next_str = f"--> {d['next_waypoint']}" if d['next_waypoint'] else "[DESTINATION REACHED]"
            branches_str = ', '.join(d['branch_choices'])
            print(f"Step {d['step']:02d}: {d['waypoint_id']:<14} ({d['type']:<11}) | Pos: ({d['x']:.2f}m, {d['y']:.2f}m, {d['yaw_deg']}°) | Next: {next_str:<18} | Branch Choices: [{branches_str}]")
    else:
        print("[Error] Path generation failed.")

if __name__ == '__main__':
    main()
