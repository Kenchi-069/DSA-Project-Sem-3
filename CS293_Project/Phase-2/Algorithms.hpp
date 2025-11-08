#ifndef ALGORITHMS_HPP_PHASE2
#define ALGORITHMS_HPP_PHASE2

#include "Graph.hpp"
#include <vector>
#include <unordered_set>
#include <string>
#include <utility>

struct PathResult {
    bool possible;
    double cost;
    std::vector<int> path;
    PathResult() : possible(false), cost(INF) {}
};

// FIXED: Added forbidden_edges for proper Yen's implementation
struct Constraints {
    std::unordered_set<int> forbidden_nodes;
    std::unordered_set<std::string> forbidden_road_types;
    std::unordered_set<std::pair<int,int>, boost::hash<std::pair<int,int>>> forbidden_edges;
    
    bool is_node_forbidden(int node_id) const {
        return forbidden_nodes.count(node_id) > 0;
    }
    bool is_road_type_forbidden(const std::string& road_type) const {
        return forbidden_road_types.count(road_type) > 0;
    }
    bool is_edge_forbidden(int from, int to) const {
        return forbidden_edges.count({from, to}) > 0;
    }
};

struct EdgeHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

class AlgorithmsPhase2 {
public:
    // Phase 2 specific queries
    static std::vector<PathResult> k_shortest_paths(const Graph& graph, int source, int target, int k);
    static std::vector<PathResult> k_shortest_paths_heuristic(const Graph& graph, int source, int target, int k, int overlap_threshold);
    static std::vector<std::pair<int, int>> approximate_shortest_paths(const Graph& graph, const std::vector<std::pair<int, int>>& queries, double time_budget_ms, double acceptable_error_pct);
    
private:
    // Basic Dijkstra with edge constraints
    static PathResult dijkstra(const Graph& graph, int source, int target, const std::unordered_set<std::pair<int,int>, EdgeHash>& forbidden_edges);
    static PathResult dijkstra_simple(const Graph& graph, int source, int target);
    
    // A* for approximate shortest paths
    static PathResult astar(const Graph& graph, int source, int target, double heuristic_weight);
    
    static bool is_simple_path(const std::vector<int>& path);
    static double calculate_edge_overlap_percent(const std::vector<int>& path1, const std::vector<int>& path2);
    
    // Helper for heuristic calculation
    static double euclidean_heuristic(const Graph& graph, int from, int to);
};

#endif
