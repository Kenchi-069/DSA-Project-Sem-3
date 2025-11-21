#ifndef ALGORITHMS_HPP_PHASE2
#define ALGORITHMS_HPP_PHASE2

#include "Graph.hpp"
#include <vector>
#include <unordered_set>
#include <string>
#include <utility>

struct PathResult
{
    bool possible;
    double cost;
    std::vector<int> path;
    PathResult() : possible(false), cost(INF) {}
};

struct ApproxResult
{
    int source;
    int target;
    double distance;
};

struct EdgeHash
{
    std::size_t operator()(const std::pair<int, int> &p) const
    {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

class AlgorithmsPhase2
{
public:
    static std::vector<PathResult> k_shortest_paths(const Graph &graph, int source, int target, int k);
    static std::vector<PathResult> k_shortest_paths_heuristic(const Graph &graph, int source, int target, int k, int overlap_threshold);
    static std::vector<ApproxResult> approximate_shortest_paths(const Graph &graph, const std::vector<std::pair<int, int>> &queries, double time_budget_ms, double acceptable_error_pct);
    static PathResult astar(const Graph &graph, int source, int target, double heuristic_weight);

private:
    static PathResult dijkstra(const Graph &graph, int source, int target, const std::unordered_set<std::pair<int, int>, EdgeHash> &forbidden_edges, const std::unordered_set<int> &forbidden_nodes);
    static PathResult dijkstraSimple(const Graph &graph, int source, int target);

    static bool isSimplePath(const std::vector<int> &path);
    static double OverlappingEdgePercent(const std::vector<int> &path1, const std::vector<int> &path2);

    static double euclideanHeuristic(const Graph &graph, int from, int to);
};

#endif
