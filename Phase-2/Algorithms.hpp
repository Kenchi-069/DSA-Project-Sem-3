#ifndef ALGORITHMS_HPP_PHASE2
#define ALGORITHMS_HPP_PHASE2

#include "Graph.hpp"
#include <vector>
#include <unordered_set>
#include <string>
#include <utility>
#include <random>

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

struct VectorHash
{
    size_t operator()(const std::vector<int> &v) const
    {
        size_t seed = v.size();
        for (auto x : v)
        {
            seed ^= x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

struct FastEdge
{
    int to;
    double weight;
    int id;
};

struct DState
{
    double search_cost;
    double true_cost;
    int node;

    bool operator>(const DState &other) const
    {
        return search_cost > other.search_cost;
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
    static PathResult dijkstraFast(int n, const std::vector<std::vector<FastEdge>> &adj, int source, int target, std::vector<double> &dist, std::vector<int> &parent, const std::vector<bool> &node_blocked, const std::vector<bool> &edge_blocked);
    static PathResult dijkstraSimple(const Graph &graph, int source, int target);

    static bool isSimplePath(const std::vector<int> &path);
    static double OverlappingEdgePercent(const std::vector<int> &path1, const std::vector<int> &path2);

    static PathResult dijkstraPenaltyDual(const Graph &graph, int source, int target, const std::unordered_map<std::pair<int, int>, double, EdgeHash> &penalty);
   
    static double euclideanHeuristic(const Graph &graph, int from, int to);

    static void resetDijkstraState(std::vector<double> &dist, std::vector<int> &parent)
    {
        std::fill(dist.begin(), dist.end(), std::numeric_limits<double>::max());
        std::fill(parent.begin(), parent.end(), -1);
    }
};

#endif
