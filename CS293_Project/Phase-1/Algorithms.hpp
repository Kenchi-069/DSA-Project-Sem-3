#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include "Graph.hpp"
#include <vector>
#include <unordered_set>
#include <string>

struct PathResult {
    bool possible;
    double cost;
    std::vector<int> path;
    PathResult() : possible(false), cost(INF) {}
};

struct Constraints {
    std::unordered_set<int> forbidden_nodes;
    std::unordered_set<std::string> forbidden_road_types;
    
    bool is_node_forbidden(int node_id) const {
        return forbidden_nodes.count(node_id) > 0;
    }
    bool is_road_type_forbidden(const std::string& road_type) const {
        return forbidden_road_types.count(road_type) > 0;
    }
};

class Algorithms {
public:
    static PathResult shortest_path_distance(const Graph& graph, int source, int target, const Constraints& constraints = Constraints());
    static PathResult shortest_path_time(const Graph& graph, int source, int target, const Constraints& constraints = Constraints());
    
    static std::vector<int> knn_euclidean(const Graph& graph, double query_lat, double query_lon, const std::string& poi, int k);
    static std::vector<int> knn_shortest_path(const Graph& graph, double query_lat, double query_lon, const std::string& poi, int k);
    
private:
    static std::pair<double, int> computeEdgeTravelTime(const Edge &e, double start_time);
    static std::unordered_map<int, double> dijkstra_all_distances(const Graph& graph, int source, bool use_time, const Constraints& constraints = Constraints());
};

#endif
