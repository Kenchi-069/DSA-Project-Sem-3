#ifndef SHORTEST_PATH_HPP
#define SHORTEST_PATH_HPP

#include "graph.hpp"
#include <vector>
#include <unordered_set>
#include <string>

struct PathConstraints {
    std::unordered_set<int> forbidden_nodes;
    std::unordered_set<std::string> forbidden_road_types;
};

struct PathResult {
    bool possible;
    double cost;
    std::vector<int> path;
};

class ShortestPathSolver {
private:
    const Graph& graph;
    
public:
    ShortestPathSolver(const Graph& g);
    
    PathResult findShortestPath(
        int source,
        int target,
        const std::string& mode,
        const PathConstraints& constraints,
        int time_slot = -1
    );
    
private:
    bool isNodeAllowed(int node_id, const PathConstraints& constraints) const;
    bool isEdgeAllowed(int edge_id, const PathConstraints& constraints) const;
};

#endif
