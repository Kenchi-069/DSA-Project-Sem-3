#ifndef KNN_HPP
#define KNN_HPP

#include "graph.hpp"
#include "shortest_path.hpp"
#include <vector>
#include <string>

struct QueryPoint {
    double lat;
    double lon;
};

class KNNSolver {
private:
    const Graph& graph;
    ShortestPathSolver& sp_solver;
    
public:
    KNNSolver(const Graph& g, ShortestPathSolver& sps);
    
    std::vector<int> findKNN(
        const QueryPoint& query,
        const std::string& poi_type,
        int k,
        const std::string& metric
    );
    
private:
    std::vector<int> getNodesWithPOI(const std::string& poi_type) const;
    double euclideanDistance(const QueryPoint& p, int node_id) const;
};

#endif
