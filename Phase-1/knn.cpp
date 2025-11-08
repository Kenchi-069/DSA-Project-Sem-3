#include "knn.hpp"
#include <algorithm>
#include <limits>

KNNSolver::KNNSolver(const Graph& g, ShortestPathSolver& sps) 
    : graph(g), sp_solver(sps) {}

std::vector<int> KNNSolver::findKNN(
    const QueryPoint& query,
    const std::string& poi_type,
    int k,
    const std::string& metric
) {
    std::vector<int> poi_nodes = getNodesWithPOI(poi_type);
    
    if (poi_nodes.empty()) return {};
    
    std::vector<std::pair<double, int>> distances;
    
    if (metric == "euclidean") {
        for (int node_id : poi_nodes) {
            double dist = euclideanDistance(query, node_id);
            distances.push_back({dist, node_id});
        }
    } else if (metric == "shortest_path") {
        // Find nearest node to query point first
        int nearest_node = -1;
        double min_dist = std::numeric_limits<double>::infinity();
        
        for (int node_id : graph.getAllNodeIds()) {
            double dist = euclideanDistance(query, node_id);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node = node_id;
            }
        }
        
        if (nearest_node == -1) return {};
        
        // Calculate shortest path distances
        PathConstraints empty_constraints;
        for (int node_id : poi_nodes) {
            PathResult result = sp_solver.findShortestPath(
                nearest_node, node_id, "distance", empty_constraints
            );
            double dist = result.possible ? result.cost : std::numeric_limits<double>::infinity();
            distances.push_back({dist, node_id});
        }
    }
    
    // Sort by distance
    std::sort(distances.begin(), distances.end());
    
    // Return top k
    std::vector<int> result;
    int count = std::min(k, static_cast<int>(distances.size()));
    for (int i = 0; i < count; i++) {
        result.push_back(distances[i].second);
    }
    
    return result;
}

std::vector<int> KNNSolver::getNodesWithPOI(const std::string& poi_type) const {
    std::vector<int> result;
    for (int node_id : graph.getAllNodeIds()) {
        const Node* node = graph.getNode(node_id);
        if (node) {
            for (const auto& poi : node->pois) {
                if (poi == poi_type) {
                    result.push_back(node_id);
                    break;
                }
            }
        }
    }
    return result;
}

double KNNSolver::euclideanDistance(const QueryPoint& p, int node_id) const {
    const Node* node = graph.getNode(node_id);
    if (!node) return std::numeric_limits<double>::infinity();
    
    double lat_diff = p.lat - node->lat;
    double lon_diff = p.lon - node->lon;
    
    double lat_meters = lat_diff * 111000;
    double lon_meters = lon_diff * 111000 * cos(p.lat * M_PI / 180.0);
    
    return sqrt(lat_meters * lat_meters + lon_meters * lon_meters);
}
