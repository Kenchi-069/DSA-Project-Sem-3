#include "shortest_path.hpp"
#include <queue>
#include <limits>
#include <algorithm>

ShortestPathSolver::ShortestPathSolver(const Graph& g) : graph(g) {}

PathResult ShortestPathSolver::findShortestPath(
    int source,
    int target,
    const std::string& mode,
    const PathConstraints& constraints,
    int time_slot
) {
    PathResult result;
    result.possible = false;
    result.cost = std::numeric_limits<double>::infinity();
    
    // Check if source or target are forbidden
    if (!isNodeAllowed(source, constraints) || !isNodeAllowed(target, constraints)) {
        return result;
    }
    
    // Dijkstra's algorithm
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    
    using pii = std::pair<double, int>;
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>> pq;
    
    dist[source] = 0.0;
    pq.push({0.0, source});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > dist[u]) continue;
        
        if (u == target) {
            result.possible = true;
            result.cost = d;
            
            // Reconstruct path
            std::vector<int> path;
            int curr = target;
            while (curr != source) {
                path.push_back(curr);
                curr = parent[curr];
            }
            path.push_back(source);
            std::reverse(path.begin(), path.end());
            result.path = path;
            
            return result;
        }
        
        // Explore neighbors
        for (int edge_id : graph.getNeighborEdges(u)) {
            if (!isEdgeAllowed(edge_id, constraints)) continue;
            
            const Edge* edge = graph.getEdge(edge_id);
            if (!edge) continue;
            
            int v = (edge->u == u) ? edge->v : edge->u;
            
            // Skip if one-way edge in wrong direction
            if (edge->oneway && edge->u != u) continue;
            
            if (!isNodeAllowed(v, constraints)) continue;
            
            double weight = graph.getEdgeWeight(edge_id, mode, time_slot);
            double new_dist = d + weight;
            
            if (dist.find(v) == dist.end() || new_dist < dist[v]) {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }
    
    return result;
}

bool ShortestPathSolver::isNodeAllowed(int node_id, const PathConstraints& constraints) const {
    return constraints.forbidden_nodes.find(node_id) == constraints.forbidden_nodes.end();
}

bool ShortestPathSolver::isEdgeAllowed(int edge_id, const PathConstraints& constraints) const {
    const Edge* edge = graph.getEdge(edge_id);
    if (!edge) return false;
    
    return constraints.forbidden_road_types.find(edge->road_type) == 
           constraints.forbidden_road_types.end();
}
