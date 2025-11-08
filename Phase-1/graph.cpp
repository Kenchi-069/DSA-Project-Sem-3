#include "graph.hpp"
#include <algorithm>
#include <cmath>

Graph::Graph() {}

void Graph::addNode(const Node& node) {
    nodes[node.id] = node;
    adj_list[node.id] = std::vector<int>();
}

void Graph::addEdge(const Edge& edge) {
    edges[edge.id] = edge;
    adj_list[edge.u].push_back(edge.id);
    if (!edge.oneway) {
        // For bidirectional edges, add reverse direction
        adj_list[edge.v].push_back(edge.id);
    }
}

void Graph::removeEdge(int edge_id) {
    if (edges.find(edge_id) == edges.end()) return;
    
    const Edge& edge = edges[edge_id];
    
    // Remove from adjacency list
    auto& u_edges = adj_list[edge.u];
    u_edges.erase(std::remove(u_edges.begin(), u_edges.end(), edge_id), u_edges.end());
    
    if (!edge.oneway) {
        auto& v_edges = adj_list[edge.v];
        v_edges.erase(std::remove(v_edges.begin(), v_edges.end(), edge_id), v_edges.end());
    }
    
    edges.erase(edge_id);
}

void Graph::modifyEdge(int edge_id, const std::string& field, double value) {
    if (edges.find(edge_id) == edges.end()) return;
    
    if (field == "length") {
        edges[edge_id].length = value;
    } else if (field == "average_time") {
        edges[edge_id].average_time = value;
    }
}

const Node* Graph::getNode(int node_id) const {
    auto it = nodes.find(node_id);
    return (it != nodes.end()) ? &(it->second) : nullptr;
}

const Edge* Graph::getEdge(int edge_id) const {
    auto it = edges.find(edge_id);
    return (it != edges.end()) ? &(it->second) : nullptr;
}

const std::vector<int>& Graph::getNeighborEdges(int node_id) const {
    static std::vector<int> empty;
    auto it = adj_list.find(node_id);
    return (it != adj_list.end()) ? it->second : empty;
}

std::vector<int> Graph::getAllNodeIds() const {
    std::vector<int> node_ids;
    for (const auto& pair : nodes) {
        node_ids.push_back(pair.first);
    }
    return node_ids;
}

double Graph::getEdgeWeight(int edge_id, const std::string& mode, int time_slot) const {
    const Edge* edge = getEdge(edge_id);
    if (!edge) return 1e9;
    
    if (mode == "distance") {
        return edge->length;
    } else if (mode == "time") {
        if (time_slot >= 0 && time_slot < 96 && !edge->speed_profile.empty()) {
            double speed = edge->speed_profile[time_slot];
            return edge->length / speed;
        }
        return edge->average_time;
    }
    return 1e9;
}

double Graph::euclideanDistance(int node1, int node2) const {
    const Node* n1 = getNode(node1);
    const Node* n2 = getNode(node2);
    if (!n1 || !n2) return 1e9;
    
    double lat_diff = n1->lat - n2->lat;
    double lon_diff = n1->lon - n2->lon;
    
    // Approximate conversion to meters (111km per degree latitude)
    double lat_meters = lat_diff * 111000;
    double lon_meters = lon_diff * 111000 * cos(n1->lat * M_PI / 180.0);
    
    return sqrt(lat_meters * lat_meters + lon_meters * lon_meters);
}

bool Graph::hasNode(int node_id) const {
    return nodes.find(node_id) != nodes.end();
}

bool Graph::hasEdge(int edge_id) const {
    return edges.find(edge_id) != edges.end();
}
