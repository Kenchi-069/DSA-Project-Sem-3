#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

struct Node {
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;
};

struct Edge {
    int id;
    int u;
    int v;
    double length;
    double average_time;
    std::vector<double> speed_profile; // 96 slots for time-dependent speed
    bool oneway;
    std::string road_type;
};

class Graph {
private:
    std::unordered_map<int, Node> nodes;
    std::unordered_map<int, Edge> edges;
    std::unordered_map<int, std::vector<int>> adj_list; // node_id -> list of edge_ids
    
public:
    Graph();
    
    // Add/Remove operations
    void addNode(const Node& node);
    void addEdge(const Edge& edge);
    void removeEdge(int edge_id);
    void modifyEdge(int edge_id, const std::string& field, double value);
    
    // Getters
    const Node* getNode(int node_id) const;
    const Edge* getEdge(int edge_id) const;
    const std::vector<int>& getNeighborEdges(int node_id) const;
    std::vector<int> getAllNodeIds() const;
    
    // Utility
    double getEdgeWeight(int edge_id, const std::string& mode, int time_slot = -1) const;
    double euclideanDistance(int node1, int node2) const;
    bool hasNode(int node_id) const;
    bool hasEdge(int edge_id) const;
};

#endif
