#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <memory> // <-- ADDED for smart pointers

const double INF = std::numeric_limits<double>::infinity();

struct Node {
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;
    
    Node() : id(-1), lat(0), lon(0) {}
    Node(int id, double lat, double lon) : id(id), lat(lat), lon(lon) {}
};

struct Edge {
    int id;
    int u;
    int v;
    double length;
    double average_time;
    std::vector<double> speed_profile;
    bool oneway;
    std::string road_type;
    bool is_deleted;
    
    double original_length;
    double original_average_time;
    std::vector<double> original_speed_profile;
    std::string original_road_type;
    
    Edge() : id(-1), u(-1), v(-1), length(0), average_time(0), 
             oneway(false), is_deleted(false),
             original_length(0), original_average_time(0) {}
};

class Graph {
private:
    // --- MODIFIED: Use unique_ptr to store objects on the heap ---
    std::unordered_map<int, std::unique_ptr<Node>> nodes;
    std::unordered_map<int, std::unique_ptr<Edge>> edges;
    // ---
    
    std::unordered_map<int, std::vector<int>> adjacency_list;
    std::unordered_map<std::string, std::vector<int>> poi_index;
    
public:
    Graph() {}
    
    // No explicit destructor needed!
    // std::unique_ptr automatically deletes memory.
    
    void add_node(const Node& node);
    void add_edge(const Edge& edge);
    
    Node* get_node(int node_id);
    const Node* get_node(int node_id) const;
    Edge* get_edge(int edge_id);
    const Edge* get_edge(int edge_id) const;
    
    bool has_node(int node_id) const;
    bool has_edge(int edge_id) const;
    bool remove_edge(int edge_id);
    bool modify_edge(int edge_id, const Edge& patch, bool has_patch_data);
    
    const std::vector<int>& get_adjacent_edges(int node_id) const;
    std::vector<int> get_nodes_with_poi(const std::string& poi) const;
    
    double euclidean_distance(int node1, int node2) const;
    double euclidean_distance(double lat1, double lon1, double lat2, double lon2) const;
    double get_edge_time(int edge_id, int time_slot) const;
    
    // --- MODIFIED: Return types reflect unique_ptr map ---
    const std::unordered_map<int, std::unique_ptr<Node>>& get_nodes() const { return nodes; }
    const std::unordered_map<int, std::unique_ptr<Edge>>& get_edges() const { return edges; }
    // ---
    
    int get_node_count() const { return nodes.size(); }
    int find_nearest_node(double lat, double lon) const;
};

#endif
