#!/bin/bash

# CS293 Phase-1 Project Setup Script
echo "Setting up Phase-1 directory structure..."

# Create directory structure
mkdir -p Phase-1

# Change to Phase-1 directory
cd Phase-1

echo "Creating header files..."

# ============== graph.hpp ==============
cat > graph.hpp << 'EOF'
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
EOF

# ============== shortest_path.hpp ==============
cat > shortest_path.hpp << 'EOF'
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
EOF

# ============== knn.hpp ==============
cat > knn.hpp << 'EOF'
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
EOF

# ============== json_utils.hpp ==============
cat > json_utils.hpp << 'EOF'
#ifndef JSON_UTILS_HPP
#define JSON_UTILS_HPP

#include "graph.hpp"
#include "shortest_path.hpp"
#include "knn.hpp"
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class JSONParser {
public:
    static Graph parseGraph(const std::string& filename);
    static json parseQueries(const std::string& filename);
    static void writeOutput(const std::string& filename, const json& output);
};

class QueryProcessor {
private:
    Graph& graph;
    ShortestPathSolver sp_solver;
    KNNSolver knn_solver;
    
public:
    QueryProcessor(Graph& g);
    
    json processQueries(const json& queries);
    
private:
    json processUpdate(const json& query);
    json processShortestPath(const json& query);
    json processKNN(const json& query);
};

#endif
EOF

echo "Creating implementation files..."

# ============== graph.cpp ==============
cat > graph.cpp << 'EOF'
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
EOF

# ============== shortest_path.cpp ==============
cat > shortest_path.cpp << 'EOF'
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
EOF

# ============== knn.cpp ==============
cat > knn.cpp << 'EOF'
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
EOF

# ============== json_utils.cpp ==============
cat > json_utils.cpp << 'EOF'
#include "json_utils.hpp"
#include <fstream>
#include <iostream>

Graph JSONParser::parseGraph(const std::string& filename) {
    std::ifstream file(filename);
    json j;
    file >> j;
    
    Graph graph;
    
    // Parse nodes
    for (const auto& node_json : j["nodes"]) {
        Node node;
        node.id = node_json["id"];
        node.lat = node_json["lat"];
        node.lon = node_json["lon"];
        
        if (node_json.contains("pois")) {
            for (const auto& poi : node_json["pois"]) {
                node.pois.push_back(poi);
            }
        }
        
        graph.addNode(node);
    }
    
    // Parse edges
    for (const auto& edge_json : j["edges"]) {
        Edge edge;
        edge.id = edge_json["id"];
        edge.u = edge_json["u"];
        edge.v = edge_json["v"];
        edge.length = edge_json["length"];
        edge.average_time = edge_json["average_time"];
        edge.oneway = edge_json.value("oneway", false);
        edge.road_type = edge_json.value("road_type", "");
        
        if (edge_json.contains("speed_profile")) {
            for (const auto& speed : edge_json["speed_profile"]) {
                edge.speed_profile.push_back(speed);
            }
        }
        
        graph.addEdge(edge);
    }
    
    return graph;
}

json JSONParser::parseQueries(const std::string& filename) {
    std::ifstream file(filename);
    json j;
    file >> j;
    return j;
}

void JSONParser::writeOutput(const std::string& filename, const json& output) {
    std::ofstream file(filename);
    file << output.dump(2) << std::endl;
}

QueryProcessor::QueryProcessor(Graph& g) 
    : graph(g), sp_solver(g), knn_solver(g, sp_solver) {}

json QueryProcessor::processQueries(const json& queries) {
    json output;
    output["results"] = json::array();
    
    for (const auto& event : queries["events"]) {
        std::string type = event["type"];
        
        json result;
        if (type == "remove_edge" || type == "modify_edge") {
            result = processUpdate(event);
        } else if (type == "shortest_path") {
            result = processShortestPath(event);
        } else if (type == "knn") {
            result = processKNN(event);
        }
        
        output["results"].push_back(result);
    }
    
    return output;
}

json QueryProcessor::processUpdate(const json& query) {
    json result;
    
    if (query["type"] == "remove_edge") {
        int edge_id = query["edge_id"];
        graph.removeEdge(edge_id);
        result["done"] = true;
    } else if (query["type"] == "modify_edge") {
        int edge_id = query["edge_id"];
        const json& patch = query["patch"];
        
        for (auto it = patch.begin(); it != patch.end(); ++it) {
            graph.modifyEdge(edge_id, it.key(), it.value());
        }
        
        result["done"] = true;
    }
    
    return result;
}

json QueryProcessor::processShortestPath(const json& query) {
    json result;
    result["id"] = query["id"];
    
    int source = query["source"];
    int target = query["target"];
    std::string mode = query["mode"];
    
    PathConstraints constraints;
    if (query.contains("constraints")) {
        if (query["constraints"].contains("forbidden_nodes")) {
            for (int node : query["constraints"]["forbidden_nodes"]) {
                constraints.forbidden_nodes.insert(node);
            }
        }
        if (query["constraints"].contains("forbidden_road_types")) {
            for (const std::string& type : query["constraints"]["forbidden_road_types"]) {
                constraints.forbidden_road_types.insert(type);
            }
        }
    }
    
    PathResult path_result = sp_solver.findShortestPath(source, target, mode, constraints);
    
    result["possible"] = path_result.possible;
    if (path_result.possible) {
        if (mode == "time") {
            result["minimum_time"] = path_result.cost;
        } else {
            result["minimum_distance"] = path_result.cost;
        }
        result["path"] = path_result.path;
    }
    
    return result;
}

json QueryProcessor::processKNN(const json& query) {
    json result;
    result["id"] = query["id"];
    
    QueryPoint qp;
    qp.lat = query["query_point"]["lat"];
    qp.lon = query["query_point"]["lon"];
    
    std::string poi_type = query["type"];
    int k = query["k"];
    std::string metric = query["metric"];
    
    std::vector<int> knn_nodes = knn_solver.findKNN(qp, poi_type, k, metric);
    result["nodes"] = knn_nodes;
    
    return result;
}
EOF

# ============== SampleDriver.cpp ==============
cat > SampleDriver.cpp << 'EOF'
#include "json_utils.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }
    
    std::string graph_file = argv[1];
    std::string query_file = argv[2];
    std::string output_file = argv[3];
    
    try {
        // Parse graph
        std::cout << "Loading graph..." << std::endl;
        Graph graph = JSONParser::parseGraph(graph_file);
        
        // Parse queries
        std::cout << "Loading queries..." << std::endl;
        json queries = JSONParser::parseQueries(query_file);
        
        // Process queries
        std::cout << "Processing queries..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        
        QueryProcessor processor(graph);
        json output = processor.processQueries(queries);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        output["processing_time_ms"] = duration.count();
        
        // Write output
        std::cout << "Writing output..." << std::endl;
        JSONParser::writeOutput(output_file, output);
        
        std::cout << "Done! Processing time: " << duration.count() << "ms" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
EOF

echo "Creating Makefile..."

# ============== Makefile ==============
cat > Makefile << 'EOF'
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I.
LDFLAGS = 

# Source files
SOURCES = graph.cpp shortest_path.cpp knn.cpp json_utils.cpp SampleDriver.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Target executable
TARGET = phase1

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

# Dependencies
graph.o: graph.cpp graph.hpp
shortest_path.o: shortest_path.cpp shortest_path.hpp graph.hpp
knn.o: knn.cpp knn.hpp graph.hpp shortest_path.hpp
json_utils.o: json_utils.cpp json_utils.hpp graph.hpp shortest_path.hpp knn.hpp
SampleDriver.o: SampleDriver.cpp json_utils.hpp
EOF

echo "Creating README..."

# ============== README.md ==============
cat > README.md << 'EOF'
# Phase-1: Graph-Based Routing System

## Overview
This directory contains the implementation for Phase-1 of the CS293 project.

## Building
```bash
make
```

This will create the `phase1` executable.

## Running
```bash
./phase1 <graph.json> <queries.json> <output.json>
```

## Dependencies
- C++17 compiler (g++ or clang++)
- nlohmann/json library (json.hpp header)

## Installation of JSON Library

### Option 1: Download header file
```bash
wget https://github.com/nlohmann/json/releases/download/v3.11.2/json.hpp
```

### Option 2: Using package manager (Ubuntu/Debian)
```bash
sudo apt-get install nlohmann-json3-dev
```

### Option 3: Using Homebrew (macOS)
```bash
brew install nlohmann-json
```

## File Structure
- `graph.hpp/cpp`: Graph data structure and operations
- `shortest_path.hpp/cpp`: Dijkstra's algorithm implementation
- `knn.hpp/cpp`: K-Nearest Neighbor search
- `json_utils.hpp/cpp`: JSON parsing and query processing
- `SampleDriver.cpp`: Main driver program
- `Makefile`: Build configuration

## Features Implemented
1. **Graph Management**
   - Dynamic edge removal
   - Edge modification

2. **Shortest Path**
   - Distance-based routing
   - Time-based routing with traffic profiles
   - Support for constraints (forbidden nodes/roads)

3. **K-Nearest Neighbors**
   - Euclidean distance metric
   - Shortest path distance metric

## Testing
Create test cases in JSON format according to the project specification.

Example:
```bash
./phase1 test_graph.json test_queries.json output.json
```
EOF

cd ..

echo "Creating test data generator script..."

# ============== test_generator.py ==============
cat > test_generator.py << 'EOF'
#!/usr/bin/env python3
import json
import random
import math

def generate_test_graph(num_nodes=20, num_edges=40, filename="test_graph.json"):
    """Generate a random test graph"""
    
    # Generate nodes in a grid around Mumbai coordinates
    base_lat, base_lon = 19.0760, 72.8777
    nodes = []
    
    for i in range(num_nodes):
        node = {
            "id": i,
            "lat": base_lat + random.uniform(-0.05, 0.05),
            "lon": base_lon + random.uniform(-0.05, 0.05),
            "pois": random.sample(["Restaurant", "Hospital", "School", "Park"], k=random.randint(0, 2))
        }
        nodes.append(node)
    
    # Generate edges
    edges = []
    edge_id = 1000
    
    for _ in range(num_edges):
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        
        if u == v:
            continue
        
        # Calculate approximate distance
        lat_diff = (nodes[u]["lat"] - nodes[v]["lat"]) * 111000
        lon_diff = (nodes[u]["lon"] - nodes[v]["lon"]) * 111000
        length = math.sqrt(lat_diff**2 + lon_diff**2)
        
        edge = {
            "id": edge_id,
            "u": u,
            "v": v,
            "length": round(length, 2),
            "average_time": round(length / random.uniform(10, 20), 2),
            "speed_profile": [random.uniform(8, 25) for _ in range(96)],
            "oneway": random.choice([True, False]),
            "road_type": random.choice(["primary", "secondary", "expressway"])
        }
        edges.append(edge)
        edge_id += 1
    
    graph = {
        "meta": {
            "id": "test_case_1",
            "nodes": num_nodes,
            "description": "Auto-generated test graph"
        },
        "nodes": nodes,
        "edges": edges
    }
    
    with open(filename, 'w') as f:
        json.dump(graph, f, indent=2)
    
    print(f"Generated {filename}")

def generate_test_queries(num_nodes=20, filename="test_queries.json"):
    """Generate test queries"""
    
    events = []
    query_id = 1
    
    # Shortest path queries
    for _ in range(3):
        events.append({
            "type": "shortest_path",
            "id": query_id,
            "source": random.randint(0, num_nodes - 1),
            "target": random.randint(0, num_nodes - 1),
            "mode": random.choice(["time", "distance"]),
            "constraints": {
                "forbidden_nodes": random.sample(range(num_nodes), k=random.randint(0, 2)),
                "forbidden_road_types": random.sample(["primary", "secondary"], k=random.randint(0, 1))
            }
        })
        query_id += 1
    
    # KNN queries
    for _ in range(2):
        events.append({
            "type": "knn",
            "id": query_id,
            "type": random.choice(["Restaurant", "Hospital"]),
            "query_point": {
                "lat": 19.0760 + random.uniform(-0.05, 0.05),
                "lon": 72.8777 + random.uniform(-0.05, 0.05)
            },
            "k": random.randint(3, 5),
            "metric": random.choice(["euclidean", "shortest_path"])
        })
        query_id += 1
    
    queries = {
        "meta": {"id": "test_queries"},
        "events": events
    }
    
    with open(filename, 'w') as f:
        json.dump(queries, f, indent=2)
    
    print(f"Generated {filename}")

if __name__ == "__main__":
    generate_test_graph()
    generate_test_queries()
    print("Test data generated successfully!")
EOF

chmod +x test_generator.py

echo ""
echo "=========================================="
echo "Phase-1 setup complete!"
echo "=========================================="
echo ""
echo "Directory structure created:"
echo "  Phase-1/"
echo "    - graph.hpp/cpp"
echo "    - shortest_path.hpp/cpp"
echo "    - knn.hpp/cpp"
echo "    - json_utils.hpp/cpp"
echo "    - SampleDriver.cpp"
echo "    - Makefile"
echo "    - README.md"
echo ""
echo "Test generator created:"
echo "  test_generator.py"
echo ""
echo "Next steps:"
echo "1. Install nlohmann/json library:"
echo "   wget https://github.com/nlohmann/json/releases/download/v3.11.2/json.hpp"
echo "   mv json.hpp Phase-1/"
echo ""
echo "2. Generate test data:"
echo "   python3 test_generator.py"
echo ""
echo "3. Build the project:"
echo "   cd Phase-1 && make"
echo ""
echo "4. Run tests:"
echo "   ./phase1 ../test_graph.json ../test_queries.json output.json"
echo ""