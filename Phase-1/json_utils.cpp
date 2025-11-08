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
