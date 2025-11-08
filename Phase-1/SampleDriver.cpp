#include "Graph.hpp"
#include "Routing.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>
#include <chrono>
#include <iomanip>

using json = nlohmann::json;

class QueryProcessor {
private:
    Graph graph;
    Routing routing;

public:
    QueryProcessor() : routing(graph) {}

    bool loadGraph(const std::string& graphFile) {
        std::ifstream file(graphFile);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open graph file: " << graphFile << std::endl;
            return false;
        }

        json root;
        try {
            file >> root;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing graph JSON: " << e.what() << std::endl;
            return false;
        }
        file.close();

        // Parse nodes
        if (root.contains("nodes")) {
            for (const auto& nodeJson : root["nodes"]) {
                Node node;
                node.id = nodeJson["id"].get<int>();
                node.lat = nodeJson["lat"].get<double>();
                node.lon = nodeJson["lon"].get<double>();
                
                if (nodeJson.contains("pois")) {
                    for (const auto& poi : nodeJson["pois"]) {
                        node.pois.push_back(poi.get<std::string>());
                    }
                }
                
                graph.addNode(node);
            }
        }

        // Parse edges
        if (root.contains("edges")) {
            for (const auto& edgeJson : root["edges"]) {
                Edge edge;
                edge.id = edgeJson["id"].get<int>();
                edge.u = edgeJson["u"].get<int>();
                edge.v = edgeJson["v"].get<int>();
                edge.length = edgeJson["length"].get<double>();
                edge.averageTime = edgeJson["averagetime"].get<double>();
                edge.roadType = edgeJson["roadtype"].get<std::string>();
                edge.oneWay = edgeJson.contains("oneway") ? edgeJson["oneway"].get<bool>() : false;
                
                if (edgeJson.contains("speedprofile")) {
                    for (const auto& speed : edgeJson["speedprofile"]) {
                        edge.speedProfile.push_back(speed.get<double>());
                    }
                }
                
                graph.addEdge(edge);
            }
        }

        return true;
    }

    bool processQueries(const std::string& queryFile, const std::string& outputFile) {
        std::ifstream qFile(queryFile);
        if (!qFile.is_open()) {
            std::cerr << "Error: Cannot open query file: " << queryFile << std::endl;
            return false;
        }

        json queryRoot;
        try {
            qFile >> queryRoot;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing query JSON: " << e.what() << std::endl;
            return false;
        }
        qFile.close();

        json outputRoot;
        outputRoot["meta"] = queryRoot["meta"];
        json results = json::array();

        if (queryRoot.contains("events")) {
            for (const auto& event : queryRoot["events"]) {
                auto startTime = std::chrono::high_resolution_clock::now();

                try {
                    json result;
                    result["id"] = event["id"];

                    std::string eventType = event["type"].get<std::string>();

                    if (eventType == "removeedge") {
                        int edgeId = event["edgeid"].get<int>();
                        const Edge* edge = graph.getEdge(edgeId);
                        bool wasActive = edge && edge->active;
                        graph.removeEdge(edgeId);
                        result["done"] = wasActive;
                    } 
                    else if (eventType == "modifyedge") {
                        int edgeId = event["edgeid"].get<int>();
                        std::map<std::string, double> patch;
                        
                        if (event.contains("patch") && !event["patch"].is_null()) {
                            const auto& patchJson = event["patch"];
                            if (patchJson.contains("length")) patch["length"] = patchJson["length"].get<double>();
                            if (patchJson.contains("averagetime")) patch["averagetime"] = patchJson["averagetime"].get<double>();
                        }
                        
                        const Edge* existingEdge = graph.getEdge(edgeId);
                        if (existingEdge && !existingEdge->active && !patch.empty()) {
                            graph.restoreEdge(edgeId, patch);
                            result["done"] = true;
                        } else if (existingEdge && existingEdge->active) {
                            graph.modifyEdge(edgeId, patch);
                            result["done"] = true;
                        } else {
                            result["done"] = false;
                        }
                    } 
                    else if (eventType == "shortestpath") {
                        int source = event["source"].get<int>();
                        int target = event["target"].get<int>();
                        std::string mode = event["mode"].get<std::string>();
                        
                        std::set<int> forbiddenNodes;
                        std::set<std::string> forbiddenRoadTypes;
                        
                        if (event.contains("constraints") && event["constraints"].is_object()) {
                            const auto& constraints = event["constraints"];
                            if (constraints.contains("forbiddennodes")) {
                                for (const auto& node : constraints["forbiddennodes"]) {
                                    forbiddenNodes.insert(node.get<int>());
                                }
                            }
                            if (constraints.contains("forbiddenroadtypes")) {
                                for (const auto& roadType : constraints["forbiddenroadtypes"]) {
                                    forbiddenRoadTypes.insert(roadType.get<std::string>());
                                }
                            }
                        }

                        PathResult pathResult;
                        if (mode == "distance") {
                            pathResult = routing.shortestPathDistance(source, target, forbiddenNodes, forbiddenRoadTypes);
                        } else if (mode == "time") {
                            pathResult = routing.shortestPathTime(source, target, 0.0, forbiddenNodes, forbiddenRoadTypes);
                        }

                        result["possible"] = pathResult.possible;
                        if (pathResult.possible) {
                            result["minimumdistance"] = pathResult.minimumDistance;
                            result["minimumtime"] = pathResult.minimumTime;
                            json pathArray = json::array();
                            for (int nodeId : pathResult.path) {
                                pathArray.push_back(nodeId);
                            }
                            result["path"] = pathArray;
                        }
                    } 
                    else if (eventType == "knn") {
                        double lat = event["querypoint"]["lat"].get<double>();
                        double lon = event["querypoint"]["lon"].get<double>();
                        std::string poi = event["poi"].get<std::string>();
                        int k = event["k"].get<int>();
                        std::string metric = event["metric"].get<std::string>();

                        KNNResult knnResult;
                        if (metric == "euclidean") {
                            knnResult = routing.knnEuclidean(lat, lon, poi, k);
                        } else if (metric == "shortestpath") {
                            knnResult = routing.knnShortestPath(lat, lon, poi, k);
                        }

                        json nodesArray = json::array();
                        for (int nodeId : knnResult.nodes) {
                            nodesArray.push_back(nodeId);
                        }
                        result["nodes"] = nodesArray;
                    }

                } catch (const std::exception& e) {
                    std::cerr << "Error processing event " << event["id"] << ": " << e.what() << std::endl;
                }

                auto endTime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
                result["processingtime"] = duration.count();

                results.push_back(result);
            }
        }

        outputRoot["results"] = results;

        std::ofstream oFile(outputFile);
        if (!oFile.is_open()) {
            std::cerr << "Error: Cannot open output file: " << outputFile << std::endl;
            return false;
        }

        oFile << outputRoot.dump(2);
        oFile.close();

        return true;
    }
};

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    QueryProcessor processor;

    if (!processor.loadGraph(argv[1])) {
        return 1;
    }

    if (!processor.processQueries(argv[2], argv[3])) {
        return 1;
    }

    return 0;
}
