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
