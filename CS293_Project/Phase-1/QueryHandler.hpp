#ifndef QUERY_HANDLER_HPP
#define QUERY_HANDLER_HPP

#include "Graph.hpp"
#include "Algorithms.hpp"
#include "../json.hpp"

using json = nlohmann::json;

class QueryHandler {
public:
    QueryHandler(Graph& graph) : graph(graph) {}
    json process_queries(const json& queries_json);
    
private:
    Graph& graph;
    json handle_remove_edge(const json& query);
    json handle_modify_edge(const json& query);
    json handle_shortest_path(const json& query);
    json handle_knn(const json& query);
    Constraints parse_constraints(const json& query);
};

#endif
