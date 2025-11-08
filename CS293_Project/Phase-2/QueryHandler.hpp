#ifndef QUERY_HANDLER_PHASE2_HPP
#define QUERY_HANDLER_PHASE2_HPP

#include "Graph.hpp"
#include "Algorithms.hpp"
#include "../json.hpp"

using json = nlohmann::json;

class QueryHandlerPhase2 {
public:
    QueryHandlerPhase2(Graph& graph) : graph(graph) {}
    json process_queries(const json& queries_json);
    
private:
    Graph& graph;
    json handle_k_shortest_paths(const json& query);
    json handle_k_shortest_paths_heuristic(const json& query);
    json handle_approx_shortest_path(const json& query);
};

#endif
