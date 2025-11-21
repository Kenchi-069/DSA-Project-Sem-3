#ifndef QUERY_HANDLER_PHASE3_HPP
#define QUERY_HANDLER_PHASE3_HPP

#include "Graph.hpp"
#include "Algorithms.hpp"
#include "../json.hpp"

using json = nlohmann::json;

class QueryHandlerPhase3
{
    Graph &graph;

public:
    QueryHandlerPhase3(Graph &g) : graph(g) {}
    json solveScheduling(const json &input_json);
};

#endif