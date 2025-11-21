#ifndef QUERY_HANDLER_PHASE2_HPP
#define QUERY_HANDLER_PHASE2_HPP

#include "Graph.hpp"
#include "Algorithms.hpp"
#include "../json.hpp"

using json = nlohmann::json;

class QueryHandlerPhase2
{
public:
    QueryHandlerPhase2(Graph &graph) : graph(graph) {}
    json processQueries(const json &queries_json);

private:
    Graph &graph;
    json handleKShortestPaths(const json &query);
    json handleKShortestPathsHeuristic(const json &query);
    json handleApproxShortestPath(const json &query);
};

#endif
