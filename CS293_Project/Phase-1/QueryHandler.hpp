#ifndef QUERY_HANDLER_HPP
#define QUERY_HANDLER_HPP

#include "Graph.hpp"
#include "Algorithms.hpp"
#include "../json.hpp"

using json = nlohmann::json;

class QueryHandler
{
public:
    QueryHandler(Graph &graph) : graph(graph) {}
    json processQueries(const json &queries_json);

private:
    Graph &graph;
    json handleRemoveEdges(const json &query);
    json handleModifyEdge(const json &query);
    json handleShortestPath(const json &query);
    json handleKNN(const json &query);
    Constraints parseConstraints(const json &query);
};

#endif
