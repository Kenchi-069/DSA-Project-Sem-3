#include "QueryHandler.hpp"
#include <chrono>
#include <iostream>

json QueryHandlerPhase2::processQueries(const json &queries_json)
{
    json output;
    output["results"] = json::array();
    if (queries_json.contains("meta"))
    {
        output["meta"] = queries_json["meta"];
    }
    if (!queries_json.contains("events"))
        return output;

    for (const auto &query : queries_json["events"])
    {
        json result;
        try
        {
            std::string type = query.value("type", "");
            auto start = std::chrono::high_resolution_clock::now();

            if (type == "k_shortest_paths")
            {
                result = handleKShortestPaths(query);
            }
            else if (type == "k_shortest_paths_heuristic")
            {
                result = handleKShortestPathsHeuristic(query);
            }
            else if (type == "approx_shortest_path")
            {
                result = handleApproxShortestPath(query);
            }
            else
            {
                result["error"] = "Unknown Phase-2 query type: " + type;
                if (query.contains("id"))
                    result["id"] = query["id"];
            }

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            result["processing_time"] = duration.count();
        }
        catch (...)
        {
            result["error"] = "Error";
            if (query.contains("id"))
                result["id"] = query["id"];
            result["processing_time"] = 0;
        }
        output["results"].push_back(result);
    }
    return output;
}

json QueryHandlerPhase2::handleKShortestPaths(const json &query)
{
    json result;
    result["id"] = query["id"];

    int source = query["source"];
    int target = query["target"];
    int k = query["k"];

    auto paths = AlgorithmsPhase2::k_shortest_paths(graph, source, target, k);
    result["paths"] = json::array();

    for (const auto &path : paths)
    {
        json path_obj;
        path_obj["path"] = path.path;
        path_obj["length"] = path.cost;
        result["paths"].push_back(path_obj);
    }

    return result;
}

json QueryHandlerPhase2::handleKShortestPathsHeuristic(const json &query)
{
    json result;
    result["id"] = query["id"];

    int source = query["source"];
    int target = query["target"];
    int k = query["k"];
    int overlap_threshold = query["overlap_threshold"];

    auto paths = AlgorithmsPhase2::k_shortest_paths_heuristic(graph, source, target, k, overlap_threshold);
    result["paths"] = json::array();

    for (const auto &path : paths)
    {
        json path_obj;
        path_obj["path"] = path.path;
        path_obj["length"] = path.cost;
        result["paths"].push_back(path_obj);
    }

    return result;
}

json QueryHandlerPhase2::handleApproxShortestPath(const json &query)
{
    json result;
    result["id"] = query["id"];

    std::vector<std::pair<int, int>> queries;
    for (const auto &q : query["queries"])
    {
        queries.push_back({q["source"], q["target"]});
    }

    double time_budget = query["time_budget_ms"];
    double acceptable_error = query["acceptable_error_pct"];

    auto processed = AlgorithmsPhase2::approximate_shortest_paths(
        graph, queries, time_budget, acceptable_error);

    result["distances"] = json::array();

    // For each processed query, compute actual distance
    for (const auto &[src, tgt] : processed)
    {
        PathResult path = AlgorithmsPhase2::astar(graph, src, tgt, 1.0);
        json dist_obj;
        dist_obj["source"] = src;
        dist_obj["target"] = tgt;
        dist_obj["approx_shortest_distance"] = path.possible ? path.cost : -1;
        result["distances"].push_back(dist_obj);
    }

    return result;
}
