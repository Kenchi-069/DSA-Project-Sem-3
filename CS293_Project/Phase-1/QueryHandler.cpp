#include "QueryHandler.hpp"
#include <chrono>
#include <iostream>

json QueryHandler::process_queries(const json &queries_json)
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

            if (type == "remove_edge")
                result = handle_remove_edge(query);
            else if (type == "modify_edge")
                result = handle_modify_edge(query);
            else if (type == "shortest_path")
                result = handle_shortest_path(query);
            else if (type == "knn")
                result = handle_knn(query);
            else
            {
                result["error"] = "Unknown query type: " + type;
                if (query.contains("id"))
                    result["id"] = query["id"];
            }

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            result["processing_time"] = duration.count();
        }
        catch (const std::exception &e)
        {
            result["error"] = std::string("Exception: ") + e.what();
            if (query.contains("id"))
                result["id"] = query["id"];
            result["processing_time"] = 0;
        }
        catch (...)
        {
            result["error"] = "Unknown exception";
            if (query.contains("id"))
                result["id"] = query["id"];
            result["processing_time"] = 0;
        }
        output["results"].push_back(result);
    }
    return output;
}

json QueryHandler::handle_remove_edge(const json &query)
{
    json result;
    if (query.contains("id"))
        result["id"] = query["id"];
    int edge_id = query["edge_id"];
    bool success = graph.removeEdge(edge_id);
    result["done"] = success;
    return result;
}

json QueryHandler::handle_modify_edge(const json &query)
{
    json result;
    if (query.contains("id"))
        result["id"] = query["id"];
    int edge_id = query["edge_id"];
    bool has_patch_data = false;
    Edge patch;

    if (query.contains("patch"))
    {
        const json &p = query["patch"];
        has_patch_data = !p.empty();
        if (p.contains("length"))
            patch.length = p["length"];
        if (p.contains("average_time"))
            patch.average_time = p["average_time"];
        if (p.contains("road_type"))
            patch.road_type = p["road_type"];
        if (p.contains("speed_profile"))
        {
            for (const auto &speed : p["speed_profile"])
            {
                patch.speed_profile.push_back(speed);
            }
        }
    }

    bool success = graph.modifyEdge(edge_id, patch, has_patch_data);
    result["done"] = success;
    return result;
}

Constraints QueryHandler::parse_constraints(const json &query)
{
    Constraints constraints;
    if (query.contains("constraints"))
    {
        const json &c = query["constraints"];
        if (c.contains("forbidden_nodes"))
        {
            for (int node : c["forbidden_nodes"])
            {
                constraints.forbidden_nodes.insert(node);
            }
        }
        if (c.contains("forbidden_road_types"))
        {
            for (const std::string &type : c["forbidden_road_types"])
            {
                constraints.forbidden_road_types.insert(type);
            }
        }
    }
    return constraints;
}

json QueryHandler::handle_shortest_path(const json &query)
{
    json result;
    result["id"] = query["id"];
    int source = query["source"];
    int target = query["target"];
    std::string mode = query.value("mode", "distance");
    Constraints constraints = parse_constraints(query);

    PathResult path;
    if (mode == "time")
    {
        path = Algorithms::shortest_path_time(graph, source, target, constraints);
    }
    else
    {
        path = Algorithms::shortest_path_distance(graph, source, target, constraints);
    }

    result["possible"] = path.possible;
    if (path.possible)
    {
        if (mode == "time")
        {
            result["minimum_time"] = path.cost;
        }
        else
        {
            result["minimum_distance"] = path.cost;
        }
        result["path"] = path.path;
    }
    return result;
}

json QueryHandler::handle_knn(const json &query)
{
    json result;
    result["id"] = query["id"];
    std::string poi = query["poi"];
    double lat = query["query_point"]["lat"];
    double lon = query["query_point"]["lon"];
    int k = query["k"];
    std::string metric = query.value("metric", "euclidean");

    std::vector<int> nodes;
    if (metric == "euclidean")
    {
        nodes = Algorithms::knn_euclidean(graph, lat, lon, poi, k);
    }
    else
    {
        nodes = Algorithms::knn_shortest_path(graph, lat, lon, poi, k);
    }
    result["nodes"] = nodes;
    return result;
}
