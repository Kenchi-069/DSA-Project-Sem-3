#include "JsonParser.hpp"
#include <iostream>

Graph JsonParser::parseGraph(const std::string &filename)
{
    std::ifstream file(filename);
    json j;
    file >> j;
    Graph graph;

    if (j.contains("nodes"))
    {
        for (const auto &node_json : j["nodes"])
        {
            Node node = parseNode(node_json);
            graph.addNode(node);
        }
    }
    if (j.contains("edges"))
    {
        for (const auto &edge_json : j["edges"])
        {
            Edge edge = parseEdge(edge_json);
            graph.addEdge(edge);
        }
    }
    return graph;
}

Node JsonParser::parseNode(const json &node_json)
{
    Node node;
    node.id = node_json["id"];
    node.lat = node_json["lat"];
    node.lon = node_json["lon"];
    if (node_json.contains("pois"))
    {
        for (const auto &poi : node_json["pois"])
        {
            node.pois.push_back(poi);
        }
    }
    return node;
}

Edge JsonParser::parseEdge(const json &edge_json)
{
    Edge edge;
    edge.id = edge_json["id"];
    edge.u = edge_json["u"];
    edge.v = edge_json["v"];
    edge.length = edge_json["length"];
    edge.average_time = edge_json["average_time"];
    edge.oneway = edge_json.value("oneway", false);
    edge.road_type = edge_json.value("road_type", "");
    edge.is_deleted = false;
    if (edge_json.contains("speed_profile"))
    {
        for (const auto &speed : edge_json["speed_profile"])
        {
            edge.speed_profile.push_back(speed);
        }
    }
    return edge;
}

json JsonParser::parseQueries(const std::string &filename)
{
    std::ifstream file(filename);
    json j;
    file >> j;
    return j;
}

void JsonParser::writeOutput(const std::string &filename, const json &output)
{
    std::ofstream file(filename);
    file << output.dump(2);
    file.close();
}
