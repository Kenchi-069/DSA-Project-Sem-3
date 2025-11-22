#include "JsonParser.hpp"
#include <fstream>
#include <iostream>

Graph JsonParserPhase3::parseGraph(const std::string &filename)
{
    std::ifstream file(filename);
    json j;
    file >> j;
    Graph graph;
    if (j.contains("nodes"))
        for (const auto &n : j["nodes"])
            graph.addNode(parseNode(n));
    if (j.contains("edges"))
        for (const auto &e : j["edges"])
            graph.addEdge(parseEdge(e));
    return graph;
}

Node JsonParserPhase3::parseNode(const json &j)
{
    Node n;
    n.id = j["id"];
    n.lat = j["lat"];
    n.lon = j["lon"];
    if (j.contains("pois"))
        for (const auto &p : j["pois"])
            n.pois.push_back(p);
    return n;
}

Edge JsonParserPhase3::parseEdge(const json &j)
{
    Edge e;
    static int x = 0;
    e.id = j.value("id", x++);  
    e.u = j["u"];
    e.v = j["v"];
    e.length = j.value("length", 1);
    e.average_time = j["average_time"];
    e.oneway = j.value("oneway", false);
    e.is_deleted = false;
    return e;
}

json JsonParserPhase3::parseInput(const std::string &filename)
{
    std::ifstream file(filename);
    json j;
    file >> j;
    return j;
}

std::vector<Order> JsonParserPhase3::parseOrders(const json &j)
{
    std::vector<Order> orders;
    if (j.contains("orders"))
    {
        for (const auto &jo : j["orders"])
        {
            Order o;
            o.order_id = jo["order_id"];
            o.pickup_node = jo["pickup"];
            o.dropoff_node = jo["dropoff"];
            o.priority = jo.value("priority", 1.0);
            o.ready_time = jo.value("ready_time", 0.0);

            if (jo.contains("price"))
            {
                double price = jo["price"];
                if (price > 500.0)
                    o.priority += 0.5;
            }
            orders.push_back(o);
        }
    }
    return orders;
}

int JsonParserPhase3::parseFleetSize(const json &j)
{
    if (j.contains("fleet"))
    {
        if (j["fleet"].contains("num_delievery_guys"))
            return j["fleet"]["num_delievery_guys"];

        if (j["fleet"].contains("num_delivery_guys"))
            return j["fleet"]["num_delivery_guys"];
    }
    return 1;
}

int JsonParserPhase3::parseDepot(const json &j)
{
    if (j.contains("fleet") && j["fleet"].contains("depot_node"))
        return j["fleet"]["depot_node"];
    return 0;
}

void JsonParserPhase3::writeOutput(const std::string &filename, const json &output)
{
    std::ofstream file(filename);
    file << output.dump(2);
}