#include "Graph.hpp"
#include <algorithm>
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

void Graph::addNode(const Node &node)
{
    nodes[node.id] = node;
    adjacency_list[node.id] = std::vector<int>();

    for (const auto &poi : node.pois)
    {
        if (!poi.empty())
        {
            poi_index[poi].push_back(node.id);
        }
    }
}

void Graph::addEdge(const Edge &edge)
{
    Edge e = edge;
    e.original_length = edge.length;
    e.original_average_time = edge.average_time;
    e.original_speed_profile = edge.speed_profile;
    e.original_road_type = edge.road_type;

    edges[e.id] = e;
    adjacency_list[e.u].push_back(e.id);
    if (!e.oneway)
    {
        adjacency_list[e.v].push_back(e.id);
    }
}

const Node *Graph::getNode(int node_id) const
{
    auto it = nodes.find(node_id);
    return (it != nodes.end()) ? &(it->second) : nullptr;
}

const Edge *Graph::getEdge(int edge_id) const
{
    auto it = edges.find(edge_id);
    return (it != edges.end()) ? &(it->second) : nullptr;
}

bool Graph::hasNode(int node_id) const
{
    return nodes.find(node_id) != nodes.end();
}

bool Graph::hasEdge(int edge_id) const
{
    return edges.find(edge_id) != edges.end();
}

bool Graph::removeEdge(int edge_id)
{
    auto it = edges.find(edge_id);
    if (it != edges.end())
    {
        if (it->second.is_deleted)
            return false;
        it->second.is_deleted = true;
        return true;
    }
    return false;
}

bool Graph::modifyEdge(int edge_id, const Edge &patch, bool has_patch_data)
{
    auto it = edges.find(edge_id);
    if (it == edges.end())
        return false;

    Edge &e = it->second;
    if (e.is_deleted)
    {
        if (!has_patch_data)
        {
            e.length = e.original_length;
            e.average_time = e.original_average_time;
            e.speed_profile = e.original_speed_profile;
            e.road_type = e.original_road_type;
        }
        else
        {
            if (patch.length > 0)
                e.length = patch.length;
            if (patch.average_time > 0)
                e.average_time = patch.average_time;
            if (!patch.speed_profile.empty())
                e.speed_profile = patch.speed_profile;
            if (!patch.road_type.empty())
                e.road_type = patch.road_type;
        }
        e.is_deleted = false;
        return true;
    }
    else
    {
        if (!has_patch_data)
            return false;
        if (patch.length > 0)
            e.length = patch.length;
        if (patch.average_time > 0)
            e.average_time = patch.average_time;
        if (!patch.speed_profile.empty())
            e.speed_profile = patch.speed_profile;
        if (!patch.road_type.empty())
            e.road_type = patch.road_type;
        return true;
    }
}

const std::vector<int> &Graph::getAdjEdges(int node_id) const
{
    static const std::vector<int> empty; // Return type is reference need to declare empty
    auto it = adjacency_list.find(node_id);
    return (it != adjacency_list.end()) ? it->second : empty;
}

std::vector<int> Graph::getNodesPOI(const std::string &poi) const
{
    auto it = poi_index.find(poi);
    return (it != poi_index.end()) ? it->second : std::vector<int>();
}

double Graph::nodeDistance(int node1, int node2) const
{
    const Node *n1 = getNode(node1);
    const Node *n2 = getNode(node2);
    if (!n1 || !n2)
        return INF;
    return EuDistance(n1->lat, n1->lon, n2->lat, n2->lon);
}

double Graph::EuDistance(double lat1, double lon1, double lat2, double lon2, bool exact) const
{
    if (!exact)
    {
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;

        double dx = dlon * 111000.0 * std::cos((lat1 + lat2) / 2.0 * M_PI / 180.0);
        double dy = dlat * 111000.0;

        return std::sqrt(dx * dx + dy * dy);
    }
    else
    {
        const double R = 6371e3;

        double phi1 = lat1 * M_PI / 180.0;
        double phi2 = lat2 * M_PI / 180.0;
        double dphi = (lat2 - lat1) * M_PI / 180.0;
        double dlambda = (lon2 - lon1) * M_PI / 180.0;

        double a = std::sin(dphi / 2.0) * std::sin(dphi / 2.0) +
                   std::cos(phi1) * std::cos(phi2) *
                       std::sin(dlambda / 2.0) * std::sin(dlambda / 2.0);
        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

        return R * c;
    }
}

double Graph::edgeTimeinSlot(int edge_id, int time_slot) const
{
    const Edge *e = getEdge(edge_id);
    if (!e)
        return INF;
    if (e->speed_profile.empty() || time_slot < 0 || time_slot >= 96)
    {
        return e->average_time;
    }
    double speed = e->speed_profile[time_slot];
    if (speed <= 0)
        return e->average_time;
    return e->length / speed;
}

int Graph::findNearestNode(double lat, double lon) const
{
    int nearest = -1;
    double min_dist = INF;
    for (const auto &[id, node] : nodes)
    {
        double dist = EuDistance(lat, lon, node.lat, node.lon);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest = id;
        }
    }
    return nearest;
}
