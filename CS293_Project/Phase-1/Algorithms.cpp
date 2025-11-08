#include "Algorithms.hpp"
#include "Graph.hpp"
#include <queue>
#include <algorithm>
#include <cmath>

std::pair<double, int> Algorithms::computeEdgeTravelTime(const Edge &e, double startTime)
{
    // startTime: absolute seconds since t=0
    double remDistance = e.length;
    double totalTime = 0.0;

    double timeSpentInSlot = std::fmod(startTime, 900.0);
    if (timeSpentInSlot < 0.0)
        timeSpentInSlot += 900.0;

    double timeLeftInFirstSlot = (timeSpentInSlot == 0.0) ? 900.0 : (900.0 - timeSpentInSlot);
    bool firstSegment = true;
    double absoluteSegmentStartTime = startTime;

    while (remDistance > 1e-9) // floating point tolerance
    {
        int slotIdx = static_cast<int>(std::floor(absoluteSegmentStartTime / 900.0)) % 96;
        if (slotIdx < 0)
            slotIdx += 96;

        double speed = 0.0;
        if (!e.speed_profile.empty())
            speed = e.speed_profile[slotIdx];

        // fallback to average time
        if (speed <= 0.0)
        {
            speed = e.length / e.average_time;
        }

        double timeAvailable = firstSegment ? timeLeftInFirstSlot : 900.0;
        double distanceCanCover = speed * timeAvailable;

        if (distanceCanCover >= remDistance)
        {
            double tNeeded = remDistance / speed;
            totalTime += tNeeded;
            remDistance = 0.0;
        }
        else
        {
            totalTime += timeAvailable;
            remDistance -= distanceCanCover;
            absoluteSegmentStartTime = startTime + totalTime;
            firstSegment = false;
        }
    }

    double arrivalTime = startTime + totalTime;
    int arrivalSlot = static_cast<int>(std::floor(arrivalTime / 900.0)) % 96;
    if (arrivalSlot < 0)
        arrivalSlot += 96;

    return {totalTime, arrivalSlot};
}

PathResult Algorithms::shortest_path_distance(const Graph &graph, int source, int target, const Constraints &constraints)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    dist[source] = 0.0;
    pq.push({0.0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (dist.count(u) && d > dist[u])
            continue;

        if (u == target)
        {
            result.possible = true;
            result.cost = d;
            for (int cur = target; cur != source; cur = parent[cur])
                result.path.push_back(cur);
            result.path.push_back(source);
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        for (int edge_id : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(edge_id);
            if (!e || e->is_deleted)
                continue;
            if (constraints.is_road_type_forbidden(e->road_type))
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;
            if (constraints.is_node_forbidden(v))
                continue;

            double new_dist = d + e->length;
            if (!dist.count(v) || new_dist < dist[v])
            {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }

    return result;
}

PathResult Algorithms::shortest_path_time(const Graph &graph, int source, int target, const Constraints &constraints)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    struct State
    {
        double time;
        int node;
        bool operator>(const State &o) const { return time > o.time; }
    };

    std::unordered_map<int, double> best_time;
    std::unordered_map<int, int> parent;
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

    best_time[source] = 0.0;
    pq.push({0.0, source});

    while (!pq.empty())
    {
        auto st = pq.top();
        pq.pop();
        double curr_time = st.time;
        int u = st.node;

        if (best_time.count(u) && curr_time > best_time[u])
            continue;

        if (u == target)
        {
            result.possible = true;
            result.cost = curr_time;
            for (int cur = target; cur != source; cur = parent[cur])
                result.path.push_back(cur);
            result.path.push_back(source);
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        for (int edge_id : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(edge_id);
            if (!e || e->is_deleted)
                continue;
            if (constraints.is_road_type_forbidden(e->road_type))
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;
            if (constraints.is_node_forbidden(v))
                continue;

            auto [edge_time, arrival_slot] = computeEdgeTravelTime(*e, curr_time);
            double arrival_time = curr_time + edge_time;

            if (!best_time.count(v) || arrival_time < best_time[v])
            {
                best_time[v] = arrival_time;
                parent[v] = u;
                pq.push({arrival_time, v});
            }
        }
    }

    return result;
}

std::unordered_map<int, double> Algorithms::dijkstraAllDist(const Graph &graph, int source, bool use_time, const Constraints &constraints)
{
    std::unordered_map<int, double> dist;
    if (!graph.hasNode(source))
        return dist;

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    dist[source] = 0.0;
    pq.push({0.0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (dist.count(u) && d > dist[u])
            continue;

        for (int edge_id : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(edge_id);
            if (!e || e->is_deleted)
                continue;
            if (constraints.is_road_type_forbidden(e->road_type))
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;
            if (constraints.is_node_forbidden(v))
                continue;

            double edge_cost = use_time ? e->average_time : e->length;
            double new_dist = d + edge_cost;
            if (!dist.count(v) || new_dist < dist[v])
            {
                dist[v] = new_dist;
                pq.push({new_dist, v});
            }
        }
    }

    return dist;
}

std::vector<int> Algorithms::knn_euclidean(const Graph &graph, double query_lat, double query_lon, const std::string &poi, int k)
{
    std::vector<int> poi_nodes = graph.getNodesPOI(poi);
    std::vector<std::pair<double, int>> distances;

    for (int node_id : poi_nodes)
    {
        const Node *n = graph.getNode(node_id);
        if (!n)
            continue;
        double dist = graph.EuDistance(query_lat, query_lon, n->lat, n->lon);
        distances.push_back({dist, node_id});
    }
    std::sort(distances.begin(), distances.end());

    std::vector<int> result;
    for (int i = 0; i < std::min(k, (int)distances.size()); i++)
    {
        result.push_back(distances[i].second);
    }
    return result;
}

std::vector<int> Algorithms::knn_shortest_path(const Graph &graph, double query_lat, double query_lon, const std::string &poi, int k)
{
    int query_node = graph.findNearestNode(query_lat, query_lon);
    if (query_node == -1)
        return std::vector<int>();

    std::vector<int> poi_nodes = graph.getNodesPOI(poi);
    auto distances = dijkstraAllDist(graph, query_node, false);

    std::vector<std::pair<double, int>> sorted_distances;
    for (int node_id : poi_nodes)
    {
        if (distances.count(node_id) && distances[node_id] < INF)
        {
            sorted_distances.push_back({distances[node_id], node_id});
        }
    }
    std::sort(sorted_distances.begin(), sorted_distances.end());

    std::vector<int> result;
    for (int i = 0; i < std::min(k, (int)sorted_distances.size()); i++)
    {
        result.push_back(sorted_distances[i].second);
    }
    return result;
}
