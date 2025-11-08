#include "Algorithms.hpp"
#include "Graph.hpp"
#include <queue>
#include <algorithm>

PathResult Algorithms::shortest_path_distance(const Graph &graph, int source, int target, const Constraints &constraints)
{
    return dijkstra(graph, source, target, false, constraints);
}

PathResult Algorithms::shortest_path_time(const Graph &graph, int source, int target, const Constraints &constraints)
{
    return dijkstra(graph, source, target, true, constraints);
}

PathResult Algorithms::dijkstra(const Graph &graph, int source, int target, bool use_time, const Constraints &constraints)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;
    if (constraints.is_node_forbidden(source) || constraints.is_node_forbidden(target))
        return result;

    std::unordered_map<int, double> distance_ya_time;
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> time_slot;

    std::priority_queue<std::pair<double, int>, std::vector< std::pair<double, int>>, std::greater< std::pair<double, int>>> pq;

    distance_ya_time[source] = 0;
    time_slot[source] = 0;
    pq.push({0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top(); // d = distance, u = node
        pq.pop();
        if (distance_ya_time.count(u) && d > distance_ya_time[u])
            continue;
        if (u == target)
        {
            result.possible = true;
            result.cost = distance_ya_time[u];
            std::vector<int> path;
            int curr = target;
            while (parent.count(curr))
            {
                path.push_back(curr);
                curr = parent[curr];
            }
            path.push_back(source);
            std::reverse(path.begin(), path.end());
            result.path = path;
            return result;
        }

        int current_time_slot = time_slot.count(u) ? time_slot[u] : 0;
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

            double edge_cost;
            int next_time_slot = current_time_slot;
            if (use_time)
            {
                edge_cost = graph.edgeTimeinSlot(edge_id, current_time_slot);
                double time_minutes = edge_cost / 60.0;
                int slots_passed = static_cast<int>(time_minutes / 15.0);
                next_time_slot = (current_time_slot + slots_passed) % 96;
            }
            else
            {
                edge_cost = e->length;
            }

            double new_dist = distance_ya_time[u] + edge_cost;
            if (!distance_ya_time.count(v) || new_dist < distance_ya_time[v])
            {
                distance_ya_time[v] = new_dist;
                parent[v] = u;
                time_slot[v] = next_time_slot;
                pq.push({new_dist, v});
            }
        }
    }
    return result;
}

std::unordered_map<int, double> Algorithms::dijkstra_all_distances(const Graph &graph, int source, bool use_time, const Constraints &constraints)
{
    std::unordered_map<int, double> dist;
    if (!graph.hasNode(source))
        return dist;

    std::priority_queue< std::pair<double, int>, std::vector< std::pair<double, int>>, std::greater< std::pair<double, int>>> pq;
    dist[source] = 0;
    pq.push({0, source});

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
            double new_dist = dist[u] + edge_cost;
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
    std::vector< std::pair<double, int>> distances;

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
    auto distances = dijkstra_all_distances(graph, query_node, false);

    std::vector< std::pair<double, int>> sorted_distances;
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
