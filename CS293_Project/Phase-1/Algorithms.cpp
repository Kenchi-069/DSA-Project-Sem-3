#include "Algorithms.hpp"
#include "Graph.hpp"
#include <queue>
#include <algorithm>

PathResult Algorithms::shortest_path_distance(const Graph &graph, int source, int target, const Constraints &constraints)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u])
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

            double new_dist = dist[u] + e->length;
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
        int slot;
        bool operator>(const State &other) const { return time > other.time; }
    };

    std::unordered_map<int, double> best_time;
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> slot_at_node;

    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;
    pq.push({0.0, source, 0});
    best_time[source] = 0.0;
    slot_at_node[source] = 0;

    while (!pq.empty())
    {
        auto [curr_time, u, curr_slot] = pq.top();
        pq.pop();

        if (curr_time > best_time[u])
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

            double edge_time = 0.0;
            double remaining_distance = e->length;
            int slot = curr_slot;

            while (remaining_distance > 0.0)
            {
                double speed = e->speed_profile.empty() ? 0 : e->speed_profile[slot];
                if (speed <= 0)
                    speed = e->average_time > 0 ? e->length / e->average_time : 10.0;

                double time_left_in_slot = 900.0;
                double distance_covered = speed * time_left_in_slot;

                if (distance_covered >= remaining_distance)
                {
                    edge_time += remaining_distance / speed;
                    remaining_distance = 0;
                }
                else
                {
                    edge_time += time_left_in_slot;
                    remaining_distance -= distance_covered;
                    slot = (slot + 1) % 96;
                }
            }

            double new_time = curr_time + edge_time;
            int next_slot = (curr_slot + static_cast<int>(new_time / 900)) % 96;

            if (!best_time.count(v) || new_time < best_time[v])
            {
                best_time[v] = new_time;
                parent[v] = u;
                slot_at_node[v] = next_slot;
                pq.push({new_time, v, next_slot});
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
