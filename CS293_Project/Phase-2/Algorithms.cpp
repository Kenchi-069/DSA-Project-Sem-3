#include "Algorithms.hpp"
#include <queue>
#include <algorithm>
#include <chrono>
#include <set>
#include <cmath>
#include <iostream>

bool AlgorithmsPhase2::is_simple_path(const std::vector<int> &path)
{
    std::unordered_set<int> visited;
    for (int node : path)
    {
        if (visited.count(node))
            return false;
        visited.insert(node);
    }
    return true;
}

double AlgorithmsPhase2::calculate_edge_overlap_percent(const std::vector<int> &path1, const std::vector<int> &path2)
{
    if (path1.size() < 2 || path2.size() < 2)
        return 0.0;

    std::set<std::pair<int, int>> edges1;
    for (size_t i = 0; i < path1.size() - 1; i++)
    {
        int u = path1[i], v = path1[i + 1];
        edges1.insert({std::min(u, v), std::max(u, v)});
    }

    int common_edges = 0;
    for (size_t i = 0; i < path2.size() - 1; i++)
    {
        int u = path2[i], v = path2[i + 1];
        if (edges1.count({std::min(u, v), std::max(u, v)}))
        {
            common_edges++;
        }
    }

    int total_edges = static_cast<int>(path2.size()) - 1;
    return total_edges > 0 ? (100.0 * common_edges / total_edges) : 0.0;
}

PathResult AlgorithmsPhase2::dijkstra_simple(const Graph &graph, int source, int target)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;

    using PQElement = std::pair<double, int>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (dist.count(u) && d > dist[u])
            continue;
        if (u == target)
        {
            result.possible = true;
            result.cost = dist[u];
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

        for (int edge_id : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(edge_id);
            if (!e || e->is_deleted)
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;

            double edge_cost = e->length;
            double new_dist = dist[u] + edge_cost;
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

// FIXED: Dijkstra with edge-level constraints for Yen's algorithm
PathResult AlgorithmsPhase2::dijkstra(const Graph &graph, int source, int target, const std::unordered_set<std::pair<int, int>, EdgeHash> &forbidden_edges)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;

    using PQElement = std::pair<double, int>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (dist.count(u) && d > dist[u])
            continue;
        if (u == target)
        {
            result.possible = true;
            result.cost = dist[u];
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

        for (int edge_id : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(edge_id);
            if (!e || e->is_deleted)
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;

            // FIXED: Check if this specific edge is forbidden
            if (forbidden_edges.count({u, v}))
                continue;
            if (!e->oneway && forbidden_edges.count({v, u}))
                continue;

            double edge_cost = e->length;
            double new_dist = dist[u] + edge_cost;
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

// FIXED: Yen's K Shortest Paths with proper edge forbidding
std::vector<PathResult> AlgorithmsPhase2::k_shortest_paths(const Graph &graph, int source, int target, int k)
{
    std::vector<PathResult> results;

    PathResult first = dijkstra_simple(graph, source, target);
    if (!first.possible || !is_simple_path(first.path))
        return results;
    results.push_back(first);

    using PathCandidate = std::pair<double, PathResult>;

    struct ComparePathCandidate
    {
        bool operator()(const PathCandidate &a, const PathCandidate &b) const noexcept
        {
            return a.first > b.first; // smaller cost → higher priority
        }
    };

    std::priority_queue<PathCandidate, std::vector<PathCandidate>, ComparePathCandidate> candidates;

    std::set<std::vector<int>> seen_paths;
    seen_paths.insert(first.path);

    for (int k_iter = 1; k_iter < k; k_iter++)
    {
        const PathResult &prev_path = results.back();

        for (size_t i = 0; i < prev_path.path.size() - 1; i++)
        {
            int spur_node = prev_path.path[i];
            std::vector<int> root_path(prev_path.path.begin(), prev_path.path.begin() + i + 1);

            // FIXED: Forbid edges instead of nodes
            std::unordered_set<std::pair<int, int>, EdgeHash> forbidden_edges;

            // Forbid edges from previously found paths with same root
            for (const auto &p : results)
            {
                if (p.path.size() > i + 1)
                {
                    bool same_root = true;
                    for (size_t j = 0; j <= i; j++)
                    {
                        if (j >= p.path.size() || p.path[j] != root_path[j])
                        {
                            same_root = false;
                            break;
                        }
                    }
                    if (same_root)
                    {
                        // Forbid the edge from spur_node to next node in this path
                        forbidden_edges.insert({p.path[i], p.path[i + 1]});
                    }
                }
            }

            // Forbid edges in the root path (to avoid reusing them)
            for (size_t j = 0; j < root_path.size() - 1; j++)
            {
                forbidden_edges.insert({root_path[j], root_path[j + 1]});
            }

            PathResult spur_path = dijkstra(graph, spur_node, target, forbidden_edges);

            if (spur_path.possible)
            {
                PathResult total_path;
                total_path.possible = true;
                total_path.path = root_path;
                total_path.path.insert(total_path.path.end(), spur_path.path.begin() + 1, spur_path.path.end());

                if (!is_simple_path(total_path.path))
                    continue;
                if (seen_paths.count(total_path.path))
                    continue;

                // Calculate total cost
                total_path.cost = 0;
                for (size_t j = 0; j < total_path.path.size() - 1; j++)
                {
                    PathResult segment = dijkstra_simple(graph, total_path.path[j], total_path.path[j + 1]);
                    if (segment.possible)
                        total_path.cost += segment.cost;
                }

                candidates.push({total_path.cost, total_path});
                seen_paths.insert(total_path.path);
            }
        }

        if (candidates.empty())
            break;

        PathResult next = candidates.top().second;
        candidates.pop();
        results.push_back(next);
    }
    return results;
}

std::vector<PathResult> AlgorithmsPhase2::k_shortest_paths_heuristic(const Graph &graph, int source, int target, int k, int overlap_threshold)
{
    std::vector<PathResult> results;

    auto candidates = k_shortest_paths(graph, source, target, std::min(50, k * 10));
    if (candidates.empty())
        return results;

    results.push_back(candidates[0]);

    while (results.size() < k && results.size() < candidates.size())
    {
        double best_penalty = INF;
        int best_idx = -1;

        for (size_t i = 1; i < candidates.size(); i++)
        {
            bool already_selected = false;
            for (const auto &r : results)
            {
                if (r.path == candidates[i].path)
                {
                    already_selected = true;
                    break;
                }
            }
            if (already_selected)
                continue;

            int overlap_penalty = 0;
            for (const auto &selected : results)
            {
                double overlap_pct = calculate_edge_overlap_percent(selected.path, candidates[i].path);
                if (overlap_pct > overlap_threshold)
                {
                    overlap_penalty++;
                }
            }

            double distance_penalty = (candidates[i].cost - candidates[0].cost) / candidates[0].cost / 100.0 + 0.1;
            double total_penalty = overlap_penalty * distance_penalty;

            if (total_penalty < best_penalty)
            {
                best_penalty = total_penalty;
                best_idx = i;
            }
        }

        if (best_idx == -1)
            break;
        results.push_back(candidates[best_idx]);
    }

    return results;
}

double AlgorithmsPhase2::euclidean_heuristic(const Graph &graph, int from, int to)
{
    const Node *n1 = graph.getNode(from);
    const Node *n2 = graph.getNode(to);
    if (!n1 || !n2)
        return 0;
    return graph.nodeDistance(from, to);
}

// A* for faster approximate shortest paths
PathResult AlgorithmsPhase2::astar(const Graph &graph, int source, int target, double heuristic_weight)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> g_score; // Actual cost from source
    std::unordered_map<int, double> f_score; // g_score + heuristic
    std::unordered_map<int, int> parent;

    using PQElement = std::pair<double, int>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

    g_score[source] = 0;
    f_score[source] = heuristic_weight * euclidean_heuristic(graph, source, target);
    pq.push({f_score[source], source});

    while (!pq.empty())
    {
        auto [f, u] = pq.top();
        pq.pop();

        if (f_score.count(u) && f > f_score[u])
            continue;

        if (u == target)
        {
            result.possible = true;
            result.cost = g_score[u];
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

        for (int edge_id : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(edge_id);
            if (!e || e->is_deleted)
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;

            double tentative_g = g_score[u] + e->length;

            if (!g_score.count(v) || tentative_g < g_score[v])
            {
                parent[v] = u;
                g_score[v] = tentative_g;
                f_score[v] = g_score[v] + heuristic_weight * euclidean_heuristic(graph, v, target);
                pq.push({f_score[v], v});
            }
        }
    }
    return result;
}

std::vector<std::pair<int, int>> AlgorithmsPhase2::approximate_shortest_paths(
    const Graph &graph,
    const std::vector<std::pair<int, int>> &queries,
    double time_budget_ms,
    double acceptable_error_pct)
{

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<int, int>> results;

    // Use A* with moderate heuristic weight for speed
    double heuristic_weight = 1.0; // Can tune: higher = faster but less accurate

    for (const auto &[source, target] : queries)
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           current_time - start_time)
                           .count();

        // Safety margin: stop at 85% of budget
        if (elapsed >= time_budget_ms * 0.85)
        {
            break;
        }

        // Use A* for faster computation
        PathResult result = astar(graph, source, target, heuristic_weight);
        if (result.possible)
        {
            results.push_back({source, target});
        }
    }

    return results;
}
