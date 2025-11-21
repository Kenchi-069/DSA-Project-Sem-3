#include "Algorithms.hpp"
#include <queue>
#include <algorithm>
#include <chrono>
#include <set>
#include <cmath>
#include <iostream>

bool AlgorithmsPhase2::isSimplePath(const std::vector<int> &path)
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

double AlgorithmsPhase2::OverlappingEdgePercent(const std::vector<int> &pathBase, const std::vector<int> &pathReference)
{
    if (pathBase.size() < 2 || pathReference.size() < 2)
        return 0.0;

    std::unordered_set<std::pair<int, int>, EdgeHash> refEdges;
    for (size_t i = 0; i < pathReference.size() - 1; i++)
    {
        int u = pathReference[i], v = pathReference[i + 1];
        refEdges.insert({std::min(u, v), std::max(u, v)});
    }

    int common_edges = 0;
    for (size_t i = 0; i < pathBase.size() - 1; i++)
    {
        int u = pathBase[i], v = pathBase[i + 1];
        if (refEdges.count({std::min(u, v), std::max(u, v)}))
        {
            common_edges++;
        }
    }

    int total_edges = static_cast<int>(pathBase.size()) - 1;
    return total_edges > 0 ? (100.0 * common_edges / total_edges) : 0.0;
}

PathResult AlgorithmsPhase2::dijkstraSimple(const Graph &graph, int source, int target)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

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

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
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

// Dijkstra with constraints for Yen's algorithm
PathResult AlgorithmsPhase2::dijkstra(const Graph &graph, int source, int target, const std::unordered_set<std::pair<int, int>, EdgeHash> &forbidden_edges, const std::unordered_set<int> &forbidden_nodes)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;
    if (forbidden_nodes.count(source))
        return result;

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (dist.count(u) && d > dist[u])
            continue;
        if (forbidden_nodes.count(u))
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

            if (forbidden_edges.count({u, v}))
                continue;
            if (!e->oneway && forbidden_edges.count({v, u}))
                continue;
            if (forbidden_nodes.count(v))
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

// Yen's K Shortest Paths
std::vector<PathResult> AlgorithmsPhase2::k_shortest_paths(
    const Graph &graph, int source, int target, int k)
{
    std::vector<PathResult> results;

    PathResult first = dijkstraSimple(graph, source, target);
    if (!first.possible || !isSimplePath(first.path))
        return results;

    results.push_back(first);

    using PathCandidate = std::pair<double, PathResult>;
    struct ComparePathCandidate
    {
        bool operator()(const PathCandidate &a, const PathCandidate &b) const noexcept
        {
            return a.first > b.first;
        }
    };

    std::priority_queue<PathCandidate,
                        std::vector<PathCandidate>,
                        ComparePathCandidate>
        candidates;

    std::set<std::vector<int>> seen_paths;
    seen_paths.insert(first.path);

    for (int kth = 1; kth < k; kth++)
    {
        const auto &prev = results.back();

        for (size_t i = 0; i + 1 < prev.path.size(); i++)
        {
            int spur_node = prev.path[i];
            std::vector<int> root_path(prev.path.begin(), prev.path.begin() + i + 1);

            std::unordered_set<std::pair<int, int>, EdgeHash> forbidden_edges;
            std::unordered_set<int> forbidden_nodes;

            for (const auto &p : results)
            {
                if (p.path.size() > i &&
                    std::equal(root_path.begin(), root_path.end(), p.path.begin()))
                {
                    forbidden_edges.insert({p.path[i], p.path[i + 1]});
                }
            }

            for (size_t j = 0; j + 1 < root_path.size(); j++)
                forbidden_nodes.insert(root_path[j]);

            PathResult spur_path = dijkstra(graph, spur_node, target,
                                            forbidden_edges, forbidden_nodes);

            if (!spur_path.possible)
                continue;

            PathResult total_path;
            total_path.possible = true;
            total_path.path = root_path;
            total_path.path.insert(total_path.path.end(),
                                   spur_path.path.begin() + 1,
                                   spur_path.path.end());

            if (!isSimplePath(total_path.path))
                continue;
            if (seen_paths.count(total_path.path))
                continue;

            double root_cost = 0;

            for (size_t j = 0; j + 1 < root_path.size(); j++)
            {
                int a = root_path[j];
                int b = root_path[j + 1];

                double best = 1e100;

                for (int eid : graph.getAdjEdges(a))
                {
                    const Edge *e = graph.getEdge(eid);
                    if (!e || e->is_deleted)
                        continue;

                    int v = (e->u == a) ? e->v : e->u;
                    if (v != b)
                        continue;

                    if (e->oneway && e->u != a)
                        continue;

                    best = std::min(best, e->length);
                }

                if (best >= 1e99)
                {
                    root_cost = 1e100;
                    break;
                }

                root_cost += best;
            }

            total_path.cost = root_cost + spur_path.cost;

            candidates.push({total_path.cost, total_path});
            seen_paths.insert(total_path.path);
        }

        if (candidates.empty())
            break;

        results.push_back(candidates.top().second);
        candidates.pop();
    }

    return results;
}

std::vector<PathResult> AlgorithmsPhase2::k_shortest_paths_heuristic(
    const Graph &graph, int source, int target, int k, int overlap_threshold)
{
    int CAND_LIMIT = std::min(80, k * 20);
    auto candidates = k_shortest_paths(graph, source, target, CAND_LIMIT);

    if (candidates.empty())
        return {};

    if (candidates.size() < (size_t)k)
        return candidates;

    int n = candidates.size();

    std::vector<std::vector<double>> overlap(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i == j)
                continue;
            overlap[i][j] = OverlappingEdgePercent(candidates[i].path, candidates[j].path);
        }
    }

    double shortest_cost = candidates[0].cost;
    std::vector<double> dist_penalty(n);
    for (int i = 0; i < n; i++)
    {
        dist_penalty[i] = (candidates[i].cost - shortest_cost) / shortest_cost + 0.1;
    }

    const int BEAM_SIZE = 20;

    std::vector<std::vector<int>> beam = {{0}};

    auto compute_penalty = [&](const std::vector<int> &S)
    {
        double tot = 0;
        for (int idx_a : S)
        {
            double overlap_count = 0;
            for (int idx_b : S)
            {
                if (idx_a == idx_b)
                    continue;
                if (overlap[idx_a][idx_b] > overlap_threshold)
                {
                    overlap_count++;
                }
            }
            tot += overlap_count * dist_penalty[idx_a];
        }
        return tot;
    };

    for (int step = 2; step <= k; step++)
    {
        std::vector<std::pair<double, std::vector<int>>> next_beam;

        for (const auto &partial : beam)
        {
            int last_idx = partial.back();

            for (int i = last_idx + 1; i < n; i++)
            {
                std::vector<int> newset = partial;
                newset.push_back(i);

                double p = compute_penalty(newset);
                next_beam.push_back({p, newset});
            }
        }

        if (next_beam.empty())
            break;

        std::sort(next_beam.begin(), next_beam.end(),
                  [](const auto &a, const auto &b)
                  { return a.first < b.first; });

        if ((int)next_beam.size() > BEAM_SIZE)
            next_beam.resize(BEAM_SIZE);

        beam.clear();
        for (auto &x : next_beam)
            beam.push_back(x.second);
    }

    std::vector<int> best_state = beam[0];

    double best_val = compute_penalty(best_state);
    for (auto &s : beam)
    {
        double val = compute_penalty(s);
        if (val < best_val)
        {
            best_val = val;
            best_state = s;
        }
    }

    std::vector<PathResult> results;
    for (int idx : best_state)
        results.push_back(candidates[idx]);

    return results;
}

double AlgorithmsPhase2::euclideanHeuristic(const Graph &graph, int from, int to)
{
    const Node *n1 = graph.getNode(from);
    const Node *n2 = graph.getNode(to);
    if (!n1 || !n2)
        return 0;
    return graph.nodeDistance(from, to);
}

PathResult AlgorithmsPhase2::astar(const Graph &graph, int source, int target, double heuristic_weight)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> g_score;
    std::unordered_map<int, double> f_score;

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    g_score[source] = 0;
    f_score[source] = heuristic_weight * euclideanHeuristic(graph, source, target);
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
                g_score[v] = tentative_g;
                f_score[v] = g_score[v] + heuristic_weight * euclideanHeuristic(graph, v, target);
                pq.push({f_score[v], v});
            }
        }
    }
    return result;
}

std::vector<ApproxResult> AlgorithmsPhase2::approximate_shortest_paths(
    const Graph &graph,
    const std::vector<std::pair<int, int>> &queries,
    double time_budget_ms,
    double acceptable_error_pct)
{

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<ApproxResult> results;

    double heuristic_weight = 1.0 + (acceptable_error_pct / 100.0) * 4.0;
    if (heuristic_weight < 1.25)
        heuristic_weight = 1.25;

    for (const auto &[source, target] : queries)
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(
                           current_time - start_time)
                           .count();

        // Safety margin
        if (elapsed >= time_budget_ms * 0.90)
        {
            break;
        }

        PathResult result = astar(graph, source, target, heuristic_weight);
        if (result.possible)
        {
            results.push_back({source, target, result.cost});
        }
    }

    return results;
}
