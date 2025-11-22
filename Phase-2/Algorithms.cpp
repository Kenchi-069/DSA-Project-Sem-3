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

PathResult AlgorithmsPhase2::dijkstraFast(int n, const std::vector<std::vector<FastEdge>> &adj, int source, int target, std::vector<double> &dist, std::vector<int> &parent, const std::vector<bool> &node_blocked, const std::vector<bool> &edge_blocked)
{
    PathResult result;

    if (node_blocked[source] || node_blocked[target])
    {
        result.possible = false;
        return result;
    }

    dist[source] = 0;

    using State = std::pair<double, int>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;
    pq.push({0.0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u])
            continue;
        if (u == target)
            break;

        for (const auto &edge : adj[u])
        {
            if (edge_blocked[edge.id])
                continue;
            if (node_blocked[edge.to])
                continue;

            double new_dist = dist[u] + edge.weight;
            if (new_dist < dist[edge.to])
            {
                dist[edge.to] = new_dist;
                parent[edge.to] = u;
                pq.push({new_dist, edge.to});
            }
        }
    }

    if (dist[target] == std::numeric_limits<double>::max())
    {
        result.possible = false;
        return result;
    }

    result.possible = true;
    result.cost = dist[target];
    int curr = target;
    while (curr != -1)
    {
        result.path.push_back(curr);
        curr = parent[curr];
    }
    std::reverse(result.path.begin(), result.path.end());
    return result;
}

std::vector<PathResult> AlgorithmsPhase2::k_shortest_paths(const Graph &graph, int source, int target, int k)
{
    std::vector<PathResult> results;
    int n = graph.getNodeCount();

    std::vector<std::vector<FastEdge>> adj(n);
    int max_edge_id = 0;

    for (const auto &pair : graph.getEdges())
    {
        const Edge &e = pair.second;
        if (e.is_deleted)
            continue;

        adj[e.u].push_back({e.v, e.length, e.id});
        if (!e.oneway)
        {
            adj[e.v].push_back({e.u, e.length, e.id});
        }
        max_edge_id = std::max(max_edge_id, e.id);
    }

    std::vector<double> dist(n);
    std::vector<int> parent(n);
    std::vector<bool> node_blocked(n, false);
    std::vector<bool> edge_blocked(max_edge_id + 1, false);

    resetDijkstraState(dist, parent);
    PathResult first = dijkstraFast(n, adj, source, target, dist, parent, node_blocked, edge_blocked);

    if (!first.possible)
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
    std::priority_queue<PathCandidate, std::vector<PathCandidate>, ComparePathCandidate> candidates;

    for (int kth = 1; kth < k; kth++)
    {
        const auto &prev_path = results.back().path;

        for (size_t i = 0; i < prev_path.size() - 1; i++)
        {
            int spur_node = prev_path[i];
            std::vector<int> root_path(prev_path.begin(), prev_path.begin() + i + 1);

            double root_path_cost = 0;

            for (size_t j = 0; j < i; j++)
            {
                node_blocked[root_path[j]] = true;
                int u = root_path[j];
                int v = root_path[j + 1];
                for (auto &e : adj[u])
                {
                    if (e.to == v && !edge_blocked[e.id])
                    {
                        root_path_cost += e.weight;
                        break;
                    }
                }
            }

            std::vector<int> edges_to_restore;
            for (const auto &result : results)
            {
                const auto &p_path = result.path;
                if (p_path.size() > i + 1 &&
                    std::equal(root_path.begin(), root_path.end(), p_path.begin(), p_path.begin() + i + 1))
                {
                    int u = p_path[i];
                    int v = p_path[i + 1];

                    for (const auto &e : adj[u])
                    {
                        if (e.to == v)
                        {
                            if (!edge_blocked[e.id])
                            {
                                edge_blocked[e.id] = true;
                                edges_to_restore.push_back(e.id);
                            }
                        }
                    }
                }
            }

            resetDijkstraState(dist, parent);
            PathResult spur_path = dijkstraFast(n, adj, spur_node, target, dist, parent, node_blocked, edge_blocked);

            for (int eid : edges_to_restore)
                edge_blocked[eid] = false;
            for (size_t j = 0; j < i; j++)
                node_blocked[root_path[j]] = false;

            if (spur_path.possible)
            {
                PathResult total_path;
                total_path.path = root_path;
                total_path.path.insert(total_path.path.end(), spur_path.path.begin() + 1, spur_path.path.end());
                total_path.cost = root_path_cost + spur_path.cost;

                candidates.push({total_path.cost, total_path});
            }
        }

        if (candidates.empty())
            break;

        results.push_back(candidates.top().second);
        candidates.pop();
    }

    return results;
}

std::vector<PathResult> AlgorithmsPhase2::k_shortest_paths_heuristic(
    const Graph &graph, int source, int target,
    int k, int overlap_threshold)
{
    std::vector<PathResult> results;

    PathResult P0 = dijkstraSimple(graph, source, target);
    if (!P0.possible)
        return results;

    results.push_back(P0);

    double alpha = std::max(0.12, std::min(0.7, overlap_threshold / 120.0));
    double expo = 1.7;
    double neighbor_frac = 0.55;

    for (int i = 2; i <= k; i++)
    {
        std::unordered_map<std::pair<int, int>, int, EdgeHash> edge_usage;
        for (auto &Pj : results)
        {
            for (int idx = 0; idx + 1 < (int)Pj.path.size(); idx++)
            {
                int u = Pj.path[idx], v = Pj.path[idx + 1];
                edge_usage[{std::min(u, v), std::max(u, v)}] += 1;
            }
        }

        std::unordered_map<std::pair<int, int>, double, EdgeHash> penalty;
        for (auto &kv : edge_usage)
        {
            auto key = kv.first;
            int usage = kv.second;
            double p = alpha * std::pow(expo, (double)(usage - 1));
            penalty[key] = p;
        }

        std::unordered_set<int> path_nodes;
        for (auto &Pj : results)
            for (int n : Pj.path)
                path_nodes.insert(n);

        for (int n : path_nodes)
        {
            for (int eid : graph.getAdjEdges(n))
            {
                const Edge *e = graph.getEdge(eid);
                if (!e || e->is_deleted)
                    continue;

                int a = std::min(e->u, e->v);
                int b = std::max(e->u, e->v);
                penalty[{a, b}] += neighbor_frac * alpha;
            }
        }

        PathResult Pi = dijkstraPenaltyDual(graph, source, target, penalty);
        if (!Pi.possible)
            break;

        results.push_back(Pi);
    }

    std::sort(results.begin(), results.end(),
              [](const PathResult &a, const PathResult &b)
              { return a.cost < b.cost; });

    return results;
}

PathResult AlgorithmsPhase2::dijkstraPenaltyDual(
    const Graph &graph, int source, int target,
    const std::unordered_map<std::pair<int, int>, double, EdgeHash> &penalty)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    using PQ = std::priority_queue<DState, std::vector<DState>, std::greater<DState>>;
    PQ pq;

    std::unordered_map<int, double> best_search_cost;
    std::unordered_map<int, double> best_true_cost;
    std::unordered_map<int, int> parent;

    best_search_cost[source] = 0.0;
    best_true_cost[source] = 0.0;

    pq.push({0.0, 0.0, source});

    while (!pq.empty())
    {
        auto [sc, tc, u] = pq.top();
        pq.pop();

        if (sc > best_search_cost[u])
            continue;

        if (u == target)
        {
            result.possible = true;
            result.cost = tc;

            result.path.clear();
            for (int x = target; x != source; x = parent[x])
                result.path.push_back(x);
            result.path.push_back(source);
            std::reverse(result.path.begin(), result.path.end());

            return result;
        }

        for (int eid : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(eid);
            if (!e || e->is_deleted)
                continue;

            int v = (e->u == u ? e->v : e->u);
            if (e->oneway && e->u != u)
                continue;

            double len = e->length;

            auto key = std::make_pair(std::min(u, v), std::max(u, v));
            double mul = 1.0;
            if (penalty.count(key))
                mul += penalty.at(key);

            double new_true = tc + len;
            double new_search = sc + len * mul;

            if (!best_search_cost.count(v) || new_search < best_search_cost[v])
            {
                best_search_cost[v] = new_search;
                best_true_cost[v] = new_true;
                parent[v] = u;

                pq.push({new_search, new_true, v});
            }
        }
    }

    return result;
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
