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
    const Graph &graph, int source, int target, int k, int overlap_threshold)
{
    PathResult shortest = dijkstraSimple(graph, source, target);
    if (!shortest.possible)
        return {};

    double shortest_cost = shortest.cost;
    std::vector<PathResult> candidates;
    candidates.push_back(shortest);

    double diversity_need = (100.0 - overlap_threshold) / 100.0;
    double max_detour_factor = 1.0 + (0.6 * diversity_need);
    double max_acceptable_cost = shortest_cost * max_detour_factor;
    double base_penalty = 0.05 + (0.75 * diversity_need);

    int num_nodes = graph.getNodeCount();
    int num_edges = graph.getEdges().size();
    double avg_degree = (2.0 * num_edges) / num_nodes;
    bool is_sparse = avg_degree < 4.0;

    if (is_sparse)
    {
        max_detour_factor = std::min(max_detour_factor, 1.0 + (0.4 * diversity_need));
        max_acceptable_cost = shortest_cost * max_detour_factor;
    }

    std::unordered_map<std::pair<int, int>, double, EdgeHash> edge_penalties;
    std::unordered_map<std::pair<int, int>, int, EdgeHash> edge_usage_count;

    for (size_t i = 0; i + 1 < shortest.path.size(); i++)
    {
        int u = shortest.path[i], v = shortest.path[i + 1];
        edge_usage_count[{std::min(u, v), std::max(u, v)}] = 1;
    }

    std::vector<double> penalty_levels;
    if (diversity_need > 0.6)
    {
        penalty_levels = {0.0, base_penalty * 0.5, base_penalty, base_penalty * 1.5,
                          base_penalty * 2, base_penalty * 3, base_penalty * 4, base_penalty * 6};
    }
    else if (diversity_need > 0.4)
    {
        penalty_levels = {0.0, base_penalty * 0.5, base_penalty, base_penalty * 1.5,
                          base_penalty * 2, base_penalty * 2.5, base_penalty * 3};
    }
    else
    {
        penalty_levels = {0.0, base_penalty * 0.3, base_penalty * 0.6, base_penalty,
                          base_penalty * 1.3, base_penalty * 1.6, base_penalty * 2};
    }

    for (double penalty_mult : penalty_levels)
    {
        if (candidates.size() >= 50)
            break;

        PathResult path = dijkstraWithPenalties(
            graph, source, target, edge_usage_count, penalty_mult, max_acceptable_cost);

        if (path.possible && path.cost <= max_acceptable_cost)
        {
            bool is_diverse = true;
            for (const auto &existing : candidates)
            {
                if (OverlappingEdgePercent(path.path, existing.path) > 90.0)
                {
                    is_diverse = false;
                    break;
                }
            }

            if (is_diverse)
            {
                candidates.push_back(path);
                for (size_t i = 0; i + 1 < path.path.size(); i++)
                {
                    int u = path.path[i], v = path.path[i + 1];
                    edge_usage_count[{std::min(u, v), std::max(u, v)}]++;
                }
            }
        }
    }

    if (!is_sparse && candidates.size() < 40)
    {
        std::vector<int> via_candidates =
            selectViaCandidates(graph, shortest.path, std::min(15, k * 3));

        for (int via : via_candidates)
        {
            if (candidates.size() >= 60)
                break;

            PathResult to_via = dijkstraSimple(graph, source, via);
            PathResult from_via = dijkstraSimple(graph, via, target);

            if (to_via.possible && from_via.possible)
            {
                double total_cost = to_via.cost + from_via.cost;
                if (total_cost <= max_acceptable_cost)
                {
                    PathResult combined;
                    combined.possible = true;
                    combined.cost = total_cost;
                    combined.path = to_via.path;
                    combined.path.insert(combined.path.end(),
                                         from_via.path.begin() + 1, from_via.path.end());

                    bool is_diverse = true;
                    for (const auto &existing : candidates)
                    {
                        if (OverlappingEdgePercent(combined.path, existing.path) > 85.0)
                        {
                            is_diverse = false;
                            break;
                        }
                    }

                    if (is_diverse && isSimplePath(combined.path))
                    {
                        candidates.push_back(combined);
                    }
                }
            }
        }
    }

    if (candidates.size() < 50)
    {
        std::vector<double> cost_limits;
        cost_limits.push_back(shortest_cost * 1.05);
        cost_limits.push_back(shortest_cost * 1.10);
        cost_limits.push_back(shortest_cost * 1.15);
        if (diversity_need > 0.5)
        {
            cost_limits.push_back(shortest_cost * 1.20);
            cost_limits.push_back(shortest_cost * 1.25);
        }
        if (diversity_need > 0.7)
        {
            cost_limits.push_back(shortest_cost * 1.35);
        }

        for (double limit : cost_limits)
        {
            if (limit > max_acceptable_cost)
                break;
            if (candidates.size() >= 60)
                break;

            PathResult path = dijkstraWithCostLimit(graph, source, target, limit,
                                                    edge_usage_count);

            if (path.possible)
            {
                bool is_diverse = true;
                for (const auto &existing : candidates)
                {
                    if (OverlappingEdgePercent(path.path, existing.path) > 85.0)
                    {
                        is_diverse = false;
                        break;
                    }
                }

                if (is_diverse)
                {
                    candidates.push_back(path);
                    for (size_t i = 0; i + 1 < path.path.size(); i++)
                    {
                        int u = path.path[i], v = path.path[i + 1];
                        edge_usage_count[{std::min(u, v), std::max(u, v)}]++;
                    }
                }
            }
        }
    }

    if (candidates.size() < k * 3)
    {
        double perturbation = 0.05 + 0.10 * diversity_need;
        std::mt19937 rng(42);

        for (int iter = 0; iter < std::min(20, k * 4); iter++)
        {
            if (candidates.size() >= 70)
                break;

            PathResult path = dijkstraRandomized(
                graph, source, target, perturbation, rng, max_acceptable_cost);

            if (path.possible)
            {
                bool is_diverse = true;
                for (const auto &existing : candidates)
                {
                    if (OverlappingEdgePercent(path.path, existing.path) > 85.0)
                    {
                        is_diverse = false;
                        break;
                    }
                }

                if (is_diverse)
                {
                    candidates.push_back(path);
                }
            }
        }
    }

    if (candidates.size() < (size_t)k)
    {
        return candidates;
    }

    int n = candidates.size();

    std::vector<std::vector<double>> overlap(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            double ov = OverlappingEdgePercent(candidates[i].path, candidates[j].path);
            overlap[i][j] = ov;
            overlap[j][i] = ov;
        }
    }

    std::vector<double> dist_penalty(n);
    for (int i = 0; i < n; i++)
    {
        dist_penalty[i] = (candidates[i].cost - shortest_cost) / shortest_cost + 0.1;
    }

    auto compute_penalty = [&](const std::vector<int> &selection)
    {
        double total = 0;
        for (int idx_a : selection)
        {
            int overlap_count = 0;
            for (int idx_b : selection)
            {
                if (idx_a != idx_b && overlap[idx_a][idx_b] > overlap_threshold)
                {
                    overlap_count++;
                }
            }
            total += overlap_count * dist_penalty[idx_a];
        }
        return total;
    };

    int beam_size = std::min(50, n / 2);
    beam_size = std::max(beam_size, 20);

    std::vector<std::vector<int>> beam = {{0}};

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

                double penalty = compute_penalty(newset);
                next_beam.push_back({penalty, newset});
            }
        }

        if (next_beam.empty())
            break;

        std::sort(next_beam.begin(), next_beam.end());

        if ((int)next_beam.size() > beam_size)
        {
            next_beam.resize(beam_size);
        }

        beam.clear();
        for (auto &[pen, state] : next_beam)
        {
            beam.push_back(state);
        }
    }

    std::vector<int> best_state = beam[0];
    double best_penalty = compute_penalty(best_state);

    for (auto &state : beam)
    {
        double penalty = compute_penalty(state);
        if (penalty < best_penalty)
        {
            best_penalty = penalty;
            best_state = state;
        }
    }

    bool improved = true;
    int refinement_iters = 0;

    while (improved && refinement_iters++ < 5)
    {
        improved = false;

        for (size_t i = 1; i < best_state.size(); i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (std::find(best_state.begin(), best_state.end(), j) != best_state.end())
                    continue;

                std::vector<int> candidate = best_state;
                candidate[i] = j;
                std::sort(candidate.begin(), candidate.end());

                double new_penalty = compute_penalty(candidate);
                if (new_penalty < best_penalty - 1e-6)
                {
                    best_state = candidate;
                    best_penalty = new_penalty;
                    improved = true;
                    break;
                }
            }
            if (improved)
                break;
        }
    }

    std::vector<PathResult> results;
    for (int idx : best_state)
    {
        results.push_back(candidates[idx]);
    }

    std::sort(results.begin(), results.end(),
              [](const PathResult &a, const PathResult &b)
              {
                  return a.cost < b.cost;
              });

    return results;
}

PathResult AlgorithmsPhase2::dijkstraWithPenalties(
    const Graph &graph, int source, int target,
    const std::unordered_map<std::pair<int, int>, int, EdgeHash> &edge_usage,
    double penalty_multiplier,
    double max_cost)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        std::greater<>>
        pq;

    dist[source] = 0.0;
    pq.push({0.0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > max_cost)
            continue;
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

        for (int eid : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(eid);
            if (!e || e->is_deleted)
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;

            double base_weight = e->length;

            std::pair<int, int> edge_key = {std::min(u, v), std::max(u, v)};
            double penalty = 0.0;
            if (edge_usage.count(edge_key))
            {
                int usage = edge_usage.at(edge_key);
                penalty = base_weight * penalty_multiplier * usage;
            }

            double new_dist = d + base_weight + penalty;

            if (new_dist <= max_cost && (!dist.count(v) || new_dist < dist[v]))
            {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }

    return result;
}

std::vector<int> AlgorithmsPhase2::selectViaCandidates(
    const Graph &graph,
    const std::vector<int> &shortest_path,
    int count)
{
    std::unordered_set<int> on_shortest(shortest_path.begin(), shortest_path.end());
    std::vector<std::pair<double, int>> candidates;

    for (const auto &[node_id, node] : graph.getNodes())
    {
        if (on_shortest.count(node_id))
            continue;

        int degree = graph.getAdjEdges(node_id).size();

        double min_dist_to_path = 1e9;
        for (int sp_node : shortest_path)
        {
            double dist = graph.nodeDistance(node_id, sp_node);
            min_dist_to_path = std::min(min_dist_to_path, dist);
        }

        double score = degree / (1.0 + min_dist_to_path / 1000.0);
        candidates.push_back({score, node_id});
    }

    std::sort(candidates.begin(), candidates.end(), std::greater<>());

    std::vector<int> result;
    for (int i = 0; i < std::min(count, (int)candidates.size()); i++)
    {
        result.push_back(candidates[i].second);
    }

    return result;
}

PathResult AlgorithmsPhase2::dijkstraWithCostLimit(
    const Graph &graph, int source, int target,
    double cost_limit,
    const std::unordered_map<std::pair<int, int>, int, EdgeHash> &edge_usage)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;

    using State = std::tuple<double, int, int>;
    std::priority_queue<State, std::vector<State>, std::greater<>> pq;

    dist[source] = 0.0;
    pq.push({0.0, 0, source});

    std::unordered_map<int, int> unused_edge_count;
    unused_edge_count[source] = 0;

    while (!pq.empty())
    {
        auto [d, neg_unused, u] = pq.top();
        pq.pop();

        if (d > cost_limit)
            continue;
        if (dist.count(u) && d > dist[u] + 1e-6)
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

        for (int eid : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(eid);
            if (!e || e->is_deleted)
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;

            double new_dist = d + e->length;
            if (new_dist > cost_limit)
                continue;

            std::pair<int, int> edge_key = {std::min(u, v), std::max(u, v)};
            int edge_used = edge_usage.count(edge_key) ? 1 : 0;
            int new_unused = unused_edge_count[u] + (1 - edge_used);

            if (!dist.count(v) || new_dist < dist[v] - 1e-6 ||
                (std::abs(new_dist - dist[v]) < 1e-6 && new_unused > unused_edge_count[v]))
            {
                dist[v] = new_dist;
                unused_edge_count[v] = new_unused;
                parent[v] = u;
                pq.push({new_dist, -new_unused, v});
            }
        }
    }

    return result;
}

PathResult AlgorithmsPhase2::dijkstraRandomized(
    const Graph &graph, int source, int target,
    double perturbation, std::mt19937 &rng,
    double max_cost)
{
    PathResult result;
    if (!graph.hasNode(source) || !graph.hasNode(target))
        return result;

    std::uniform_real_distribution<double> dist(1.0 - perturbation, 1.0 + perturbation);

    std::unordered_map<int, double> node_dist;
    std::unordered_map<int, int> parent;
    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        std::greater<>>
        pq;

    node_dist[source] = 0.0;
    pq.push({0.0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > max_cost)
            continue;
        if (node_dist.count(u) && d > node_dist[u])
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

        for (int eid : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(eid);
            if (!e || e->is_deleted)
                continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;

            double perturbed_weight = e->length * dist(rng);
            double new_dist = d + perturbed_weight;

            if (new_dist <= max_cost && (!node_dist.count(v) || new_dist < node_dist[v]))
            {
                node_dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
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
