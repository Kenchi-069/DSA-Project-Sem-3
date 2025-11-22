#include "Algorithms.hpp"
#include <queue>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <set>

double DistanceManager::getTime(int from, int to)
{
    if (from == to)
        return 0.0;
    if (time_cache.count(from) && time_cache[from].count(to))
    {
        return time_cache[from][to];
    }

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
    std::unordered_map<int, double> dist;

    dist[from] = 0.0;
    pq.push({0.0, from});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (dist.count(u) && d > dist[u])
            continue;

        time_cache[from][u] = d;

        if (u == to)
            break;

        for (int eid : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(eid);
            if (!e || e->is_deleted)
                continue;

            double new_dist = d + e->average_time;
            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;

            if (!dist.count(v) || new_dist < dist[v])
            {
                dist[v] = new_dist;
                pq.push({new_dist, v});
            }
        }
    }

    if (!time_cache[from].count(to))
        return 1e9;
    return time_cache[from][to];
}

std::vector<int> DistanceManager::getPath(int from, int to)
{
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    dist[from] = 0.0;
    pq.push({0.0, from});

    bool found = false;
    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (u == to)
        {
            found = true;
            break;
        }
        if (dist.count(u) && d > dist[u])
            continue;

        for (int eid : graph.getAdjEdges(u))
        {
            const Edge *e = graph.getEdge(eid);
            if (!e || e->is_deleted)
                continue;
            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u)
                continue;

            double new_dist = d + e->average_time;
            if (!dist.count(v) || new_dist < dist[v])
            {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }

    std::vector<int> path;
    if (!found)
        return path;
    for (int curr = to; curr != from; curr = parent[curr])
        path.push_back(curr);
    path.push_back(from);
    std::reverse(path.begin(), path.end());
    return path;
}

ScheduleResult AlgorithmsPhase3::solveDeliveryScheduling(
    const Graph &graph,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders)
{
    DistanceManager dm(graph);
    std::vector<std::vector<int>> driver_stops(num_drivers);
    std::unordered_set<int> unassigned;

    std::unordered_map<int, std::vector<int>> node_to_orders;
    for (size_t i = 0; i < orders.size(); ++i)
    {
        unassigned.insert(i);
        node_to_orders[orders[i].pickup_node].push_back(i);
        node_to_orders[orders[i].dropoff_node].push_back(i);
    }

    performRegretInsertion(dm, num_drivers, depot, orders, driver_stops, unassigned);

    optimizeRoutes(dm, depot, orders, driver_stops);

    ScheduleResult result;
    result.total_delivery_time = 0;

    std::unordered_map<int, double> order_delivery_times;

    for (int d = 0; d < num_drivers; ++d)
    {
        DriverRoute dr;
        dr.driver_id = d;
        int current_node = depot;
        dr.route_path.push_back(depot);
        double current_time = 0.0;

        std::unordered_set<int> picked_orders;
        std::unordered_set<int> route_stops_set(driver_stops[d].begin(), driver_stops[d].end());

        if (node_to_orders.count(depot))
        {
            for (int o_idx : node_to_orders[depot])
            {
                const auto &o = orders[o_idx];
                if (o.pickup_node == depot)
                {
                    if (route_stops_set.count(o.dropoff_node))
                    {
                        if (current_time < o.ready_time)
                        {
                            current_time = o.ready_time;
                        }
                        picked_orders.insert(o.order_id);
                        if (std::find(dr.order_ids.begin(), dr.order_ids.end(), o.order_id) == dr.order_ids.end())
                        {
                            dr.order_ids.push_back(o.order_id);
                        }
                    }
                }
            }
        }

        for (int stop_node : driver_stops[d])
        {
            std::vector<int> segment = dm.getPath(current_node, stop_node);
            if (segment.size() > 1)
            {
                dr.route_path.insert(dr.route_path.end(), segment.begin() + 1, segment.end());
            }

            double travel = dm.getTime(current_node, stop_node);
            current_time += travel;

            if (node_to_orders.count(stop_node))
            {
                for (int o_idx : node_to_orders[stop_node])
                {
                    const auto &o = orders[o_idx];

                    if (o.pickup_node == stop_node)
                    {
                        if (route_stops_set.count(o.dropoff_node))
                        {
                            if (current_time < o.ready_time)
                            {
                                current_time = o.ready_time;
                            }
                            picked_orders.insert(o.order_id);
                            if (std::find(dr.order_ids.begin(), dr.order_ids.end(), o.order_id) == dr.order_ids.end())
                            {
                                dr.order_ids.push_back(o.order_id);
                            }
                        }
                    }

                    if (o.dropoff_node == stop_node)
                    {
                        if (picked_orders.count(o.order_id))
                        {
                            order_delivery_times[o.order_id] = current_time;
                        }
                    }
                }
            }
            current_node = stop_node;
        }

        dr.completion_time = current_time;
        result.assignments.push_back(dr);
    }

    for (const auto &order : orders)
    {
        if (order_delivery_times.count(order.order_id))
        {
            result.total_delivery_time += order_delivery_times[order.order_id] * order.priority;
        }
    }

    return result;
}

void AlgorithmsPhase3::performRegretInsertion(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::unordered_set<int> &unassigned)
{
    while (!unassigned.empty())
    {
        int best_order_idx = -1;
        double max_regret = -1.0;

        struct Move
        {
            int driver;
            int p_idx;
            int d_idx;
            double cost;
        };
        Move global_best_move = {-1, -1, -1, 1e9};

        for (int order_idx : unassigned)
        {
            const Order &ord = orders[order_idx];
            double best_cost = 1e9, second_best_cost = 1e9;
            Move local_best_move = {-1, -1, -1, 1e9};

            for (int d = 0; d < num_drivers; ++d)
            {
                const auto &stops = driver_stops[d];
                int n = stops.size();

                for (int i = 0; i <= n; ++i)
                {
                    int prev_p = (i == 0) ? depot : stops[i - 1];
                    int next_p = (i == n) ? -1 : stops[i];

                    double cost_p = (next_p == -1)
                                        ? dm.getTime(prev_p, ord.pickup_node)
                                        : calculateInsertionCost(dm, prev_p, next_p, ord.pickup_node);

                    for (int j = i + 1; j <= n + 1; ++j)
                    {
                        int prev_d, next_d;

                        if (j == i + 1)
                        {
                            prev_d = ord.pickup_node;
                            next_d = (i == n) ? -1 : stops[i];
                        }
                        else
                        {
                            prev_d = stops[j - 2];
                            next_d = (j - 1 == n) ? -1 : stops[j - 1];
                        }

                        double cost_d = (next_d == -1)
                                            ? dm.getTime(prev_d, ord.dropoff_node)
                                            : calculateInsertionCost(dm, prev_d, next_d, ord.dropoff_node);

                        double total = cost_p + cost_d;

                        if (total < best_cost)
                        {
                            second_best_cost = best_cost;
                            best_cost = total;
                            local_best_move = {d, i, j, total};
                        }
                        else if (total < second_best_cost)
                        {
                            second_best_cost = total;
                        }
                    }
                }
            }

            double regret = second_best_cost - best_cost;
            if (second_best_cost >= 1e9)
                regret = best_cost * 10.0;
            regret *= ord.priority;

            if (regret > max_regret)
            {
                max_regret = regret;
                best_order_idx = order_idx;
                global_best_move = local_best_move;
            }
        }

        if (best_order_idx != -1)
        {
            const Order &ord = orders[best_order_idx];
            auto &r = driver_stops[global_best_move.driver];

            r.insert(r.begin() + global_best_move.p_idx, ord.pickup_node);
            r.insert(r.begin() + global_best_move.d_idx, ord.dropoff_node);

            unassigned.erase(best_order_idx);
        }
        else
        {
            break;
        }
    }
}

void AlgorithmsPhase3::optimizeRoutes(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops)
{
    std::unordered_map<int, std::vector<int>> node_to_orders;
    for (size_t i = 0; i < orders.size(); ++i)
    {
        node_to_orders[orders[i].pickup_node].push_back(i);
        node_to_orders[orders[i].dropoff_node].push_back(i);
    }

    for (auto &stops : driver_stops)
    {
        if (stops.size() < 3)
            continue;

        bool improved = true;
        int iter = 0;
        while (improved && iter++ < 50)
        {
            improved = false;
            double current_score = evaluateRouteScore(dm, depot, stops, orders);

            for (size_t i = 0; i < stops.size(); ++i)
            {
                for (size_t j = 0; j <= stops.size(); ++j)
                {
                    if (i == j || i + 1 == j)
                        continue;

                    std::vector<int> candidate = stops;
                    int node = candidate[i];
                    candidate.erase(candidate.begin() + i);
                    size_t insert_pos = (i < j) ? j - 1 : j;
                    candidate.insert(candidate.begin() + insert_pos, node);

                    bool valid = true;
                    std::unordered_set<int> picked;
                    for (int n : candidate)
                    {
                        if (node_to_orders.count(n))
                        {
                            for (int oid : node_to_orders[n])
                            {
                                const auto &o = orders[oid];
                                if (o.pickup_node == n)
                                    picked.insert(o.order_id);
                                if (o.dropoff_node == n)
                                {
                                    if (!picked.count(o.order_id))
                                    {
                                        valid = false;
                                        break;
                                    }
                                }
                            }
                        }
                        if (!valid)
                            break;
                    }

                    if (valid)
                    {
                        double new_score = evaluateRouteScore(dm, depot, candidate, orders);
                        if (new_score < current_score - 1e-5)
                        {
                            stops = candidate;
                            current_score = new_score;
                            improved = true;
                            goto next_iter;
                        }
                    }
                }
            }
        next_iter:;
        }
    }
}

double AlgorithmsPhase3::evaluateRouteScore(
    DistanceManager &dm,
    int depot,
    const std::vector<int> &stops,
    const std::vector<Order> &orders)
{
    double total_weighted = 0;
    double time = 0;
    int curr = depot;
    std::unordered_set<int> picked_orders;

    for (int node : stops)
    {
        time += dm.getTime(curr, node);

        for (const auto &o : orders)
        {
            if (o.pickup_node == node)
            {
                if (time < o.ready_time)
                    time = o.ready_time;
                picked_orders.insert(o.order_id);
            }
            if (o.dropoff_node == node && picked_orders.count(o.order_id))
            {
                total_weighted += time * o.priority;
            }
        }
        curr = node;
    }
    return total_weighted;
}

double AlgorithmsPhase3::calculateInsertionCost(DistanceManager &dm, int prev, int next, int insert)
{
    return dm.getTime(prev, insert) + dm.getTime(insert, next) - dm.getTime(prev, next);
}