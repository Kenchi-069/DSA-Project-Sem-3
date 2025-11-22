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

// Helper function to calculate total objective for a given assignment
double calculateTotalObjective(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    const std::vector<std::vector<int>> &driver_stops,
    const std::vector<std::unordered_set<int>> &driver_order_sets)
{
    double total = 0.0;
    
    for (size_t d = 0; d < driver_stops.size(); ++d)
    {
        double time = 0.0;
        int curr = depot;
        std::unordered_set<int> picked;
        
        // Check for depot pickups
        for (const auto &order : orders)
        {
            if (order.pickup_node == depot && driver_order_sets[d].count(order.order_id))
            {
                if (time < order.ready_time)
                    time = order.ready_time;
                picked.insert(order.order_id);
            }
        }
        
        for (int node : driver_stops[d])
        {
            time += dm.getTime(curr, node);
            
            for (const auto &order : orders)
            {
                if (!driver_order_sets[d].count(order.order_id))
                    continue;
                    
                if (order.pickup_node == node)
                {
                    if (time < order.ready_time)
                        time = order.ready_time;
                    picked.insert(order.order_id);
                }
                
                if (order.dropoff_node == node && picked.count(order.order_id))
                {
                    total += time * order.priority;
                }
            }
            curr = node;
        }
    }
    
    return total;
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
    // Track which orders are assigned to which driver
    std::vector<std::unordered_set<int>> driver_orders(num_drivers);
    
    while (!unassigned.empty())
    {
        int best_order_idx = -1;
        int best_driver = -1;
        int best_pickup_pos = -1;
        int best_dropoff_pos = -1;
        double best_objective = 1e9;

        // For each unassigned order, find the best insertion
        for (int order_idx : unassigned)
        {
            const Order &ord = orders[order_idx];

            // Try inserting into each driver
            for (int d = 0; d < num_drivers; ++d)
            {
                auto &stops = driver_stops[d];
                int n = stops.size();
                
                bool pickup_at_depot = (ord.pickup_node == depot);

                // Try all possible insertion positions
                int pickup_start = pickup_at_depot ? 0 : 0;
                int pickup_end = pickup_at_depot ? 1 : (n + 1);
                
                for (int p_pos = pickup_start; p_pos < pickup_end; ++p_pos)
                {
                    for (int d_pos = p_pos + 1; d_pos <= n + 1; ++d_pos)
                    {
                        // Create candidate route
                        std::vector<int> candidate = stops;
                        auto candidate_orders = driver_orders[d];
                        candidate_orders.insert(order_idx);
                        
                        if (!pickup_at_depot)
                        {
                            candidate.insert(candidate.begin() + p_pos, ord.pickup_node);
                            candidate.insert(candidate.begin() + d_pos, ord.dropoff_node);
                        }
                        else
                        {
                            candidate.insert(candidate.begin() + (d_pos - 1), ord.dropoff_node);
                        }
                        
                        // Calculate objective with this insertion
                        double time = 0.0;
                        int curr = depot;
                        std::unordered_set<int> picked;
                        double objective = 0.0;
                        
                        // Handle depot pickups
                        for (int oi : candidate_orders)
                        {
                            const auto &o = orders[oi];
                            if (o.pickup_node == depot)
                            {
                                if (time < o.ready_time)
                                    time = o.ready_time;
                                picked.insert(oi);
                            }
                        }
                        
                        for (int node : candidate)
                        {
                            time += dm.getTime(curr, node);
                            
                            for (int oi : candidate_orders)
                            {
                                const auto &o = orders[oi];
                                
                                if (o.pickup_node == node)
                                {
                                    if (time < o.ready_time)
                                        time = o.ready_time;
                                    picked.insert(oi);
                                }
                                
                                if (o.dropoff_node == node && picked.count(oi))
                                {
                                    objective += time * o.priority;
                                }
                            }
                            curr = node;
                        }
                        
                        if (objective < best_objective)
                        {
                            best_objective = objective;
                            best_order_idx = order_idx;
                            best_driver = d;
                            best_pickup_pos = p_pos;
                            best_dropoff_pos = d_pos;
                        }
                    }
                }
            }
        }

        // Insert the best order
        if (best_order_idx != -1)
        {
            const Order &ord = orders[best_order_idx];
            auto &stops = driver_stops[best_driver];
            
            if (ord.pickup_node != depot)
            {
                stops.insert(stops.begin() + best_pickup_pos, ord.pickup_node);
                stops.insert(stops.begin() + best_dropoff_pos, ord.dropoff_node);
            }
            else
            {
                stops.insert(stops.begin() + (best_dropoff_pos - 1), ord.dropoff_node);
            }
            
            driver_orders[best_driver].insert(best_order_idx);
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