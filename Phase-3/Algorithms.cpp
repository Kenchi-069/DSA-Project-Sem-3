#include "Algorithms.hpp"
#include <queue>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <set>
#include <limits>
#include <chrono>
#include <random>

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

double calculateDriverObjective(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    const std::vector<int> &stops,
    const std::unordered_set<int> &order_indices)
{
    double objective = 0.0;
    double time = 0.0;
    int curr = depot;
    std::unordered_set<int> picked;
    
    for (int oi : order_indices)
    {
        const auto &o = orders[oi];
        if (o.pickup_node == depot)
        {
            if (time < o.ready_time)
                time = o.ready_time;
            picked.insert(oi);
        }
    }
    
    for (int node : stops)
    {
        time += dm.getTime(curr, node);
        
        for (int oi : order_indices)
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
    
    return objective;
}

double calculateTotalObjective(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    const std::vector<std::vector<int>> &driver_stops,
    const std::vector<std::unordered_set<int>> &driver_orders)
{
    double total = 0.0;
    for (size_t d = 0; d < driver_stops.size(); ++d)
    {
        total += calculateDriverObjective(dm, depot, orders, driver_stops[d], driver_orders[d]);
    }
    return total;
}

ScheduleResult AlgorithmsPhase3::solveDeliveryScheduling(
    const Graph &graph,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    const double TIME_LIMIT = 25.0;
    
    DistanceManager dm(graph);
    std::vector<std::vector<int>> driver_stops(num_drivers);
    std::vector<std::unordered_set<int>> driver_orders(num_drivers);
    std::unordered_set<int> unassigned;

    std::unordered_map<int, std::vector<int>> node_to_orders;
    for (size_t i = 0; i < orders.size(); ++i)
    {
        unassigned.insert(i);
        node_to_orders[orders[i].pickup_node].push_back(i);
        node_to_orders[orders[i].dropoff_node].push_back(i);
    }

    // Phase 1: Initial construction (spend minimal time)
    performSmartInsertion(dm, num_drivers, depot, orders, driver_stops, driver_orders, unassigned, start_time, 2.0);

    auto best_driver_stops = driver_stops;
    auto best_driver_orders = driver_orders;
    double best_objective = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
    
    std::cout << "Initial objective: " << best_objective << std::endl;

    // Phase 2: Continuous multi-level optimization
    int iteration = 0;
    int stagnation_count = 0;
    double last_improvement_time = 0.0;
    
    while (true)
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        
        if (elapsed > TIME_LIMIT)
            break;
        
        iteration++;
        bool improved_this_round = false;
        
        // Multi-pass optimization: don't give up after first improvement
        int num_passes = (stagnation_count < 3) ? 3 : 5; // More aggressive when stuck
        
        for (int pass = 0; pass < num_passes && elapsed < TIME_LIMIT * 0.95; ++pass)
        {
            current_time = std::chrono::high_resolution_clock::now();
            elapsed = std::chrono::duration<double>(current_time - start_time).count();
            
            if (elapsed > TIME_LIMIT * 0.95)
                break;
            
            double before_pass = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
            
            // Intra-route: relocate with best improvement (not first)
            optimizeRoutesIntensiveBest(dm, depot, orders, driver_stops, driver_orders, start_time, TIME_LIMIT);
            
            // Intra-route: 2-opt
            twoOptIntraRoute(dm, depot, orders, driver_stops, driver_orders, start_time, TIME_LIMIT);
            
            // Inter-route: relocate (find BEST move, not first)
            interRouteRelocateBest(dm, num_drivers, depot, orders, driver_stops, driver_orders, start_time, TIME_LIMIT);
            
            // Inter-route: swap (find BEST swap)
            interRouteSwapBest(dm, num_drivers, depot, orders, driver_stops, driver_orders, start_time, TIME_LIMIT);
            
            double after_pass = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
            
            if (after_pass < before_pass - 1e-9)
            {
                improved_this_round = true;
            }
        }
        
        double current_objective = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
        
        if (current_objective < best_objective - 1e-9)
        {
            best_objective = current_objective;
            best_driver_stops = driver_stops;
            best_driver_orders = driver_orders;
            stagnation_count = 0;
            last_improvement_time = elapsed;
            std::cout << "Iteration " << iteration << ": New best = " << best_objective 
                      << " (time: " << elapsed << "s)" << std::endl;
        }
        else
        {
            stagnation_count++;
            // Always revert to best solution when no improvement
            driver_stops = best_driver_stops;
            driver_orders = best_driver_orders;
        }
        
        // Double-check we're in sync
        double check_obj = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
        if (std::abs(check_obj - best_objective) > 1e-3)
        {
            std::cout << "WARNING: Sync issue detected! check=" << check_obj << " best=" << best_objective << std::endl;
            driver_stops = best_driver_stops;
            driver_orders = best_driver_orders;
        }
        
        // Adaptive perturbation when stuck
        if (stagnation_count >= 4 && elapsed < TIME_LIMIT * 0.85)
        {
            std::cout << "Strong perturbation at iteration " << iteration << " (stagnation: " << stagnation_count << ")" << std::endl;
            
            // Save the current best before perturbation
            auto saved_stops = best_driver_stops;
            auto saved_orders = best_driver_orders;
            
            // Start perturbation from best solution
            driver_stops = best_driver_stops;
            driver_orders = best_driver_orders;
            
            int strength = std::min(stagnation_count - 3, 5);
            perturbSolutionAdaptive(dm, num_drivers, depot, orders, driver_stops, driver_orders, strength);
            
            // Check time before optimization
            current_time = std::chrono::high_resolution_clock::now();
            elapsed = std::chrono::duration<double>(current_time - start_time).count();
            
            if (elapsed < TIME_LIMIT * 0.85)
            {
                // Optimize after perturbation
                for (int i = 0; i < 3; ++i)
                {
                    current_time = std::chrono::high_resolution_clock::now();
                    elapsed = std::chrono::duration<double>(current_time - start_time).count();
                    
                    if (elapsed > TIME_LIMIT * 0.85)
                        break;
                    
                    optimizeRoutesIntensiveBest(dm, depot, orders, driver_stops, driver_orders, start_time, TIME_LIMIT);
                    interRouteRelocateBest(dm, num_drivers, depot, orders, driver_stops, driver_orders, start_time, TIME_LIMIT);
                }
            }
            
            double perturbed_obj = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
            
            if (perturbed_obj < best_objective - 1e-9)
            {
                best_objective = perturbed_obj;
                best_driver_stops = driver_stops;
                best_driver_orders = driver_orders;
                stagnation_count = 0;
                std::cout << "Perturbation SUCCESS! New best = " << best_objective << std::endl;
            }
            else
            {
                // Revert to saved best
                driver_stops = saved_stops;
                driver_orders = saved_orders;
                std::cout << "Perturbation failed, reverting (obj was " << perturbed_obj << ")" << std::endl;
                stagnation_count++; // Increase to try stronger next time
            }
        }
        
        // Safety check: always work with best solution
        if (elapsed > TIME_LIMIT * 0.9)
        {
            driver_stops = best_driver_stops;
            driver_orders = best_driver_orders;
            break; // Exit main loop
        }
    }
    
    driver_stops = best_driver_stops;
    driver_orders = best_driver_orders;
    
    // Final optimization pass on best solution
    auto current_time = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(current_time - start_time).count();
    if (elapsed < TIME_LIMIT * 0.98)
    {
        optimizeRoutesIntensiveBest(dm, depot, orders, driver_stops, driver_orders, start_time, TIME_LIMIT);
        twoOptIntraRoute(dm, depot, orders, driver_stops, driver_orders, start_time, TIME_LIMIT);
        
        double final_check = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
        if (final_check < best_objective)
        {
            best_objective = final_check;
            best_driver_stops = driver_stops;
            best_driver_orders = driver_orders;
            std::cout << "Final polish improved to: " << best_objective << std::endl;
        }
        else
        {
            driver_stops = best_driver_stops;
            driver_orders = best_driver_orders;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double>(end_time - start_time).count();
    std::cout << "Final objective: " << best_objective << " (total time: " << total_time << "s)" << std::endl;
    
    // Verify we're using the best solution
    double verify_obj = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
    std::cout << "Verification objective: " << verify_obj << std::endl;
    if (std::abs(verify_obj - best_objective) > 1e-3)
    {
        std::cout << "WARNING: Objective mismatch! Using stored best solution." << std::endl;
        driver_stops = best_driver_stops;
        driver_orders = best_driver_orders;
    }

    // Build result
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
                if (o.pickup_node == depot && route_stops_set.count(o.dropoff_node))
                {
                    if (current_time < o.ready_time)
                        current_time = o.ready_time;
                    picked_orders.insert(o.order_id);
                    if (std::find(dr.order_ids.begin(), dr.order_ids.end(), o.order_id) == dr.order_ids.end())
                        dr.order_ids.push_back(o.order_id);
                }
            }
        }

        for (int stop_node : driver_stops[d])
        {
            std::vector<int> segment = dm.getPath(current_node, stop_node);
            if (segment.size() > 1)
                dr.route_path.insert(dr.route_path.end(), segment.begin() + 1, segment.end());

            double travel = dm.getTime(current_node, stop_node);
            current_time += travel;

            if (node_to_orders.count(stop_node))
            {
                for (int o_idx : node_to_orders[stop_node])
                {
                    const auto &o = orders[o_idx];

                    if (o.pickup_node == stop_node && route_stops_set.count(o.dropoff_node))
                    {
                        if (current_time < o.ready_time)
                            current_time = o.ready_time;
                        picked_orders.insert(o.order_id);
                        if (std::find(dr.order_ids.begin(), dr.order_ids.end(), o.order_id) == dr.order_ids.end())
                            dr.order_ids.push_back(o.order_id);
                    }

                    if (o.dropoff_node == stop_node && picked_orders.count(o.order_id))
                    {
                        order_delivery_times[o.order_id] = current_time;
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
            result.total_delivery_time += order_delivery_times[order.order_id] * order.priority;
    }

    return result;
}

void AlgorithmsPhase3::performSmartInsertion(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::unordered_set<int> &unassigned,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    while (!unassigned.empty())
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed > time_limit)
            break;
        
        int best_order_idx = -1;
        int best_driver = -1;
        int best_pickup_pos = -1;
        int best_dropoff_pos = -1;
        double best_marginal_cost = std::numeric_limits<double>::max();

        for (int order_idx : unassigned)
        {
            const Order &ord = orders[order_idx];

            for (int d = 0; d < num_drivers; ++d)
            {
                auto &stops = driver_stops[d];
                int n = stops.size();
                
                bool pickup_at_depot = (ord.pickup_node == depot);
                double current_obj = calculateDriverObjective(dm, depot, orders, stops, driver_orders[d]);

                int pickup_start = pickup_at_depot ? 0 : 0;
                int pickup_end = pickup_at_depot ? 1 : (n + 1);
                
                for (int p_pos = pickup_start; p_pos < pickup_end; ++p_pos)
                {
                    for (int d_pos = p_pos + 1; d_pos <= n + 1; ++d_pos)
                    {
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
                        
                        double new_obj = calculateDriverObjective(dm, depot, orders, candidate, candidate_orders);
                        double marginal_cost = new_obj - current_obj;
                        
                        if (marginal_cost < best_marginal_cost)
                        {
                            best_marginal_cost = marginal_cost;
                            best_order_idx = order_idx;
                            best_driver = d;
                            best_pickup_pos = p_pos;
                            best_dropoff_pos = d_pos;
                        }
                    }
                }
            }
        }

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

void AlgorithmsPhase3::optimizeRoutesIntensiveBest(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    for (size_t d = 0; d < driver_stops.size(); ++d)
    {
        auto &stops = driver_stops[d];
        if (stops.size() < 2)
            continue;

        bool improved = true;
        while (improved)
        {
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            if (elapsed > time_limit)
                return;
            
            improved = false;
            double current_score = calculateDriverObjective(dm, depot, orders, stops, driver_orders[d]);

            // Find BEST relocation, not first
            double best_score = current_score;
            std::vector<int> best_candidate;
            
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

                    if (!isRouteValid(candidate, orders, driver_orders[d], depot))
                        continue;

                    double new_score = calculateDriverObjective(dm, depot, orders, candidate, driver_orders[d]);
                    if (new_score < best_score - 1e-9)
                    {
                        best_score = new_score;
                        best_candidate = candidate;
                        improved = true;
                    }
                }
            }
            
            if (improved)
            {
                stops = best_candidate;
            }
        }
    }
}

void AlgorithmsPhase3::twoOptIntraRoute(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    for (size_t d = 0; d < driver_stops.size(); ++d)
    {
        auto &stops = driver_stops[d];
        if (stops.size() < 2)
            continue;

        bool improved = true;
        while (improved)
        {
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            if (elapsed > time_limit)
                return;
            
            improved = false;
            double current_score = calculateDriverObjective(dm, depot, orders, stops, driver_orders[d]);

            double best_score = current_score;
            std::vector<int> best_candidate;

            for (size_t i = 0; i < stops.size() - 1; ++i)
            {
                for (size_t j = i + 1; j < stops.size(); ++j)
                {
                    std::vector<int> candidate = stops;
                    std::reverse(candidate.begin() + i, candidate.begin() + j + 1);
                    
                    if (!isRouteValid(candidate, orders, driver_orders[d], depot))
                        continue;

                    double new_score = calculateDriverObjective(dm, depot, orders, candidate, driver_orders[d]);
                    if (new_score < best_score - 1e-9)
                    {
                        best_score = new_score;
                        best_candidate = candidate;
                        improved = true;
                    }
                }
            }
            
            if (improved)
            {
                stops = best_candidate;
            }
        }
    }
}

void AlgorithmsPhase3::interRouteRelocateBest(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    if (num_drivers < 2)
        return;
    
    bool improved = true;
    while (improved)
    {
        improved = false;
        
        double best_total = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
        int best_oi = -1, best_d1 = -1, best_d2 = -1;
        std::vector<int> best_stops1, best_stops2;
        std::unordered_set<int> best_orders1, best_orders2;
        
        for (int d1 = 0; d1 < num_drivers; ++d1)
        {
            if (driver_orders[d1].empty())
                continue;
            
            std::vector<int> orders_list(driver_orders[d1].begin(), driver_orders[d1].end());
            
            for (int oi : orders_list)
            {
                auto current_time = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(current_time - start_time).count();
                if (elapsed > time_limit)
                    return;
                
                const auto &ord = orders[oi];
                double current_total = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
                
                for (int d2 = 0; d2 < num_drivers; ++d2)
                {
                    if (d1 == d2)
                        continue;
                    
                    auto test_stops1 = driver_stops[d1];
                    auto test_orders1 = driver_orders[d1];
                    test_orders1.erase(oi);
                    
                    test_stops1.erase(std::remove(test_stops1.begin(), test_stops1.end(), ord.pickup_node), test_stops1.end());
                    test_stops1.erase(std::remove(test_stops1.begin(), test_stops1.end(), ord.dropoff_node), test_stops1.end());
                    
                    auto &stops2 = driver_stops[d2];
                    int n = stops2.size();
                    bool pickup_at_depot = (ord.pickup_node == depot);
                    
                    int p_limit = pickup_at_depot ? 1 : (n + 1);
                    for (int p = 0; p < p_limit; ++p)
                    {
                        if (pickup_at_depot && p != 0)
                            continue;
                        
                        for (int d_pos = p + 1; d_pos <= n + 1; ++d_pos)
                        {
                            auto candidate = stops2;
                            auto candidate_orders = driver_orders[d2];
                            candidate_orders.insert(oi);
                            
                            if (!pickup_at_depot)
                            {
                                candidate.insert(candidate.begin() + p, ord.pickup_node);
                                candidate.insert(candidate.begin() + d_pos, ord.dropoff_node);
                            }
                            else
                            {
                                candidate.insert(candidate.begin() + (d_pos - 1), ord.dropoff_node);
                            }
                            
                            double obj1 = calculateDriverObjective(dm, depot, orders, test_stops1, test_orders1);
                            double obj2 = calculateDriverObjective(dm, depot, orders, candidate, candidate_orders);
                            
                            double new_total = current_total - 
                                              calculateDriverObjective(dm, depot, orders, driver_stops[d1], driver_orders[d1]) -
                                              calculateDriverObjective(dm, depot, orders, driver_stops[d2], driver_orders[d2]) +
                                              obj1 + obj2;
                            
                            if (new_total < best_total - 1e-9)
                            {
                                best_total = new_total;
                                best_oi = oi;
                                best_d1 = d1;
                                best_d2 = d2;
                                best_stops1 = test_stops1;
                                best_stops2 = candidate;
                                best_orders1 = test_orders1;
                                best_orders2 = candidate_orders;
                                improved = true;
                            }
                        }
                    }
                }
            }
        }
        
        if (improved)
        {
            driver_stops[best_d1] = best_stops1;
            driver_stops[best_d2] = best_stops2;
            driver_orders[best_d1] = best_orders1;
            driver_orders[best_d2] = best_orders2;
        }
    }
}

void AlgorithmsPhase3::interRouteSwapBest(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    if (num_drivers < 2)
        return;
    
    bool improved = true;
    while (improved)
    {
        improved = false;
        
        double best_total = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
        int best_oi1 = -1, best_oi2 = -1, best_d1 = -1, best_d2 = -1;
        std::vector<int> best_stops1, best_stops2;
        std::unordered_set<int> best_orders1, best_orders2;
        
        for (int d1 = 0; d1 < num_drivers; ++d1)
        {
            for (int d2 = d1 + 1; d2 < num_drivers; ++d2)
            {
                auto current_time = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(current_time - start_time).count();
                if (elapsed > time_limit)
                    return;
                
                if (driver_orders[d1].empty() || driver_orders[d2].empty())
                    continue;
                
                for (int oi1 : driver_orders[d1])
                {
                    for (int oi2 : driver_orders[d2])
                    {
                        double current_total = calculateTotalObjective(dm, depot, orders, driver_stops, driver_orders);
                        
                        const auto &ord1 = orders[oi1];
                        const auto &ord2 = orders[oi2];
                        
                        auto test_stops1 = driver_stops[d1];
                        auto test_stops2 = driver_stops[d2];
                        auto test_orders1 = driver_orders[d1];
                        auto test_orders2 = driver_orders[d2];
                        
                        test_orders1.erase(oi1);
                        test_orders2.erase(oi2);
                        test_orders1.insert(oi2);
                        test_orders2.insert(oi1);
                        
                        test_stops1.erase(std::remove(test_stops1.begin(), test_stops1.end(), ord1.pickup_node), test_stops1.end());
                        test_stops1.erase(std::remove(test_stops1.begin(), test_stops1.end(), ord1.dropoff_node), test_stops1.end());
                        test_stops2.erase(std::remove(test_stops2.begin(), test_stops2.end(), ord2.pickup_node), test_stops2.end());
                        test_stops2.erase(std::remove(test_stops2.begin(), test_stops2.end(), ord2.dropoff_node), test_stops2.end());
                        
                        if (ord2.pickup_node != depot)
                            test_stops1.push_back(ord2.pickup_node);
                        test_stops1.push_back(ord2.dropoff_node);
                        
                        if (ord1.pickup_node != depot)
                            test_stops2.push_back(ord1.pickup_node);
                        test_stops2.push_back(ord1.dropoff_node);
                        
                        if (isRouteValid(test_stops1, orders, test_orders1, depot) &&
                            isRouteValid(test_stops2, orders, test_orders2, depot))
                        {
                            double obj1 = calculateDriverObjective(dm, depot, orders, test_stops1, test_orders1);
                            double obj2 = calculateDriverObjective(dm, depot, orders, test_stops2, test_orders2);
                            
                            double new_total = current_total -
                                              calculateDriverObjective(dm, depot, orders, driver_stops[d1], driver_orders[d1]) -
                                              calculateDriverObjective(dm, depot, orders, driver_stops[d2], driver_orders[d2]) +
                                              obj1 + obj2;
                            
                            if (new_total < best_total - 1e-9)
                            {
                                best_total = new_total;
                                best_oi1 = oi1;
                                best_oi2 = oi2;
                                best_d1 = d1;
                                best_d2 = d2;
                                best_stops1 = test_stops1;
                                best_stops2 = test_stops2;
                                best_orders1 = test_orders1;
                                best_orders2 = test_orders2;
                                improved = true;
                            }
                        }
                    }
                }
            }
        }
        
        if (improved)
        {
            driver_stops[best_d1] = best_stops1;
            driver_stops[best_d2] = best_stops2;
            driver_orders[best_d1] = best_orders1;
            driver_orders[best_d2] = best_orders2;
        }
    }
}

void AlgorithmsPhase3::perturbSolutionAdaptive(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    int strength)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    
    int num_moves = std::min(2 + strength, (int)orders.size() / 3);
    
    for (int m = 0; m < num_moves; ++m)
    {
        std::vector<int> non_empty;
        for (int d = 0; d < num_drivers; ++d)
        {
            if (!driver_orders[d].empty())
                non_empty.push_back(d);
        }
        
        if (non_empty.empty())
            continue;
        
        int d1 = non_empty[gen() % non_empty.size()];
        std::vector<int> ords(driver_orders[d1].begin(), driver_orders[d1].end());
        int oi = ords[gen() % ords.size()];
        
        int d2 = gen() % num_drivers;
        int tries = 0;
        while (d2 == d1 && tries++ < 10)
            d2 = gen() % num_drivers;
        
        if (d2 == d1)
            continue;
        
        const auto &ord = orders[oi];
        
        driver_orders[d1].erase(oi);
        driver_stops[d1].erase(std::remove(driver_stops[d1].begin(), driver_stops[d1].end(), ord.pickup_node), driver_stops[d1].end());
        driver_stops[d1].erase(std::remove(driver_stops[d1].begin(), driver_stops[d1].end(), ord.dropoff_node), driver_stops[d1].end());
        
        driver_orders[d2].insert(oi);
        if (ord.pickup_node != depot)
        {
            int pos = driver_stops[d2].empty() ? 0 : (gen() % (driver_stops[d2].size() + 1));
            driver_stops[d2].insert(driver_stops[d2].begin() + pos, ord.pickup_node);
        }
        
        int dpos = driver_stops[d2].empty() ? 0 : (gen() % (driver_stops[d2].size() + 1));
        driver_stops[d2].insert(driver_stops[d2].begin() + dpos, ord.dropoff_node);
        
        if (!isRouteValid(driver_stops[d2], orders, driver_orders[d2], depot))
        {
            driver_stops[d2].erase(std::remove(driver_stops[d2].begin(), driver_stops[d2].end(), ord.dropoff_node), driver_stops[d2].end());
            driver_stops[d2].push_back(ord.dropoff_node);
        }
    }
}

// Legacy wrapper functions
void AlgorithmsPhase3::optimizeRoutesIntensive(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    optimizeRoutesIntensiveBest(dm, depot, orders, driver_stops, driver_orders, start_time, time_limit);
}

void AlgorithmsPhase3::interRouteRelocate(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    interRouteRelocateBest(dm, num_drivers, depot, orders, driver_stops, driver_orders, start_time, time_limit);
}

void AlgorithmsPhase3::interRouteSwap(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    interRouteSwapBest(dm, num_drivers, depot, orders, driver_stops, driver_orders, start_time, time_limit);
}

void AlgorithmsPhase3::twoOptBetweenRoutes(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders,
    std::chrono::high_resolution_clock::time_point start_time,
    double time_limit)
{
    // Not implemented
}

void AlgorithmsPhase3::perturbSolution(
    DistanceManager &dm,
    int num_drivers,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops,
    std::vector<std::unordered_set<int>> &driver_orders)
{
    perturbSolutionAdaptive(dm, num_drivers, depot, orders, driver_stops, driver_orders, 1);
}

bool AlgorithmsPhase3::isRouteValid(
    const std::vector<int> &stops,
    const std::vector<Order> &orders,
    const std::unordered_set<int> &order_indices,
    int depot)
{
    std::unordered_set<int> picked;
    
    for (int oi : order_indices)
    {
        const auto &o = orders[oi];
        if (o.pickup_node == depot)
        {
            picked.insert(oi);
        }
    }
    
    for (int node : stops)
    {
        for (int oi : order_indices)
        {
            const auto &o = orders[oi];
            if (o.pickup_node == node)
            {
                picked.insert(oi);
            }
            if (o.dropoff_node == node)
            {
                if (!picked.count(oi))
                    return false;
            }
        }
    }
    
    return true;
}

void AlgorithmsPhase3::optimizeRoutes(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    std::vector<std::vector<int>> &driver_stops)
{
    std::vector<std::unordered_set<int>> driver_orders(driver_stops.size());
    
    std::unordered_map<int, std::vector<int>> node_to_orders;
    for (size_t i = 0; i < orders.size(); ++i)
    {
        node_to_orders[orders[i].pickup_node].push_back(i);
        node_to_orders[orders[i].dropoff_node].push_back(i);
    }
    
    for (size_t d = 0; d < driver_stops.size(); ++d)
    {
        for (int node : driver_stops[d])
        {
            if (node_to_orders.count(node))
            {
                for (int oi : node_to_orders[node])
                {
                    const auto &o = orders[oi];
                    bool has_pickup = (o.pickup_node == depot);
                    bool has_dropoff = false;
                    
                    for (int n : driver_stops[d])
                    {
                        if (n == o.pickup_node)
                            has_pickup = true;
                        if (n == o.dropoff_node)
                            has_dropoff = true;
                    }
                    
                    if (has_pickup && has_dropoff)
                    {
                        driver_orders[d].insert(oi);
                    }
                }
            }
        }
    }

    auto start = std::chrono::high_resolution_clock::now();
    optimizeRoutesIntensiveBest(dm, depot, orders, driver_stops, driver_orders, start, 10.0);
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

    for (const auto &o : orders)
    {
        if (o.pickup_node == depot)
        {
            bool dropoff_in_route = false;
            for (int node : stops)
            {
                if (node == o.dropoff_node)
                {
                    dropoff_in_route = true;
                    break;
                }
            }
            if (dropoff_in_route)
            {
                if (time < o.ready_time)
                    time = o.ready_time;
                picked_orders.insert(o.order_id);
            }
        }
    }

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