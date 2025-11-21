#ifndef ALGORITHMS_PHASE3_HPP
#define ALGORITHMS_PHASE3_HPP

#include "Graph.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>

struct Order
{
    int order_id;
    int pickup_node;
    int dropoff_node;

    double priority;
    double ready_time;

    Order() : order_id(-1), pickup_node(-1), dropoff_node(-1), priority(1.0), ready_time(0.0) {}
};

struct DriverRoute
{
    int driver_id;
    std::vector<int> route_path;
    std::vector<int> order_ids;
    double completion_time;
};

struct ScheduleResult
{
    std::vector<DriverRoute> assignments;
    double total_delivery_time;
};

class DistanceManager
{
    const Graph &graph;
    std::unordered_map<int, std::unordered_map<int, double>> time_cache;

public:
    DistanceManager(const Graph &g) : graph(g) {}

    double getTime(int from, int to);

    std::vector<int> getPath(int from, int to);
};

class AlgorithmsPhase3
{
public:
    static ScheduleResult solveDeliveryScheduling(
        const Graph &graph,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders);

private:
    static void performRegretInsertion(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::unordered_set<int> &unassigned);

    static void optimizeRoutes(
        DistanceManager &dm,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops);

    static double evaluateRouteScore(
        DistanceManager &dm,
        int depot,
        const std::vector<int> &stops,
        const std::vector<Order> &orders);

    static double calculateInsertionCost(
        DistanceManager &dm,
        int prev_node,
        int next_node,
        int insert_node);
};

#endif