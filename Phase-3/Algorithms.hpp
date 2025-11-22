#ifndef ALGORITHMS_PHASE3_HPP
#define ALGORITHMS_PHASE3_HPP

#include "Graph.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <chrono>

struct Order
{
    int order_id;
    int pickup_node;
    int dropoff_node;

    double priority;
    double ready_time;

    Order() : order_id(-1), pickup_node(-1), dropoff_node(-1), priority(1.0), ready_time(0.0) {}
};

class DistanceManager
{
private:
    const Graph &graph;
    std::unordered_map<int, std::unordered_map<int, double>> time_cache;

public:
    DistanceManager(const Graph &g) : graph(g) {}
    
    double getTime(int from, int to);
    std::vector<int> getPath(int from, int to);
    void clearCache() { time_cache.clear(); }
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

class AlgorithmsPhase3
{
public:
    static ScheduleResult solveDeliveryScheduling(
        const Graph &graph,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders
    );

private:
    static void performSmartInsertion(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::unordered_set<int> &unassigned,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    // Best improvement strategies (not first improvement)
    static void optimizeRoutesIntensiveBest(
        DistanceManager &dm,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    static void twoOptIntraRoute(
        DistanceManager &dm,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    static void interRouteRelocateBest(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    static void interRouteSwapBest(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    static void perturbSolutionAdaptive(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        int strength
    );

    // Legacy wrapper functions for compatibility
    static void optimizeRoutesIntensive(
        DistanceManager &dm,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    static void interRouteRelocate(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    static void interRouteSwap(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    static void twoOptBetweenRoutes(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders,
        std::chrono::high_resolution_clock::time_point start_time,
        double time_limit
    );

    static void perturbSolution(
        DistanceManager &dm,
        int num_drivers,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops,
        std::vector<std::unordered_set<int>> &driver_orders
    );

    static bool isRouteValid(
        const std::vector<int> &stops,
        const std::vector<Order> &orders,
        const std::unordered_set<int> &order_indices,
        int depot
    );

    static void optimizeRoutes(
        DistanceManager &dm,
        int depot,
        const std::vector<Order> &orders,
        std::vector<std::vector<int>> &driver_stops
    );

    static double evaluateRouteScore(
        DistanceManager &dm,
        int depot,
        const std::vector<int> &stops,
        const std::vector<Order> &orders
    );

    static double calculateInsertionCost(
        DistanceManager &dm,
        int prev,
        int next,
        int insert
    );
};

double calculateDriverObjective(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    const std::vector<int> &stops,
    const std::unordered_set<int> &order_indices
);

double calculateTotalObjective(
    DistanceManager &dm,
    int depot,
    const std::vector<Order> &orders,
    const std::vector<std::vector<int>> &driver_stops,
    const std::vector<std::unordered_set<int>> &driver_orders
);

#endif