#include "QueryHandler.hpp"
#include "JsonParser.hpp"
#include <chrono>

json QueryHandlerPhase3::solveScheduling(const json &input_json)
{
    int num_drivers = JsonParserPhase3::parseFleetSize(input_json);
    int depot = JsonParserPhase3::parseDepot(input_json);
    std::vector<Order> orders = JsonParserPhase3::parseOrders(input_json);

    ScheduleResult res = AlgorithmsPhase3::solveDeliveryScheduling(graph, num_drivers, depot, orders);

    json output;
    output["assignments"] = json::array();

    for (const auto &assign : res.assignments)
    {
        json a;
        a["driver_id"] = assign.driver_id;
        a["route"] = assign.route_path;
        a["order_ids"] = assign.order_ids;
        output["assignments"].push_back(a);
    }

    output["metrics"] = {
        {"total_delivery_time_s", res.total_delivery_time}};

    return output;
}