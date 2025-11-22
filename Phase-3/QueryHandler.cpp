#include "QueryHandler.hpp"
#include "JsonParser.hpp"
#include <chrono>

json QueryHandlerPhase3::solveScheduling(const json &input_json)
{
    json output;
    output["results"] = json::array();

    if (input_json.contains("meta"))
        output["meta"] = input_json["meta"];

    if (!input_json.contains("events"))
        return output;

    for (const auto &event : input_json["events"])
    {
        json result;

        try
        {
            auto t1 = std::chrono::steady_clock::now();

            int num_drivers = JsonParserPhase3::parseFleetSize(event);
            int depot = JsonParserPhase3::parseDepot(event);
            std::vector<Order> orders = JsonParserPhase3::parseOrders(event);

            ScheduleResult res =
                AlgorithmsPhase3::solveDeliveryScheduling(graph, num_drivers, depot, orders);

            // Build result
            result["assignments"] = json::array();

            for (const auto &assign : res.assignments)
            {
                json a;
                a["driver_id"] = assign.driver_id;
                a["route"] = assign.route_path;
                a["order_ids"] = assign.order_ids;
                result["assignments"].push_back(a);
            }

            result["metrics"] = {
                {"total_delivery_time_s", res.total_delivery_time}};

            auto t2 = std::chrono::steady_clock::now();
            result["processing_time_ms"] =
                std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        }
        catch (const std::exception &e)
        {
            result["error"] = e.what();
        }
        catch (...)
        {
            result["error"] = "Unknown error in Phase-3 event.";
        }

        output["results"].push_back(result);
    }

    return output;
}
