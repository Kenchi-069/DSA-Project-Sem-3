#ifndef JSON_PARSER_PHASE3_HPP
#define JSON_PARSER_PHASE3_HPP

#include "Graph.hpp"
#include "Algorithms.hpp"
#include "../json.hpp"
#include <string>

using json = nlohmann::json;

class JsonParserPhase3
{
public:
    static Graph parseGraph(const std::string &filename);
    static json parseInput(const std::string &filename);
    static void writeOutput(const std::string &filename, const json &output);

    static std::vector<Order> parseOrders(const json &j);
    static int parseFleetSize(const json &j);
    static int parseDepot(const json &j);

private:
    static Node parseNode(const json &j);
    static Edge parseEdge(const json &j);
};

#endif