#ifndef JSON_PARSER_HPP
#define JSON_PARSER_HPP

#include "Graph.hpp"
#include "Algorithms.hpp"
#include <string>
#include <fstream>
#include "../json.hpp"

using json = nlohmann::json;

class JsonParser
{
public:
    static Graph parseGraph(const std::string &filename);
    static json parseQueries(const std::string &filename);
    static void writeOutput(const std::string &filename, const json &output);

private:
    static Node parseNode(const json &node_json);
    static Edge parseEdge(const json &edge_json);
};

#endif
