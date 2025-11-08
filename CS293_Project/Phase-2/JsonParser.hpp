#ifndef JSON_PARSER_HPP
#define JSON_PARSER_HPP

#include "Graph.hpp"
#include "Algorithms.hpp"
#include <string>
#include <fstream>
#include "../json.hpp"

using json = nlohmann::json;

class JsonParser {
public:
    static Graph parse_graph(const std::string& filename);
    static json parse_queries(const std::string& filename);
    static void write_output(const std::string& filename, const json& output);
    
private:
    static Node parse_node(const json& node_json);
    static Edge parse_edge(const json& edge_json);
};

#endif
