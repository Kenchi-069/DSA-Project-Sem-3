#include "Graph.hpp"
#include "Algorithms.hpp"
#include "JsonParser.hpp"
#include "QueryHandler.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
    }

    std::string graph_file = argv[1];
    std::string queries_file = argv[2];
    std::string output_file = argv[3];

    try
    {
        std::cout << "Phase 1" << std::endl;
        std::cout << "Loading Graph" << std::endl;
        Graph graph = JsonParser::parseGraph(graph_file);
        std::cout << "Graph Loaded: " << graph.getNodes().size() << " nodes, "
                  << graph.getEdges().size() << " edges" << std::endl;

        std::cout << "Loading Queries" << std::endl;
        json queries = JsonParser::parseQueries(queries_file);

        std::cout << "Processing Queries..." << std::endl;
        QueryHandler handler(graph);
        json output = handler.processQueries(queries);

        std::cout << "Writing Output" << std::endl;
        JsonParser::writeOutput(output_file, output);
        std::cout << "Finished" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}
