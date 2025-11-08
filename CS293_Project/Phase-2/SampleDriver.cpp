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
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    std::string graph_file = argv[1];
    std::string queries_file = argv[2];
    std::string output_file = argv[3];

    try
    {
        std::cout << "=== Phase 2 ===" << std::endl;
        std::cout << "Loading graph..." << std::endl;
        auto t1 = std::chrono::high_resolution_clock::now();
        Graph graph = JsonParser::parseGraph(graph_file);
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Graph loaded: " << graph.getNodes().size() << " nodes, "
                  << graph.getEdges().size() << " edges ("
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms)" << std::endl;

        std::cout << "Preprocessing (can take up to 5 min)..." << std::endl;
        auto t3 = std::chrono::high_resolution_clock::now();
        // TODO: Add preprocessing like landmark-based heuristics, contraction hierarchies, etc.
        // For now: no preprocessing
        auto t4 = std::chrono::high_resolution_clock::now();
        std::cout << "Preprocessing done ("
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() << " ms)" << std::endl;

        std::cout << "Loading queries..." << std::endl;
        json queries = JsonParser::parseQueries(queries_file);

        std::cout << "Processing Phase-2 queries..." << std::endl;
        QueryHandlerPhase2 handler(graph);
        json output = handler.processQueries(queries);

        std::cout << "Writing output..." << std::endl;
        JsonParser::writeOutput(output_file, output);
        std::cout << "Done!" << std::endl;

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
