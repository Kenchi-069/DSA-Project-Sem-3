#include "Graph.hpp"
#include "JsonParser.hpp"
#include "QueryHandler.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cerr << "Usage: ./phase3 <graph.json> <input.json> <output.json>" << std::endl;
        return 1;
    }

    std::string graph_file = argv[1];
    std::string input_file = argv[2];
    std::string output_file = argv[3];

    try
    {
        std::cout << "=== Phase 3: Delivery Scheduling ===" << std::endl;

        auto t1 = std::chrono::high_resolution_clock::now();
        Graph graph = JsonParserPhase3::parseGraph(graph_file);
        std::cout << "Graph Loaded" << std::endl;
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Graph Loaded in "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;

        json input_queries = JsonParserPhase3::parseInput(input_file);

        std::cout << "Scheduling..." << std::endl;
        QueryHandlerPhase3 handler(graph);
        json result = handler.solveScheduling(input_queries);

        JsonParserPhase3::writeOutput(output_file, result);
        std::cout << "Done. Metrics: " << result["metrics"].dump() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}