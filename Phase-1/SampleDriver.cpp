#include "json_utils.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }
    
    std::string graph_file = argv[1];
    std::string query_file = argv[2];
    std::string output_file = argv[3];
    
    try {
        // Parse graph
        std::cout << "Loading graph..." << std::endl;
        Graph graph = JSONParser::parseGraph(graph_file);
        
        // Parse queries
        std::cout << "Loading queries..." << std::endl;
        json queries = JSONParser::parseQueries(query_file);
        
        // Process queries
        std::cout << "Processing queries..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        
        QueryProcessor processor(graph);
        json output = processor.processQueries(queries);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        output["processing_time_ms"] = duration.count();
        
        // Write output
        std::cout << "Writing output..." << std::endl;
        JSONParser::writeOutput(output_file, output);
        
        std::cout << "Done! Processing time: " << duration.count() << "ms" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
