#include "Graph.hpp"
#include "Algorithms.hpp"
#include "JsonParser.hpp"
#include "QueryHandler.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }
    
    std::string graph_file = argv[1];
    std::string queries_file = argv[2];
    std::string output_file = argv[3];
    
    try {
        std::cout << "=== Phase 1 ===" << std::endl;
        std::cout << "Loading graph..." << std::endl;
        auto t1 = std::chrono::high_resolution_clock::now();
        Graph graph = JsonParser::parse_graph(graph_file);
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Graph loaded: " << graph.get_nodes().size() << " nodes, " 
                  << graph.get_edges().size() << " edges ("
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << " ms)" << std::endl;
        
        std::cout << "Preprocessing..." << std::endl;
        auto t3 = std::chrono::high_resolution_clock::now();
        // No preprocessing for Phase 1
        auto t4 = std::chrono::high_resolution_clock::now();
        std::cout << "Preprocessing done (" 
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t4-t3).count() << " ms)" << std::endl;
        
        std::cout << "Loading queries..." << std::endl;
        json queries = JsonParser::parse_queries(queries_file);
        
        std::cout << "Processing queries..." << std::endl;
        QueryHandler handler(graph);
        json output = handler.process_queries(queries);
        
        std::cout << "Writing output..." << std::endl;
        JsonParser::write_output(output_file, output);
        std::cout << "Done!" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
