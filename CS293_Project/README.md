# CS293 Graph-Based Routing System v3.0

**COMPLETELY REWRITTEN** with all critical fixes and complete Phase separation.
**MEMORY MODEL**: This version now uses `std::unique_ptr` to store all `Node` and `Edge` objects on the heap, demonstrating modern C++ memory management and automatic resource cleanup (RAII).

## 🔥 What's New in v3.0

### Critical Fixes Implemented
✅ **Fixed Yen's Algorithm** - Now correctly forbids EDGES (not nodes)
✅ **Edge-level constraints** - Proper Dijkstra with forbidden edge support
✅ **Maps instead of vectors** - No more fixed-size arrays, handles sparse node IDs
✅ **Complete Phase separation** - Phase-1 and Phase-2 are totally independent
✅ **POI error handling** - Graceful handling of missing/empty POIs
✅ **Edge validation** - Checks node existence before adding edges
✅ **A* implementation** - Fast approximate shortest paths
✅ **Performance optimizations** - Compiler flags, better algorithms

### Architecture Improvements
- **Phase-1**: Only handles shortest path, KNN, dynamic updates
- **Phase-2**: Only handles k-shortest paths, heuristics, approximate
- **`std::unique_ptr`**: `Node` and `Edge` objects are stored as `std::unique_ptr` in the maps, ensuring they are on the heap and that their memory is *automatically* managed. No manual `delete` or complex destructors needed.
- No cross-contamination between phases
- Uses `unordered_map<int, double>` for distances (not fixed vectors)
- Proper error handling with try-catch blocks

## 🚀 Quick Start

```bash
# Build all
make all

# Generate tests
python3 generate_tests.py

# Run Phase 1
./phase1 test_graph.json test_queries_phase1.json output1.json

# Run Phase 2
./phase2 test_graph.json test_queries_phase2.json output2.json

# View results
python3 -m json.tool output1.json | head -50
