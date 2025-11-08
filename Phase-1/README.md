# Phase-1: Graph-Based Routing System

## Overview
This directory contains the implementation for Phase-1 of the CS293 project.

## Building
```bash
make
```

This will create the `phase1` executable.

## Running
```bash
./phase1 <graph.json> <queries.json> <output.json>
```

## Dependencies
- C++17 compiler (g++ or clang++)
- nlohmann/json library (json.hpp header)

## Installation of JSON Library

### Option 1: Download header file
```bash
wget https://github.com/nlohmann/json/releases/download/v3.11.2/json.hpp
```

### Option 2: Using package manager (Ubuntu/Debian)
```bash
sudo apt-get install nlohmann-json3-dev
```

### Option 3: Using Homebrew (macOS)
```bash
brew install nlohmann-json
```

## File Structure
- `graph.hpp/cpp`: Graph data structure and operations
- `shortest_path.hpp/cpp`: Dijkstra's algorithm implementation
- `knn.hpp/cpp`: K-Nearest Neighbor search
- `json_utils.hpp/cpp`: JSON parsing and query processing
- `SampleDriver.cpp`: Main driver program
- `Makefile`: Build configuration

## Features Implemented
1. **Graph Management**
   - Dynamic edge removal
   - Edge modification

2. **Shortest Path**
   - Distance-based routing
   - Time-based routing with traffic profiles
   - Support for constraints (forbidden nodes/roads)

3. **K-Nearest Neighbors**
   - Euclidean distance metric
   - Shortest path distance metric

## Testing
Create test cases in JSON format according to the project specification.

Example:
```bash
./phase1 test_graph.json test_queries.json output.json
```
