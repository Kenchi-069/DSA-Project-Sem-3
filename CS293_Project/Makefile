CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -I. -march=native
LDFLAGS = 

PHASE1_DIR = Phase-1
PHASE2_DIR = Phase-2
PHASE3_DIR = Phase-3

PHASE1_SOURCES = $(PHASE1_DIR)/Graph.cpp $(PHASE1_DIR)/Algorithms.cpp \
                 $(PHASE1_DIR)/JsonParser.cpp $(PHASE1_DIR)/QueryHandler.cpp \
                 $(PHASE1_DIR)/SampleDriver.cpp

PHASE2_SOURCES = $(PHASE2_DIR)/Graph.cpp $(PHASE2_DIR)/Algorithms.cpp \
                 $(PHASE2_DIR)/JsonParser.cpp $(PHASE2_DIR)/QueryHandler.cpp \
                 $(PHASE2_DIR)/SampleDriver.cpp

PHASE3_SOURCES = $(PHASE3_DIR)/SampleDriver.cpp

PHASE1_OBJECTS = $(PHASE1_SOURCES:.cpp=.o)
PHASE2_OBJECTS = $(PHASE2_SOURCES:.cpp=.o)
PHASE3_OBJECTS = $(PHASE3_SOURCES:.cpp=.o)

EXECUTABLES = phase1 phase2 phase3

.PHONY: all clean phase1 phase2 phase3 test

all: $(EXECUTABLES)

phase1: $(PHASE1_OBJECTS)
	@echo "Linking phase1..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "phase1 built successfully!"

phase2: $(PHASE2_OBJECTS)
	@echo "Linking phase2..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "phase2 built successfully!"

phase3: $(PHASE3_OBJECTS)
	@echo "Linking phase3..."
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "phase3 built successfully!"

$(PHASE1_DIR)/%.o: $(PHASE1_DIR)/%.cpp
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(PHASE2_DIR)/%.o: $(PHASE2_DIR)/%.cpp
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(PHASE3_DIR)/%.o: $(PHASE3_DIR)/%.cpp
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@echo "Cleaning..."
	rm -f $(EXECUTABLES) $(PHASE1_DIR)/*.o $(PHASE2_DIR)/*.o $(PHASE3_DIR)/*.o
	@echo "Clean complete!"

test: all
	@echo "Running tests..."
	python3 generate_tests.py
	./phase1 test_graph.json test_queries_phase1.json output1.json
	./phase2 test_graph.json test_queries_phase2.json output2.json
	@echo "Tests complete!"

help:
	@echo "CS293 Project Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build all phases (default)"
	@echo "  phase1  - Build Phase 1 only"
	@echo "  phase2  - Build Phase 2 only"
	@echo "  phase3  - Build Phase 3 only"
	@echo "  test    - Build and run basic tests"
	@echo "  clean   - Remove all built files"
	@echo "  help    - Show this help"
	@echo ""
	@echo "Usage:"
	@echo "  ./phase1 <graph.json> <queries.json> <output.json>"
	@echo "  ./phase2 <graph.json> <queries.json> <output.json>"
	@echo "  ./phase3 <graph.json> <queries.json> <output.json>"
