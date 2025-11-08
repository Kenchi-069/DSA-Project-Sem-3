CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I./
LDFLAGS = -lm

# Directories
PHASE1_DIR = Phase-1
PHASE2_DIR = Phase-2
PHASE3_DIR = Phase-3

# Source files for Phase-1
PHASE1_SRCS = $(PHASE1_DIR)/Graph.cpp $(PHASE1_DIR)/Routing.cpp $(PHASE1_DIR)/SampleDriver.cpp
PHASE1_OBJS = $(PHASE1_SRCS:.cpp=.o)

# Output binaries
PHASE1_BIN = phase1
PHASE2_BIN = phase2
PHASE3_BIN = phase3

# Phony targets
.PHONY: all phase1 phase2 phase3 clean

all: phase1

phase1: $(PHASE1_BIN)

$(PHASE1_BIN): $(PHASE1_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

phase2:
	@echo "Phase-2 implementation pending"

phase3:
	@echo "Phase-3 implementation pending"

clean:
	rm -f $(PHASE1_OBJS) $(PHASE1_BIN)
	@echo "Cleaned up build artifacts"

rebuild: clean all
