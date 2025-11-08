CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I.
LDFLAGS = 

# Source files
SOURCES = graph.cpp shortest_path.cpp knn.cpp json_utils.cpp SampleDriver.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Target executable
TARGET = phase1

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

# Dependencies
graph.o: graph.cpp graph.hpp
shortest_path.o: shortest_path.cpp shortest_path.hpp graph.hpp
knn.o: knn.cpp knn.hpp graph.hpp shortest_path.hpp
json_utils.o: json_utils.cpp json_utils.hpp graph.hpp shortest_path.hpp knn.hpp
SampleDriver.o: SampleDriver.cpp json_utils.hpp
