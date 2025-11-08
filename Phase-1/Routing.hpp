#ifndef ROUTING_HPP
#define ROUTING_HPP

#include "Graph.hpp"
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <limits>

struct PathResult {
    bool possible;
    double minimumDistance;
    double minimumTime;
    std::vector<int> path;
};

struct KNNResult {
    std::vector<int> nodes;
};

class Routing {
public:
    Routing(const Graph& graph);
    ~Routing();

    PathResult shortestPathDistance(
        int source, 
        int target,
        const std::set<int>& forbiddenNodes = {},
        const std::set<std::string>& forbiddenRoadTypes = {}
    );

    PathResult shortestPathTime(
        int source, 
        int target,
        double timeOfDay = 0.0,
        const std::set<int>& forbiddenNodes = {},
        const std::set<std::string>& forbiddenRoadTypes = {}
    );

    KNNResult knnEuclidean(
        double lat, 
        double lon, 
        const std::string& poi,
        int k
    );

    KNNResult knnShortestPath(
        double lat, 
        double lon, 
        const std::string& poi,
        int k
    );

private:
    const Graph& graph;

    struct DijkstraNode {
        int nodeId;
        double distance;
        
        bool operator>(const DijkstraNode& other) const {
            return distance > other.distance;
        }
    };

    PathResult dijkstraDistance(
        int source,
        int target,
        const std::set<int>& forbiddenNodes,
        const std::set<std::string>& forbiddenRoadTypes
    );

    PathResult dijkstraTime(
        int source,
        int target,
        double timeOfDay,
        const std::set<int>& forbiddenNodes,
        const std::set<std::string>& forbiddenRoadTypes
    );

    int getTimeSlot(double secondsFromMidnight) const;
    double getSpeedAtTime(const Edge& edge, double secondsFromMidnight) const;

    std::vector<int> reconstructPath(
        const std::vector<int>& parent,
        int source,
        int target
    );

    int findNearestNode(double lat, double lon) const;
};

#endif // ROUTING_HPP
