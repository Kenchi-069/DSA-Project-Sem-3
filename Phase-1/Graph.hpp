#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <unordered_map>
#include <map>
#include <set>
#include <cmath>
#include <limits>
#include <memory>

struct Node {
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;
};

struct Edge {
    int id;
    int u;
    int v;
    double length;
    double averageTime;
    std::vector<double> speedProfile; // 96 slots for 15-min intervals
    std::string roadType;
    bool oneWay;
    bool active;

    Edge() : active(true) {}
};

class Graph {
private:
    std::unordered_map<int, Node> nodes;
    std::unordered_map<int, Edge> edges;
    std::unordered_map<int, std::vector<int>> adjList;
    std::unordered_map<int, std::set<std::string>> nodePoIs;

public:
    Graph();
    ~Graph();

    void addNode(const Node& node);
    void addEdge(const Edge& edge);
    void removeEdge(int edgeId);
    void modifyEdge(int edgeId, const std::map<std::string, double>& patch);
    void restoreEdge(int edgeId, const std::map<std::string, double>& patch);

    const Node* getNode(int nodeId) const;
    const Edge* getEdge(int edgeId) const;
    const std::vector<int>& getNeighbors(int nodeId) const;
    const std::set<std::string>& getNodePoIs(int nodeId) const;

    bool hasNode(int nodeId) const;
    bool hasEdge(int edgeId) const;
    bool isEdgeActive(int edgeId) const;

    int getNodeCount() const;
    int getEdgeCount() const;
    const std::unordered_map<int, Edge>& getAllEdges() const;
    const std::unordered_map<int, Node>& getAllNodes() const;

    double getEuclideanDistance(double lat1, double lon1, double lat2, double lon2) const;

private:
    std::vector<int> emptyNeighbors;
    std::set<std::string> emptyPoIs;
};

#endif // GRAPH_HPP
