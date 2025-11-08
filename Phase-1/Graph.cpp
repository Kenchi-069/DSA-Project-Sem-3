#include "Graph.hpp"
#include <cmath>

Graph::Graph() {}

Graph::~Graph() {}

void Graph::addNode(const Node& node) {
    nodes[node.id] = node;
    for (const auto& poi : node.pois) {
        nodePoIs[node.id].insert(poi);
    }
}

void Graph::addEdge(const Edge& edge) {
    edges[edge.id] = edge;
    adjList[edge.u].push_back(edge.id);
    if (!edge.oneWay) {
        adjList[edge.v].push_back(edge.id);
    }
}

void Graph::removeEdge(int edgeId) {
    if (edges.find(edgeId) != edges.end()) {
        edges[edgeId].active = false;
    }
}

void Graph::modifyEdge(int edgeId, const std::map<std::string, double>& patch) {
    if (edges.find(edgeId) != edges.end()) {
        Edge& edge = edges[edgeId];
        if (patch.find("length") != patch.end()) {
            edge.length = patch.at("length");
        }
        if (patch.find("averagetime") != patch.end()) {
            edge.averageTime = patch.at("averagetime");
        }
        edge.active = true;
    }
}

void Graph::restoreEdge(int edgeId, const std::map<std::string, double>& patch) {
    if (edges.find(edgeId) != edges.end()) {
        Edge& edge = edges[edgeId];
        if (!patch.empty()) {
            if (patch.find("length") != patch.end()) {
                edge.length = patch.at("length");
            }
            if (patch.find("averagetime") != patch.end()) {
                edge.averageTime = patch.at("averagetime");
            }
        }
        edge.active = true;
    }
}

const Node* Graph::getNode(int nodeId) const {
    auto it = nodes.find(nodeId);
    if (it != nodes.end()) {
        return &it->second;
    }
    return nullptr;
}

const Edge* Graph::getEdge(int edgeId) const {
    auto it = edges.find(edgeId);
    if (it != edges.end()) {
        return &it->second;
    }
    return nullptr;
}

const std::vector<int>& Graph::getNeighbors(int nodeId) const {
    auto it = adjList.find(nodeId);
    if (it != adjList.end()) {
        return it->second;
    }
    return emptyNeighbors;
}

const std::set<std::string>& Graph::getNodePoIs(int nodeId) const {
    auto it = nodePoIs.find(nodeId);
    if (it != nodePoIs.end()) {
        return it->second;
    }
    return emptyPoIs;
}

bool Graph::hasNode(int nodeId) const {
    return nodes.find(nodeId) != nodes.end();
}

bool Graph::hasEdge(int edgeId) const {
    return edges.find(edgeId) != edges.end();
}

bool Graph::isEdgeActive(int edgeId) const {
    auto it = edges.find(edgeId);
    if (it != edges.end()) {
        return it->second.active;
    }
    return false;
}

int Graph::getNodeCount() const {
    return nodes.size();
}

int Graph::getEdgeCount() const {
    return edges.size();
}

const std::unordered_map<int, Edge>& Graph::getAllEdges() const {
    return edges;
}

const std::unordered_map<int, Node>& Graph::getAllNodes() const {
    return nodes;
}

double Graph::getEuclideanDistance(double lat1, double lon1, double lat2, double lon2) const {
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    return std::sqrt(dlat * dlat + dlon * dlon);
}
