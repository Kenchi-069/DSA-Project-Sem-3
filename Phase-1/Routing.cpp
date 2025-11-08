#include "Routing.hpp"
#include <queue>
#include <algorithm>
#include <cmath>

Routing::Routing(const Graph& graph) : graph(graph) {}

Routing::~Routing() {}

int Routing::getTimeSlot(double secondsFromMidnight) const {
    int slot = static_cast<int>(secondsFromMidnight / 900);
    if (slot >= 96) slot = 95;
    if (slot < 0) slot = 0;
    return slot;
}

double Routing::getSpeedAtTime(const Edge& edge, double secondsFromMidnight) const {
    if (edge.speedProfile.empty()) {
        return edge.averageTime > 0 ? edge.length / edge.averageTime : 0;
    }
    
    int slot = getTimeSlot(secondsFromMidnight);
    return edge.speedProfile[slot];
}

std::vector<int> Routing::reconstructPath(
    const std::vector<int>& parent,
    int source,
    int target
) {
    std::vector<int> path;
    if (parent[target] == -1 && source != target) {
        return path;
    }
    
    int current = target;
    while (current != -1) {
        path.push_back(current);
        if (current == source) break;
        current = parent[current];
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

PathResult Routing::dijkstraDistance(
    int source,
    int target,
    const std::set<int>& forbiddenNodes,
    const std::set<std::string>& forbiddenRoadTypes
) {
    PathResult result;
    result.possible = false;
    result.minimumDistance = std::numeric_limits<double>::infinity();
    result.minimumTime = 0;

    if (!graph.hasNode(source) || !graph.hasNode(target)) {
        return result;
    }

    if (forbiddenNodes.find(source) != forbiddenNodes.end() ||
        forbiddenNodes.find(target) != forbiddenNodes.end()) {
        return result;
    }

    std::vector<double> dist(graph.getNodeCount(), std::numeric_limits<double>::infinity());
    std::vector<int> parent(graph.getNodeCount(), -1);
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;

    dist[source] = 0;
    pq.push({source, 0.0});

    while (!pq.empty()) {
        auto [nodeId, d] = pq.top();
        pq.pop();

        if (d > dist[nodeId]) continue;
        if (nodeId == target) break;

        for (int edgeId : graph.getNeighbors(nodeId)) {
            const Edge* edge = graph.getEdge(edgeId);
            if (!edge || !edge->active) continue;

            if (forbiddenRoadTypes.find(edge->roadType) != forbiddenRoadTypes.end()) {
                continue;
            }

            int nextNode = (edge->u == nodeId) ? edge->v : edge->u;

            if (forbiddenNodes.find(nextNode) != forbiddenNodes.end()) {
                continue;
            }

            double newDist = dist[nodeId] + edge->length;
            if (newDist < dist[nextNode]) {
                dist[nextNode] = newDist;
                parent[nextNode] = nodeId;
                pq.push({nextNode, newDist});
            }
        }
    }

    if (dist[target] != std::numeric_limits<double>::infinity()) {
        result.possible = true;
        result.minimumDistance = dist[target];
        result.path = reconstructPath(parent, source, target);
    }

    return result;
}

PathResult Routing::shortestPathDistance(
    int source,
    int target,
    const std::set<int>& forbiddenNodes,
    const std::set<std::string>& forbiddenRoadTypes
) {
    return dijkstraDistance(source, target, forbiddenNodes, forbiddenRoadTypes);
}

PathResult Routing::dijkstraTime(
    int source,
    int target,
    double timeOfDay,
    const std::set<int>& forbiddenNodes,
    const std::set<std::string>& forbiddenRoadTypes
) {
    PathResult result;
    result.possible = false;
    result.minimumTime = std::numeric_limits<double>::infinity();
    result.minimumDistance = 0;

    if (!graph.hasNode(source) || !graph.hasNode(target)) {
        return result;
    }

    if (forbiddenNodes.find(source) != forbiddenNodes.end() ||
        forbiddenNodes.find(target) != forbiddenNodes.end()) {
        return result;
    }

    std::vector<double> dist(graph.getNodeCount(), std::numeric_limits<double>::infinity());
    std::vector<int> parent(graph.getNodeCount(), -1);
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;

    dist[source] = 0;
    pq.push({source, 0.0});

    while (!pq.empty()) {
        auto [nodeId, d] = pq.top();
        pq.pop();

        if (d > dist[nodeId]) continue;
        if (nodeId == target) break;

        for (int edgeId : graph.getNeighbors(nodeId)) {
            const Edge* edge = graph.getEdge(edgeId);
            if (!edge || !edge->active) continue;

            if (forbiddenRoadTypes.find(edge->roadType) != forbiddenRoadTypes.end()) {
                continue;
            }

            int nextNode = (edge->u == nodeId) ? edge->v : edge->u;

            if (forbiddenNodes.find(nextNode) != forbiddenNodes.end()) {
                continue;
            }

            double currentTime = timeOfDay + dist[nodeId];
            double speed = getSpeedAtTime(*edge, currentTime);
            double travelTime = speed > 0 ? edge->length / speed : edge->averageTime;
            double newDist = dist[nodeId] + travelTime;

            if (newDist < dist[nextNode]) {
                dist[nextNode] = newDist;
                parent[nextNode] = nodeId;
                pq.push({nextNode, newDist});
            }
        }
    }

    if (dist[target] != std::numeric_limits<double>::infinity()) {
        result.possible = true;
        result.minimumTime = dist[target];
        result.path = reconstructPath(parent, source, target);
    }

    return result;
}

PathResult Routing::shortestPathTime(
    int source,
    int target,
    double timeOfDay,
    const std::set<int>& forbiddenNodes,
    const std::set<std::string>& forbiddenRoadTypes
) {
    return dijkstraTime(source, target, timeOfDay, forbiddenNodes, forbiddenRoadTypes);
}

int Routing::findNearestNode(double lat, double lon) const {
    int nearestNode = -1;
    double minDistance = std::numeric_limits<double>::infinity();

    for (const auto& [nodeId, node] : graph.getAllNodes()) {
        double dist = graph.getEuclideanDistance(lat, lon, node.lat, node.lon);
        if (dist < minDistance) {
            minDistance = dist;
            nearestNode = nodeId;
        }
    }

    return nearestNode;
}

KNNResult Routing::knnEuclidean(
    double lat,
    double lon,
    const std::string& poi,
    int k
) {
    KNNResult result;
    std::vector<std::pair<double, int>> candidates;

    for (const auto& [nodeId, node] : graph.getAllNodes()) {
        if (graph.getNodePoIs(nodeId).find(poi) != graph.getNodePoIs(nodeId).end()) {
            double dist = graph.getEuclideanDistance(lat, lon, node.lat, node.lon);
            candidates.push_back({dist, nodeId});
        }
    }

    std::sort(candidates.begin(), candidates.end());

    for (int i = 0; i < std::min(k, static_cast<int>(candidates.size())); ++i) {
        result.nodes.push_back(candidates[i].second);
    }

    return result;
}

KNNResult Routing::knnShortestPath(
    double lat,
    double lon,
    const std::string& poi,
    int k
) {
    KNNResult result;
    std::vector<std::pair<double, int>> candidates;

    int startNode = findNearestNode(lat, lon);
    if (startNode == -1) {
        return result;
    }

    for (const auto& [nodeId, node] : graph.getAllNodes()) {
        if (graph.getNodePoIs(nodeId).find(poi) != graph.getNodePoIs(nodeId).end()) {
            PathResult pathResult = dijkstraDistance(startNode, nodeId, {}, {});
            if (pathResult.possible) {
                candidates.push_back({pathResult.minimumDistance, nodeId});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end());

    for (int i = 0; i < std::min(k, static_cast<int>(candidates.size())); ++i) {
        result.nodes.push_back(candidates[i].second);
    }

    return result;
}
