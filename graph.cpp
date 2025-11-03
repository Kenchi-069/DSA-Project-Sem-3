#include "graph.h"

Node::Node(int id, pair<double, double> coordinates) : id(id), coordinates(coordinates) {}
int Node::getId()
{
    return id;
}
pair<double, double> Node::getCoordinates()
{
    return coordinates;
}
void Node::addPOI(string poi)
{
    pois.push_back(poi);
}
vector<string> Node::getPOIs()
{
    return pois;
}
Edge::Edge(Node *src, Node *dest, int weight, double distance, double avgTime, bool isDirected, string roadType)
    : src(src), dest(dest), weight(weight), distance(distance), avgTime(avgTime), isDirected(isDirected), roadType(roadType) {}
Node *Edge::getSrc()
{
    return src;
}
Node *Edge::getDest()
{
    return dest;
}
int Edge::getWeight()
{
    return weight;
}
double Edge::getDistance()
{
    return distance;
}
double Edge::getAvgTime()
{
    return avgTime;
}
bool Edge::getIsDirected()
{
    return isDirected;
}
string Edge::getRoadType()
{
    return roadType;
}
void Edge::addSpeed(double spd)
{
    speed.push_back(spd);
}
vector<double> Edge::getSpeed()
{
    return speed;
}
Node *Graph::getNode(int val)
{
    if (nodeLookup.find(val) != nodeLookup.end())
    {
        return nodeLookup[val];
    }
    return nullptr;
}
void Graph::addEdge(int src, int dest)
{
    Node *srcNode = getNode(src);
    Node *destNode = getNode(dest);
    if (srcNode && destNode)
    {
        edges[{srcNode, destNode}] = 1; // Example weight
    }
}
void Graph::removeEdge(int src, int dest)
{
    Node *srcNode = getNode(src);
    Node *destNode = getNode(dest);
    if (srcNode && destNode)
    {
        edges.erase({srcNode, destNode});
    }
}
void Graph::printGraph()
{
    for (const auto &pair : edges)
    {
        Node *src = pair.first.first;
        Node *dest = pair.first.second;
        cout << "Edge from Node " << src->getId() << " to Node " << dest->getId() << " with weight " << pair.second << endl;
    }
}
void Graph::loadGraphFromFile(const string &filename)
{
    return;
}
