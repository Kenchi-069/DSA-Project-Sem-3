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
Graph::Graph() {}
Node *Graph::getNode(int val)
{
    if (nodeLookup.find(val) != nodeLookup.end())
    {
        return nodeLookup[val];
    }
    return nullptr;
}
void Graph::loadGraph(const string &filename)
{
    ifstream inFile(filename);
    if (!inFile)
    {
        cerr << "Unable to open file " << filename << endl;
        return;
    }
    json j;
    inFile >> j;

    for (const auto &nodeData : j["nodes"])
    {
        int id = nodeData["id"];
        pair<double, double> coordinates = {nodeData["coordinates"][0], nodeData["coordinates"][1]};
        Node *node = new Node(id, coordinates);
        for (const auto &poi : nodeData["pois"])
        {
            node->addPOI(poi);
        }
        nodeLookup[id] = node;
    }

    for (const auto &edgeData : j["edges"])
    {
        int srcId = edgeData["src"];
        int destId = edgeData["dest"];
        addEdge(srcId, destId);
    }
}
void Graph::loadQueries(const string &filename)
{
    ifstream inFile(filename);
    if (!inFile)
    {
        cerr << "Unable to open file " << filename << endl;
        return;
    }
    json j;
    inFile >> j;

    for (const auto &query : j["events"])
    {
    }
}
void Graph::addEdge(int src, int dest)
{
    edges[{src, dest}] = 1; // weight can be modified as needed
}
void Graph::removeEdge(int src, int dest)
{
    edges.erase({src, dest});
}
void Graph::printGraph()
{
    for (const auto &pair : edges)
    {
        cout << "Edge from " << pair.first.first << " to " << pair.first.second << " with weight " << pair.second << endl;
    }
}
