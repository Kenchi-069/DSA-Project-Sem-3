#include <bits/stdc++.h>
using namespace std;

class Node
{
private:
    int id;
    pair<double, double> coordinates;
    vector<string> pois;

public:
    Node(int id, pair<double, double> coordinates);
    int getId();
    pair<double, double> getCoordinates();
    void addPOI(string poi);
    vector<string> getPOIs();
};
class Edge
{
private:
    Node *src;
    Node *dest;
    int weight;
    double distance;
    double avgTime;
    vector<double> speed;
    bool isDirected;
    string roadType;

public:
    Edge(Node *src, Node *dest, int weight, double distance, double avgTime, bool isDirected, string roadType);
    Node *getSrc();
    Node *getDest();
    int getWeight();
    double getDistance();
    double getAvgTime();
    bool getIsDirected();
    string getRoadType();
    void addSpeed(double spd);
    vector<double> getSpeed();
};
class Graph
{
private:
    unordered_map<int, Node *> nodeLookup;
    unordered_map<pair<Node *, Node *>, int> edges;

public:
    Node *getNode(int val);
    void loadGraphFromFile(const string &filename);
    void addEdge(int src, int dest);
    void removeEdge(int src, int dest);
    void printGraph();
};