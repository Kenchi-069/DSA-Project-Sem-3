
#include "graph.h"
using namespace std;

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        cerr << "Usage: " << argv[0] << " <graphFile> <queryFile> <outputFile>\n";
        return 1;
    }

    string graphFile = argv[1];
    string queryFile = argv[2];
    string outputFile = argv[3];

    // Construct Graph with the input file to avoid using a deleted default constructor.
    Graph g;

    // If your Graph implementation does not load the file in its constructor,
    // uncomment the following line instead of passing the filename to the constructor:
    // g.loadGraphFromFile(graphFile);
    g.loadGraph(graphFile);
    g.loadQueries(queryFile);

    return 0;
}
