//Ben Levintan 
//Omri Buhbut 
//Or Avital 

/*add IDs before submition*/

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <queue>
#include "std_lib_facilities.h"

using namespace std;

// Function to read the CSV file and populate the graph
void readCSV(const string& filename, unordered_map<int, vector<int>>& graph) {
    ifstream file(filename);
    string line;

    while (getline(file, line)) {
        stringstream ss(line);
        int node1, node2;
        ss >> node1 >> node2;

        // Skip edges from a node to itself
        if (node1 != node2) {
            // Add edge from node1 to node2
            graph[node1].push_back(node2);
            // Add edge from node2 to node1 (since it's an undirected graph)
            graph[node2].push_back(node1);
        }
    }
}

// Function to print the graph
void printGraph(const unordered_map<int, vector<int>>& graph) {
    for (const auto& pair : graph) {
        cout << "Node " << pair.first << " -> ";
        for (int neighbor : pair.second) {
            cout << neighbor << " ";
        }
        cout << endl;
    }
}

// Function to write the adjacency list to a file
void writeAdjacencyListToFile(const unordered_map<int, vector<int>>& adjacencyList) {
    ofstream outputFile("adjacency_list2.txt");
    if (!outputFile) {
        cout << "ERROR: Unable to create/open file for writing" << endl;
        return;
    }

    outputFile << "Adjacency List beta:\n";
    for (const auto& pair : adjacencyList) {
        outputFile << "Node " << pair.first << " -> ";
        for (int neighbor : pair.second) {
            outputFile << neighbor << " ";
        }
        outputFile << endl;
    }

    outputFile.close();
    cout << "Adjacency list written to adjacency_list.txt" << endl;
}

// Function to perform BFS and find the shortest path between two nodes
vector<int> bfsShortestPath(const unordered_map<int, vector<int>>& graph, int startNode, int endNode) {
    queue<int> q;
    vector<bool> visited(graph.size(), false);
    vector<int> parent(graph.size(), -1);

    q.push(startNode);
    visited[startNode] = true;

    while (!q.empty()) {
        int currentNode = q.front();
        q.pop();

        if (currentNode == endNode) {
            // Reconstruct the path
            vector<int> path;
            int node = endNode;
            while (node != startNode) {
                path.push_back(node);
                node = parent[node];
            }
            path.push_back(startNode);
            reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor : graph.at(currentNode)) {
            if (!visited[neighbor]) {
                q.push(neighbor);
                visited[neighbor] = true;
                parent[neighbor] = currentNode;
            }
        }
    }

    // If no path found
    return {};
}

int main() {
    string filename = "db.csv";
    unordered_map<int, vector<int>> graph;

    readCSV(filename, graph);
    printGraph(graph);
    writeAdjacencyListToFile(graph);


    int startNode = 0;
    int endNode = 19382;

    vector<int> shortestPath = bfsShortestPath(graph, startNode, endNode);

    if (shortestPath.empty()) {
        cout << "No path found between nodes " << startNode << " and " << endNode << endl;
    } else {
        cout << "Shortest path between nodes " << startNode << " and " << endNode << " is: ";
        for (int node : shortestPath) {
            cout << node << " ";
        }
        cout << endl;
    }

    return 0;
}
