#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <queue>
#include <pthread.h>
#include "std_lib_facilities.h"

using namespace std;

// Function to read the CSV file and populate the graph
void readCSV(const string& filename, map<int, vector<int>>& graph) {
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
void printGraph(const map<int, vector<int>>& graph) {
    for (const auto& pair : graph) {
        cout << "Node " << pair.first << " -> ";
        for (int neighbor : pair.second) {
            cout << neighbor << " ";
        }
        cout << endl;
    }
}


// Function to write the adjacency list to a file
void writeAdjacencyListToFile(const map<int, vector<int>>& adjacencyList) {
    ofstream outputFile("adjacency_list.txt");
    if (!outputFile) {
        cout << "ERROR: Unable to create/open file for writing" << endl;
        return;
    }

    outputFile << "Adjacency List:\n";
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
vector<int> bfsShortestPath(const map<int, vector<int>>& graph, int startNode, int endNode) {
    queue<int> q; // The frontier of nodes being explored
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
        // Adding unvisited nieghbors to the queue
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

/*
    use the following commands (in server diractory)
    -   g++ server.cpp
    -   ./a.out <filename.csv> <port number> 
*/

int main(int argc, char* argv[]){

    string filename = argv[1];
    int port = atoi(argv[2]);

    map<int, vector<int>> graph;
    readCSV(filename, graph);
    printGraph(graph);
    writeAdjacencyListToFile(graph);

    // Creating a TCP server socket to listen for incoming connections
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serv_addr = {0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serv_addr.sin_port = htons(port);
    bind(fd, (sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(fd, 5);

    // Setting up queues for 10 last requests and results
    queue<pair<int, int>> lastRequests;
    queue<vector<int>> lastResults;

    while (true) {
        // The server waits for a client connection
        sockaddr_in client_addr = {0};
        socklen_t client_len = sizeof(client_addr);
        int client_sockfd = accept(fd, (sockaddr*)&client_addr, &client_len);


        // Receiving data from client
        char buffer[256];
        memset(buffer, 0, sizeof(buffer));
        int bytes_received = read(client_sockfd, buffer, sizeof(buffer));


        // Split the message to arguments needed for the search
        string payload(buffer);
        size_t comma_pos = payload.find(',');
        int source = stoi(payload.substr(0, comma_pos));
        int destination = stoi(payload.substr(comma_pos + 1));

        vector<int> shortestPath;
        bool foundInCache = false;
        string output;

        // Check if the request is in the cache
        queue<pair<int, int>> tempRequests = lastRequests;
        queue<vector<int>> tempResults = lastResults;
        while (!tempRequests.empty()) {
            pair<int, int> cachedRequest = tempRequests.front();
            vector<int> cachedResult = tempResults.front();
            tempRequests.pop();
            tempResults.pop();

            if (cachedRequest.first == source && cachedRequest.second == destination) {
                foundInCache = true;
                for (int node : cachedResult) {
                    output += to_string(node) + " ";
                }
                output += "88888 "; // Add 88888 to the output if the result was pulled from the cache
                break;
            }
        }

        // If not found in cache, compute the shortest path
        if (!foundInCache) {
            shortestPath = bfsShortestPath(graph, source, destination);
            if (lastRequests.size() >= 10) {
                lastRequests.pop();
                lastResults.pop();
            }
            lastRequests.push(make_pair(source, destination));
            lastResults.push(shortestPath);

            if (shortestPath.empty()) {
                cout << "No path found between nodes " << source << " and " << destination << endl;
            } else {
                // Building the message for the client
                for (int node : shortestPath) {
                    output += to_string(node) + " ";
                }
            }
        }

        // Send the message and close the connection with the client
        write(client_sockfd, output.c_str(), output.length());
        close(client_sockfd);
    }
    close(fd);
    return 0;
}
