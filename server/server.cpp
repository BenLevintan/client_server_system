//Or Avital 207779802
//Omri Buhbut 209379536
//Ben Levintan 318181831

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
#include <mutex>
#include "std_lib_facilities.h"

using namespace std;


// setting global variables for caching searches 
queue<pair<int, int>> lastRequests;
queue<vector<int>> lastResults;
mutex mtx; 

//struct to be able to fit the nessecary arguments into pthread_create
struct Threadinfo {
    int client_sockfd;
    const map<int, vector<int>>& graph;
};

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
    cout << "data loaded, server is ready." << endl;
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

//function that is called each time a new thread is created
void* handleClient(void* arg){
    //recievs info from the threadinfo struct
    Threadinfo* info = static_cast<Threadinfo*>(arg);
    int client_sockfd = info->client_sockfd;
    const map<int, vector<int>>& graph = info->graph;
    delete info;

    char buffer[256];
    memset(buffer, 0, sizeof(buffer));
    int bytes_received = read(client_sockfd, buffer, sizeof(buffer));

    // converts the payload to a string, finds the comma and extracts the source and destination nodes
    string payload(buffer);
    size_t comma_pos = payload.find(',');
    int source = stoi(payload.substr(0, comma_pos));
    int destination = stoi(payload.substr(comma_pos + 1));

    //variables that store the shortest path
    vector<int> shortestPath;
    bool foundInCache = false;
    string output;

    // Lock the mutex
    mtx.lock();     

    // Check if the request is in the cache
    queue<pair<int, int>> tempRequests = lastRequests;
    queue<vector<int>> tempResults = lastResults;
    
    //if the source and end are the same, gives the saved result
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
        //assembles the message and prints it out
        if (shortestPath.empty()) {
            output = "No path found between the two nodes given";
        } else {
            for (int node : shortestPath) {
                output += to_string(node) + " ";
            }
        }
    }

    // Unlock the mutex
    mtx.unlock();

    // Send the message and close the connection with the client
    write(client_sockfd, output.c_str(), output.length());
    close(client_sockfd);
    return NULL;
}


/*
    use the following commands (in server diractory)
    -   g++ server.cpp
    -   ./a.out <filename.csv> <port number> 
*/

int main(int argc, char* argv[]){
    // get the file name and the port
    string filename = argv[1];
    int port = stoi(argv[2]);

    // initialize the graph
    map<int, vector<int>> graph;
    readCSV(filename, graph);


    // Creating a TCP server and a socket to listen for incoming connections
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serv_addr = {0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serv_addr.sin_port = htons(port);
    bind(fd, (sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(fd, 5);

    while (true) {
        // The server waits for a client connection
        sockaddr_in client_addr = {0};
        socklen_t client_len = sizeof(client_addr);

        int client_sockfd = accept(fd, (sockaddr*)&client_addr, &client_len);

        // create a new thread that leads to using the handleclient function
        Threadinfo* info = new Threadinfo{client_sockfd, graph};
        pthread_t thread;
        pthread_create(&thread, NULL, handleClient, info);

        // Detach the thread so it can run independently
        pthread_detach(thread);
    }
    // closes the socket
    close(fd);
    return 0;
}
