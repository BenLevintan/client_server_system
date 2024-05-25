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
//a function that is used each time a new thread is createdw

void* handleClient(void* arg){
    Threadinfo* info = static_cast<Threadinfo*>(info);
    int client_sockfd = info->client_sockfd;
    const map<int, vector<int>>& graph = info->graph;
    delete info;
    char buffer[256];
    memset(buffer,0,sizeof(buffer));
    int bytes_recieved=read(client_sockfd,buffer,sizeof(buffer));
    if(bytes_recieved<=0){
        cerr <<"error while recieving message"<<endl;
        close(client_sockfd);
        return nullptr;
    }
    string payload(buffer);
    size_t commaposition=payload.find(',');
    int source=stoi(payload.substr(0,commaposition));
    int destination=stoi(payload.substr(commaposition+1));
    vector<int> bfspath=bfsShortestPath(graph,source,destination);
    string output;
    if(bfspath.empty()){
        output="No path found between the two nodes given";
    }
    else{
        for(int node:bfspath){
            output+=to_string(node)+" ";
        }
    }
    write(client_sockfd, output.c_str(), output.length());
    close(client_sockfd);
    return nullptr;
}


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
        if (client_sockfd < 0) {
            cerr << "Error accepting client connection" << endl;
            continue;
        }

        // creating a new thread for each client that connects
        Threadinfo* info=new Threadinfo{client_sockfd,graph};
        pthread_t thread;
        if (pthread_create(&thread, nullptr, handleClient, info) != 0) {
            cerr << "Error creating thread" << endl;
            close(client_sockfd);
            delete info;
            continue;
        }   
        
        
    }
    close(fd);
    return 0;
}
