//Ben Levintan 318181831
//Omri Buhbut 209379536
//Or Avital 207779802

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include "std_lib_facilities.h"

using namespace std;


/*
    Use the following commands (in client diractory)
    -   g++ client.cpp
    -   ./a.out <ip> <port number> <source> <destination>
*/

// Following zoom recording of Michael from 19-03
int main(int argc, char *argv[]) {
    string ip = argv[1];
    int port = stoi(argv[2]);
    string source = argv[3];
    string destination = argv[4];

    // Creating a TCP client socket and connecting it to server
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip.c_str());
    addr.sin_port = htons(port);
    connect(fd, (sockaddr*)&addr, sizeof(addr));


    // Send source and destination to the server
    string message = source + "," + destination;
    send(fd, message.c_str(), message.length(), 0);

    // Receive data from a socket
    char buffer[256];
    memset(buffer, 0, sizeof(buffer));
    int bytes_received = recv(fd, buffer, sizeof(buffer), 0);

    string result(buffer, bytes_received);
    cout << result << endl;


    close(fd);
    return 0;
}

