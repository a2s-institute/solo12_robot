// Header for the WebServer class, which might interface with the WebSocket server directly from C++

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <string>
#include "gazebo_interface.h"

class WebServer {
public:
    WebServer(GazeboInterface& gazeboInterface);
    void init(int port);
    void run();
    void onMessageReceived(const std::string& message);

private:
    int port;
    GazeboInterface& gazeboInterface;
    // Add WebSocket server members here
    
};

#endif
