const WebSocket = require('ws');

// Server for frontend communication
const wssFrontend = new WebSocket.Server({ port: 8080 });
console.log('Frontend WebSocket server running on ws://localhost:8080');

wssFrontend.on('connection', function connection(wsFrontend) {
    console.log('Frontend client connected.');
    
    wsFrontend.on('message', function incoming(message) {
        // Convert message from Buffer to string for logging purposes
        const messageStr = message.toString();
        console.log('Message from frontend:', messageStr);
        
        // Forward the original Buffer object directly to the C++ server
        if (wsCppServer.clients.size > 0) {
            wsCppServer.clients.forEach(function each(client) {
                if (client.readyState === WebSocket.OPEN) {
                    client.send(message);
                }
            });
        }
    });
});


// Server for C++ backend communication
const wsCppServer = new WebSocket.Server({ port: 8082 });
console.log('C++ backend WebSocket server running on ws://localhost:8082');

wsCppServer.on('connection', function connection(wsBackend) {
    console.log('C++ client connected.');

    wsBackend.on('message', function incoming(message) {
        console.log('Message from C++:', message);
        // Relay this message back to the frontend, if necessary
        wssFrontend.clients.forEach(function each(client) {
            if (client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    });
});
