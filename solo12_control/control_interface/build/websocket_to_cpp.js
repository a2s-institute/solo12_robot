const WebSocket = require('ws');
const { spawn } = require('child_process');

// Path to the compiled C++ executable
const cppExecutablePath = './solo12_control_interface';

// Spawn the C++ application
const cppProcess = spawn(cppExecutablePath, {
    stdio: ['pipe', 'inherit', 'inherit'] // This ensures stdin is a pipe you can write to
});

cppProcess.on('close', (code) => {
    console.log(`C++ process exited with code ${code}`);
});

// Setup WebSocket server to listen for connections from the webpage
const wss = new WebSocket.Server({ port: 8080 });
console.log('Frontend WebSocket server running on ws://localhost:8080');

wss.on('connection', function connection(ws) {
    console.log('Frontend client connected.');
    
    ws.on('message', function incoming(message) {
        const messageStr = message.toString();
        console.log('Message from frontend:', messageStr);
        
        // Send the message to the C++ application
        cppProcess.stdin.write(messageStr + "\n");
    });

    ws.on('close', () => {
        console.log('Frontend client disconnected.');
    });
});
