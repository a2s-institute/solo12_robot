// #include "webserver.h"
// #include <iostream>
// #include <thread>
// #include <functional>
// #include <nlohmann/json.hpp>

// WebServer::WebServer() : uri("ws://localhost:8082") {
//     ws_client.init_asio();
//     ws_client.set_message_handler(bind(&WebServer::on_message, this, ::_1, ::_2));
// }

// void WebServer::connect() {
//     websocketpp::lib::error_code ec;
//     auto conn = wsClient.get_connection(uri, ec);
//     if (ec) {
//         std::cout << "Could not create connection because: " << ec.message() << std::endl;
//         return;
//     }
//     wsClient.connect(conn);
//     std::thread([this]() { wsClient.run(); }).detach();
// }

// void WebServer::handleMessage(const std::string& payload) {
//     std::cout << "Received message from WebSocket: " << payload << std::endl;
//     // Parse and format the message here
//     std::string command = parseAndFormatCommand(payload);
//     std::cout << "Formatted command: " << command << std::endl;
//     gazeboInterface.publishCommand(command);
// }

// void WebServer::on_message(websocketpp::connection_hdl, message_ptr msg) {
//     if (message_handler) {
//         // Parse the JSON message
//         try {
//             auto j = json::parse(msg->get_payload());
//             // Format the message
//             std::string formatted_msg = j["joint"].get<std::string>() + "#" +
//                                         std::to_string(j["position"].get<double>()) + "#" +
//                                         std::to_string(j["P"].get<int>()) + "#" +
//                                         std::to_string(j["I"].get<int>()) + "#" +
//                                         std::to_string(j["D"].get<int>()) + "#";

//             // Call the message handler with the formatted message
//             message_handler(formatted_msg);
//         } catch (json::parse_error& e) {
//             std::cerr << "Parsing error: " << e.what() << '\n';
//         } catch (json::type_error& e) {
//             std::cerr << "Type error: " << e.what() << '\n';
//         }
//     }
// }
