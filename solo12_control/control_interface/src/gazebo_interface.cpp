#include "gazebo_interface.h"
#include <gazebo/gazebo_client.hh>
#include <iostream>

GazeboInterface::GazeboInterface() {
    gazebo::client::setup();
}

GazeboInterface::~GazeboInterface() {
    gazebo::client::shutdown();
}

void GazeboInterface::init(const std::string& commandTopic, const std::string& feedbackTopic) {
    this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->node->Init();
    this->commandPub = this->node->Advertise<gazebo::msgs::GzString>(commandTopic);
    this->commandPub->WaitForConnection();

    this->feedbackSub = this->node->Subscribe(feedbackTopic, &GazeboInterface::feedbackCallbackInternal, this);
}

void GazeboInterface::publishCommand(const std::string& data) {
    // std::cout << "Entered publishCommand" << std::endl;

    // Example parsing assumes the format "JointName#CommandData"
    auto delimiterPos = data.find("#");
    if (delimiterPos == std::string::npos) {
        std::cerr << "Invalid command format: " << data << std::endl;
        return;
    }
    std::string jointName = data.substr(0, delimiterPos);
    std::string commandData = data.substr(delimiterPos + 1);
    // std::cout << "jointName: " << jointName << ", commandData: " << commandData << ", lastCommand: " << lastCommands[jointName] << std::endl << std::flush;


    // Check if the command data has changed since the last command for this joint
    if (lastCommands[jointName] != commandData) {
        gazebo::msgs::GzString msg;
        msg.set_data(data);
        this->commandPub->Publish(msg);
        lastCommands[jointName] = commandData; // Update the last command for this joint
        std::cout << "Publishing: " << data << std::endl;
    } 
    // else {
    //     std::cout << "Command for " << jointName << " unchanged; not republishing." << std::endl;
    // }
}

void GazeboInterface::onFeedbackReceived(std::function<void(const std::string&)> callback) {
    this->feedbackCallback = callback;
}

void GazeboInterface::feedbackCallbackInternal(ConstJointCmdPtr& msg) {
    if (this->feedbackCallback) {
        // Convert the message to a string format if necessary
        // This is a placeholder; you'll need to adapt it based on your message type and desired data format
        std::string feedback = "Received feedback"; // Placeholder
        this->feedbackCallback(feedback);
    }
}
