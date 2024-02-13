#ifndef GAZEBO_INTERFACE_H
#define GAZEBO_INTERFACE_H

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <string>
#include <unordered_map>
#include <functional>

class GazeboInterface {
public:
    GazeboInterface();
    ~GazeboInterface();

    void init(const std::string& commandTopic, const std::string& feedbackTopic);
    void publishCommand(const std::string& data);
    void onFeedbackReceived(std::function<void(const std::string&)> callback);

private:
    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr commandPub;
    gazebo::transport::SubscriberPtr feedbackSub;
    std::function<void(const std::string&)> feedbackCallback;
    std::unordered_map<std::string, std::string> lastCommands; // Tracks the last command for each joint

    void feedbackCallbackInternal(ConstJointCmdPtr& msg);
};

#endif // GAZEBO_INTERFACE_H
