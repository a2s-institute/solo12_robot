#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>

int main(int argc, char** argv)
{
    // Load gazebo
    gazebo::client::setup(argc, argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the example topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<gazebo::msgs::Vector3d>("/gazebo/solo12/example");

    // Wait for a subscriber to connect
    pub->WaitForConnection();

    // Loop and publish messages
    while (true)
    {
        gazebo::common::Time::MSleep(1000);

        // Create a vector3 message
        gazebo::msgs::Vector3d msg;
        gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::rand() % 100 / 20.0, std::rand() % 100 / 20.0, std::rand() % 100 / 20.0));

        pub->Publish(msg);
    }

    // Make sure to shut down
    gazebo::client::shutdown();
    return 0;
}
