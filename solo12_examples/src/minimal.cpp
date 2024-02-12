#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>

class OscillationPublisher : public rclcpp::Node
{
public:
    OscillationPublisher() : Node("oscillation_publisher"), current_joint_index_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OscillationPublisher::publish_next_joint_position, this));
    }

private:
    void publish_next_joint_position()
    {
        std_msgs::msg::Float64MultiArray message;
        message.data.resize(12); // Assuming 12 joints

        double amplitude = M_PI / 12.0; // 15 degrees in radians
        double frequency = 0.1;
        double phase_shift = 0.0;

        for (size_t i = 0; i < 12; ++i)
        {
            double angle = amplitude * std::sin(2.0 * M_PI * frequency * static_cast<double>(i) + phase_shift);
            message.data[i] = angle;
        }

        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t current_joint_index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OscillationPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

