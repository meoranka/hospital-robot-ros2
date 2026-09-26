#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

class SpeechInterfaceNode : public rclcpp::Node
{
public:
    SpeechInterfaceNode()
        : Node("speech_interface_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/hospital/target_room",
            10
        );
    }

    void run()
{
    std::string room_name;

    while (rclcpp::ok())
    {
        std::cout << "Enter target room: " << std::flush;//принудительно сразу выводит приглашение в терминал

        if (!std::getline(std::cin, room_name))
        {
            std::cout << std::endl;
            break;
        }

        if (!rclcpp::ok())
        {
            break;
        }

        if (room_name.empty())
        {
            continue;
        }

        auto message = std_msgs::msg::String();
        message.data = room_name;

        publisher_->publish(message);

        RCLCPP_INFO(
            this->get_logger(),
            "Published target room: %s",
            room_name.c_str()
        );

        std::string command =
            "espeak-ng \"Destination set to " + room_name + "\"";

        std::system(command.c_str());
    }
}

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SpeechInterfaceNode>();

    node->run();

    rclcpp::shutdown();

    return 0;
}