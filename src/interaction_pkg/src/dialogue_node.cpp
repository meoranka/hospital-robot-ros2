#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class DialogueNode : public rclcpp::Node
{
public:
    DialogueNode()
        : Node("dialogue_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/detected_patient",
            10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                if (msg->data == 1)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[C++ Logic]: Received signal from Sasha! Starting dialogue");
                }
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DialogueNode>());

    rclcpp::shutdown();

    return 0;
}