#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// ROS 2 нода для получения сигнала об обнаружении пациента.
class DialogueNode : public rclcpp::Node
{
public:
    DialogueNode()
        : Node("dialogue_node")
    {
        // Подписываемся на топик, в который нода компьютерного зрения
        // отправляет информацию об обнаружении пациента.
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/detected_patient",
            10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                // Значение 1 означает, что пациент был обнаружен.
                if (msg->data == 1)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[C++ Logic]: Received signal from Sasha! Starting dialogue");
                }
            });
    }

private:
    // Храним объект подписки, чтобы она существовала всё время работы ноды.
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    // Инициализация ROS 2.
    rclcpp::init(argc, argv);

    // Создаём ноду и запускаем обработку входящих сообщений.
    rclcpp::spin(std::make_shared<DialogueNode>());

    // Корректно завершаем работу ROS 2 после остановки ноды.
    rclcpp::shutdown();

    return 0;
}