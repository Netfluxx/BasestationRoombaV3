#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ncurses.h>
#include <geometry_msgs/msg/twist.hpp>

class InputNode : public rclcpp::Node {
public:
    InputNode() : Node("input_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("input_topic", 10);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    ~InputNode() {
        endwin(); // Ensure ncurses is properly closed when the node is destroyed
    }

    void captureInput() {
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);

        std::string user_input = "stop";
        geometry_msgs::msg::Twist vel_msg;

        while (rclcpp::ok()) {
            int ch = getch();
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;

            switch (ch) {
                case 'w':
                    user_input = "forward";
                    vel_msg.linear.x = 3.0;
                    break;
                case 'a':
                    user_input = "left";
                    vel_msg.angular.z = 3.0;
                    break;
                case 's':
                    user_input = "backwards";
                    vel_msg.linear.x = -3.0;
                    break;
                case 'd':
                    user_input = "right";
                    vel_msg.angular.z = -3.0;
                    break;
                default:
                    user_input = "stopit";
                    break;
            }

            std_msgs::msg::String msg;
            msg.data = user_input;
            publisher_->publish(msg);
            velocity_publisher_->publish(vel_msg);

            if (user_input != "stop") {
                continue;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InputNode>();
    node->captureInput(); // Call captureInput method
    rclcpp::shutdown();
    return 0;
}
