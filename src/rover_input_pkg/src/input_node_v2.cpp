#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ncurses.h>

class InputNode : public rclcpp::Node {
public:
    InputNode() : Node("input_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("input_topic", 10);
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

        while (rclcpp::ok()) {
            int ch = getch();

            switch (ch) {
                case 'w':
                    user_input = "forward";
                    break;
                case 'a':
                    user_input = "left";
                    break;
                case 's':
                    user_input = "backwards";
                    break;
                case 'd':
                    user_input = "right";
                    break;
                default:
                    user_input = "stop";
                    break;
            }

            std_msgs::msg::String msg;
            msg.data = user_input;
            publisher_->publish(msg);

            if (user_input != "stop") {
                continue;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InputNode>();
    node->captureInput(); // Call captureInput method
    rclcpp::shutdown();
    return 0;
}
