#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <atomic>
#include <ncurses.h>

class InputNode : public rclcpp::Node {
public:
    InputNode() : Node("input_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("input_topic", 10);
        running_ = true;
        key_detect['w'] = false;
        key_detect['a'] = false;
        key_detect['s'] = false;
        key_detect['d'] = false;
        key_detect[' '] = false;
        std::thread(&InputNode::captureKey, this, 'w').detach();
        std::thread(&InputNode::captureKey, this, 'a').detach();
        std::thread(&InputNode::captureKey, this, 's').detach();
        std::thread(&InputNode::captureKey, this, 'd').detach();
        std::thread(&InputNode::captureKey, this, ' ').detach();
        std::thread(&InputNode::publishKeys, this).detach();
    }

    ~InputNode() {
        running_ = false;
        endwin();
    }

    void captureKey(char key) {
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);

        while (running_) {
            if (getch() == key) {
                key_detect[key] = true;
            } else {
                key_detect[key] = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        endwin();
    }

    void publishKeys() {
        while (running_) {
            bool diag_top_right = false;
            bool diag_top_left = false;
            bool diag_bottom_right = false;
            bool diag_bottom_left = false;
        
            if (key_detect['w'] && key_detect['d']) {
                diag_top_right = true;
            }else if(key_detect['w'] && key_detect['a']) {
                diag_top_left = true;
            }else if(key_detect['s'] && key_detect['d']) {
                diag_bottom_right = true;
            }else if(key_detect['s'] && key_detect['a']) {
                diag_bottom_left = true;
            }

            std_msgs::msg::String msg;
            if(key_detect['w'] && !(diag_top_right || diag_top_left)) {
                msg.data = "forward";
                publisher_->publish(msg);
            } else if(key_detect['a'] && !(diag_top_left || diag_bottom_left)) {
                msg.data = "left";
                publisher_->publish(msg);
            } else if(key_detect['s'] && !(diag_bottom_right || diag_bottom_left)) {
                msg.data = "backwards";
                publisher_->publish(msg);
            } else if(key_detect['d'] && !(diag_top_right || diag_bottom_right)) {
                msg.data = "right";
                publisher_->publish(msg);
            } else if(key_detect[' ']) {
                msg.data = "stop";
                publisher_->publish(msg);
            }else if(diag_top_right) {
                msg.data = "diag_top_right";
                publisher_->publish(msg);
            }else if(diag_top_left) {
                msg.data = "diag_top_left";
                publisher_->publish(msg);
            }else if(diag_bottom_right) {
                msg.data = "diag_bottom_right";
                publisher_->publish(msg);
            }else if(diag_bottom_left) {
                msg.data = "diag_bottom_left";
                publisher_->publish(msg);
            }else {
                msg.data = "stop";
                publisher_->publish(msg);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::atomic<bool> running_;
    std::map<char, bool> key_detect;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InputNode>();
    std::thread t1(&InputNode::captureKey, node, 'w');
    std::thread t2(&InputNode::captureKey, node, 'a');
    std::thread t3(&InputNode::captureKey, node, 's');
    std::thread t4(&InputNode::captureKey, node, 'd');
    std::thread t5(&InputNode::captureKey, node, ' ');
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    rclcpp::shutdown();
    return 0;
}
