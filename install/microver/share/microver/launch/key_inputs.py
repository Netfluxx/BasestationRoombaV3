#custom ros2 node that publishes twist messages to the cmd_vel topic to control my robot runing libgazebo_ros_diff_drive

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyInputs(Node):
    def __init__(self):
        super().__init__('key_inputs')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #whilst the w key is pressed the robot will move forward
        if self.getch() == 'w':
            msg = Twist()
            msg.linear.x = 0.5
            self.publisher_.publish(msg)
        #whilst the s key is pressed the robot will move backward
        elif self.getch() == 's':
            msg = Twist()
            msg.linear.x = -0.5
            self.publisher_.publish(msg)
        #whilst the a key is pressed the robot will turn left
        elif self.getch() == 'a':
            msg = Twist()
            msg.angular.z = 0.5
            self.publisher_.publish(msg)
        #whilst the d key is pressed the robot will turn right
        elif self.getch() == 'd':
            msg = Twist()
            msg.angular.z = -0.5
            self.publisher_.publish(msg)
        #if both a and w are pressed the robot will move forward and turn left
        
        #if no key is pressed the robot will stop
        else:
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0
            self.publisher_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)

    key_inputs = KeyInputs()

    rclpy.spin(key_inputs)

    key_inputs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()