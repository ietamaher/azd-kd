#!/usr/bin/python

import os
import sys

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time
import rclpy
from rclpy.node import Node
from om_msgs.msg import Query
from om_msgs.msg import State
from sensor_msgs.msg import Joy
from rclpy.executors import MultiThreadedExecutor

# Global variables
_state_driver = 0  # Communication flag variable (0: communicable, 1: communicating)
_state_mes = 0  # Message flag variable (0: no message, 1: message received, 2: message error)
_state_error = 0  # Error flag variable (0: no error, 1: no response, 2: exception response)
msg = Query()

# Constants
_QUEUE_SIZE = 1
_MESSAGE_ERROR = 2
_EXCEPTION_RESPONSE = 2


class MySubscription(Node):
    def __init__(self):
        super().__init__("my_sub")
        # Create a subscription to the "azd_kd_state" topic with the specified callback function
        self.sub = self.create_subscription(
            State, "azd_kd_state", self.state_callback, _QUEUE_SIZE
        )

    def state_callback(self, res):
        """Status callback function

        Reflect the subscribed status data in global variables

        """
        global _state_driver
        global _state_mes
        global _state_error
        _state_driver = res.state_driver
        _state_mes = res.state_mes
        _state_error = res.state_error


class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_pub")
        self.seq = 0
        # Create a publisher to the "om_query0" topic
        self.pub = self.create_publisher(Query, "azd_kd_query", _QUEUE_SIZE)
        # Create a timer with a callback function
        #self.timer = self.create_timer(0.03, self.timer_callback)  
        
        self.latest_joy_msg = None
        self.latest_speed = 0
        self.mapped_speed = 0     
        self.direction= 0x4000     
        self.set_speed = 0     
        self.speed = 2000
        # Subscribe to the "/joy" topic
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

    def joy_callback(self, msg):
        """Joystick callback function"""
        # Assuming axis 4 corresponds to speed control in your joystick message
        speed = msg.axes[4]

        # Map joystick range (-1 to 1) to the desired speed range (e.g., 0 to 2000 Hz)
        self.mapped_speed = speed  * 1000  # Adjust the mapping as needed

        if msg.buttons[0] == 1:
            self.speed += 1000  # Increase speed by 100 (adjust as needed)

        # Check the status of button 2 (decrease speed)
        if msg.buttons[1] == 1:
            self.speed -= 1000  # Decrease speed by 100 (adjust as needed)

        self.speed = max(0, min(self.speed, 10000)) 

        # Compare with 0 and set a fixed speed if greater
        if self.mapped_speed > 200:
            self.mapped_speed = 5000
            self.direction= 0x4000
        elif self.mapped_speed < -200:
            self.mapped_speed = 5000
            self.direction= 0x8000
        else:
            self.mapped_speed = 0
        
        # Compare with the previous speed to avoid unnecessary calls
        if self.mapped_speed != self.latest_speed:
            print("change")  
            self.latest_speed = self.mapped_speed
            if self.mapped_speed > 200:
                print("start")  
                self.set_data(self.speed)
                self.start(self.direction)
            else:
                print("stop")  
                self.stop()

            print(self.speed)


    def set_data(self, speed):
        global msg
        # Write operation data
        print('set_data')
        msg.slave_id = 0x02  # Unit selection (Hex): Unit 1
        msg.func_code = 1  # Function code selection: 1 (Write)
        msg.write_addr = 1152  # Select starting address (Dec): Operation data No.0 method (1800h)
        msg.write_num = 1   # Write data size: 6 (6x32bit)
        msg.data[0] = speed  # Speed: 1000 [Hz]
        self.pub.publish(msg)  # Send the above content to the query generation node. After creating msg in the node, send it to the driver
        self.wait()  # Wait for processing

    def start(self, direction):
        global msg
        # Start operation
        msg.slave_id = 0x02  # Unit selection (Hex): Unit 1
        msg.func_code = 1  # Function code selection: 1 (Write)
        msg.write_addr = 124  # Select starting address (Dec): Driver input command
        msg.write_num = 1  # Write data size: 1 (32bit)
        msg.data[0] = direction  # Write data: (0000 0000 0000 1000) = 8 (START signal ON)
        self.pub.publish(msg)  # Send the above content to the query generation node. After creating msg in the node, send it to the driver
        self.wait()  # Wait for processing

    def stop(self):
        """Stop service function

        Turn off the operation input command (perform stop command) service

        """
        global msg
        msg.slave_id = 0x02  # Unit selection (Hex): Unit 1
        msg.func_code = 1  # Function code selection: 1 (Write)
        msg.write_addr = 124  # Select starting address (Dec): Driver input command
        msg.write_num = 1  # Write data size: 1 (32bit)
        msg.data[0] = 32  # Write data: (0000 0000 0010 0000) = 32 (STOP signal ON)
        self.pub.publish(msg)  # Send the above content to the query generation node. After creating the msg in the node, send it to the driver.
        self.wait()  # Wait for processing

        msg.slave_id = 0x02  # Unit selection (Hex): Unit 2
        msg.func_code = 1  # Select function code: 1 (Write)
        msg.write_addr = 124  # Select starting address (Dec): Driver input command
        msg.write_num = 1  # Write data size: 1 (32bit)
        msg.data[0] = 0  # Write data: All bits OFF
        self.pub.publish(msg)  # Send the above content to the query generation node. After creating the msg in the node, send it to the driver.
        self.wait()  # Wait for processing

    def wait(self):
        """Processing wait service function

        After the specified time (30ms), wait until communication is possible.

        """
        time.sleep(0.01)  # Set wait time (1 = 1.00s)
        # Loop until communication is finished
        while _state_driver == 1:
            pass


def main(args=None):
    """Main function

    Processing 1: Write drive data
    Processing 2: Start driving (START signal ON)

    """
    rclpy.init(args=args)
    try:
        pub = MyPublisher()
        sub = MySubscription()
        executor = MultiThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pub.destroy_node()
        sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
