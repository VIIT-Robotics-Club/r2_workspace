#!/usr/bin/env python3

"""pip install pynput
pip install keyboard
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from time import time
from std_srvs.srv import SetBool
import subprocess
from pynput import keyboard
# import keyboard

class PS4JoyNode(Node):
    def __init__(self):
        super().__init__('ps4_joy_node')
        self.get_logger().info("PS4 Joy Node has been started")

        self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.lift_service = self.create_client(SetBool, 'service_lift')
        self.claw_service = self.create_client(SetBool, 'service_claw')

        # # Register keyboard event handlers
        # keyboard.on_press_key('w', lambda _: self.lift_up())     # 'w' key for lift up
        # keyboard.on_press_key('s', lambda _: self.lift_down())    # 's' key for lift down
        # keyboard.on_press_key('a', lambda _: self.claw_open())    # 'a' key for claw open
        # keyboard.on_press_key('d', lambda _: self.claw_close())   # 'd' key for claw close
        # keyboard.on_press_key('1', lambda _: self.luna_silo_1())  # '1' key for Luna Silo 1
        # keyboard.on_press_key('2', lambda _: self.luna_silo_2())  # '2' key for Luna Silo 2
        # keyboard.on_press_key('3', lambda _: self.luna_silo_3())  # '3' key for Luna Silo 3

        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()
        
    def on_press(self, key):
        try:
            if key.char == 'w':
                self.lift_up()
            elif key.char == 's':
                self.lift_down()
            elif key.char == 'a':
                self.claw_open()
            elif key.char == 'd':
                self.claw_close()
            elif key.char == '1':
                self.luna_silo_1()
            elif key.char == '2':
                self.luna_silo_2()
            elif key.char == '3':
                self.luna_silo_3()
        except AttributeError:
            pass
                
    def call_service(self, service, data):
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetBool.Request()
        request.data = data
        future = service.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service response: %r' % response)
        except Exception as e:
            self.get_logger().error('Exception while calling service: %r' % e)

    def lift_up(self):
        self.get_logger().info("Lift Up")
        self.call_service(self.lift_service, True)
        return
    
    def lift_down(self):
        self.get_logger().info("Lift Down")
        self.call_service(self.lift_service, False)
        return

    def claw_open(self):
        self.get_logger().info("Claw Open")
        self.call_service(self.claw_service, False)
        return

    def claw_close(self):
        self.get_logger().info("Claw Close")
        self.call_service(self.claw_service, True)
        return
    
    def luna_silo_1(self):
        self.get_logger().info("Luna Silo 1")
        cmd = "gnome-terminal -- ros2 run luna_control luna_wall_align --ros-args -p silo_number:=1"
        subprocess.Popen(cmd, shell=True)
        return
    
    def luna_silo_2(self):
        self.get_logger().info("Luna Silo 2")
        cmd = "gnome-terminal -- ros2 run luna_control luna_wall_align --ros-args -p silo_number:=2"
        subprocess.Popen(cmd, shell=True)
        return
    
    def luna_silo_3(self):
        self.get_logger().info("Luna Silo 3")
        cmd = "gnome-terminal -- ros2 run luna_control luna_wall_align --ros-args -p silo_number:=3"
        subprocess.Popen(cmd, shell=True)
        return

    def joy_callback(self, msg):

        
        
        life_up = msg.buttons[3] #Triangle
        life_down = msg.buttons[0] #Cross
        claw_open = msg.buttons[1]  #Circle
        claw_close = msg.buttons[2] #Square
        luna_silo_1 = msg.buttons[11] #Up Arrow
        luna_silo_2 = msg.buttons[14] #Rigt Arrow
        luna_silo_3 = msg.buttons[12] #Down Arrow

        self.get_logger().info(f"Life Up: {life_up}, Life Down: {life_down}, Claw Open: {claw_open}, Claw Close: {claw_close}")


        if life_up == 1:
            self.lift_up()

        if life_down == 1:
            self.lift_down()

        if claw_open == 1:
            self.claw_open()

        if claw_close == 1:
            self.claw_close()
        
        if luna_silo_1 == 1:
            self.luna_silo_1()

        if luna_silo_2 == 1:
            self.luna_silo_2()

        if luna_silo_3 == 1:
            self.luna_silo_3()
def main(args=None):
    rclpy.init(args=args)

    ps4_joy_node = PS4JoyNode()

    rclpy.spin(ps4_joy_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()




