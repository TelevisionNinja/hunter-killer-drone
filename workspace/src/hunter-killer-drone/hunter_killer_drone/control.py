#!/usr/bin/env python3


import sys

import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


commands = """
W: Pitch Forward
S: Pitch Backward
A: Yaw Left
D: Yaw Right
Q: Up
E: Down

Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

SPACE: Arm/disarm the drone
R: Enable AI tracking
"""

moveBindings = {
    'w': (0, 0, 0, 0, 1), # Pitch+
    's': (0, 0, 0, 0, -1), # Pitch-
    'a': (0, 0, 0, -1, 0), # Yaw+
    'd': (0, 0, 0, 1, 0), # Yaw-
    '\x1b[A': (0, 1, 0, 0, 0),  # Up Arrow Y+
    '\x1b[B': (0, -1, 0, 0, 0), # Down Arrow Y-
    '\x1b[C': (-1, 0, 0, 0, 0), # Right Arrow X-
    '\x1b[D': (1, 0, 0, 0, 0),  # Left Arrow X+
    'q': (0, 0, 1, 0, 0), # Z+
    'e': (0, 0, -1, 0, 0), # Z-
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        if key == '\x1b': # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2)  # read the next two characters
            key += additional_chars # append these characters to the key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    offboard_publisher = node.create_publisher(
        geometry_msgs.msg.Twist,
        '/offboard_velocity_cmd',
        qos_profile
    )

    arm_toggle = False
    arm_msg_publisher = node.create_publisher(
        std_msgs.msg.Bool,
        '/arm_message',
        qos_profile
    )

    vision_enabled = False
    offboard_vision_publisher = node.create_publisher(
        std_msgs.msg.Bool,
        '/vision_enabled',
        qos_profile
    )

    speed_increment = 1.0
    turn_increment = 0.16
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    pitch_val = 0.0
    yaw_val = 0.0

    try:
        while True:
            print(commands)

            key = getKey(settings)

            x_change = 0.0
            y_change = 0.0
            z_change = 0.0
            yaw_change = 0.0
            pitch_change = 0.0

            if key == '\x03':
                break
            elif key in moveBindings.keys():
                values = moveBindings[key]
                x_change = values[0]
                y_change = values[1]
                z_change = values[2]
                yaw_change = values[3]
                pitch_change = values[4]

                x_val += x_change * speed_increment
                y_val += y_change * speed_increment
                z_val += z_change * speed_increment
                yaw_val += yaw_change * turn_increment
                pitch_val += pitch_change * turn_increment

                twist = geometry_msgs.msg.Twist()
                twist.linear.x = x_val
                twist.linear.y = y_val
                twist.linear.z = z_val
                twist.angular.x = pitch_val
                twist.angular.y = 0.0
                twist.angular.z = yaw_val
                offboard_publisher.publish(twist)

                print("Yaw:", twist.angular.z,
                      "\tPitch:", twist.angular.x,
                      "\tRoll:", twist.angular.y,
                      "\tX Speed (side to side):", twist.linear.x,
                      "\tY Speed (forward & backward):", twist.linear.y,
                      "\tZ Speed (up & down):", twist.linear.z)
            elif key == ' ':
                arm_toggle = not arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_msg_publisher.publish(arm_msg)
                print(f"Arm toggle is now: {arm_toggle}")
            elif key == 'r':
                vision_enabled = not vision_enabled
                vision_msg = std_msgs.msg.Bool()
                vision_msg.data = vision_enabled
                offboard_vision_publisher.publish(vision_msg)
                print(f"AI mode: {vision_enabled}")

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        offboard_publisher.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
