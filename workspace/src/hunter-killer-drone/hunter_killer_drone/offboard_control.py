#!/usr/bin/env python

import numpy
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleAttitude, VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


class OffboardControl(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile
        )
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile
        )

        self.vision_bool_sub = self.create_subscription(
            Bool,
            '/vision_enabled',
            self.vision_enable_callback,
            qos_profile
        )

        self.vision_sub = self.create_subscription(
            Twist,
            '/offboard_vision_cmd',
            self.vision_control_callback,
            qos_profile
        )

        # publishers

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        self.publisher_velocity = self.create_publisher(
            Twist,
            '/fmu/in/setpoint_velocity/cmd_vel_unstamped',
            qos_profile
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile
        )

        # callback function for the arm timer
        # period is arbitrary, > ~2Hz
        arm_timer_period = 0.1 # seconds
        self.arm_timer = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # callback function for the command loop
        # period is arbitrary, > ~2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0 # yaw value sent
        self.pitch = 0.0 # pitch value sent
        self.trueYaw = 0.0 # current yaw value of drone
        self.truePitch = 0.0 # current pitch value of drone
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.vision_enabled = False

        self.states = {
            "IDLE": self.state_init,
            "ARMING": self.state_arming,
            "TAKEOFF": self.state_takeoff,
            "LOITER": self.state_loiter,
            "OFFBOARD": self.state_offboard
        }
        self.current_state = "IDLE"

    def arm_message_callback(self, msg):
        self.arm_message = msg.data

    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if self.flightCheck and self.arm_message == True:
                    self.current_state = "ARMING"

            case "ARMING":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10:
                    self.current_state = "TAKEOFF"
                self.arm()

            case "TAKEOFF":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    self.current_state = "LOITER"
                self.arm()
                self.take_off()

            # waits in this state while taking off, and the moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.current_state = "OFFBOARD"
                self.arm()

            case "OFFBOARD":
                if (not self.flightCheck) or self.arm_state == VehicleStatus.ARMING_STATE_DISARMED or self.failsafe == True:
                    self.current_state = "IDLE"
                self.state_offboard()

        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm_message = False

        # self.get_logger().info(self.current_state)
        self.myCnt += 1

    def state_init(self):
        self.myCnt = 0

    def state_arming(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # self.get_logger().info("Arm command send")

    def state_takeoff(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0) # param7 is altitude in meters
        # self.get_logger().info("Takeoff command send")

    def state_loiter(self):
        self.myCnt = 0
        # self.get_logger().info("Loiter Status")

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.offboardMode = True

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # self.get_logger().info("Arm command send")

    # altitude (meters)
    def take_off(self, altitude = 0.0):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=altitude)
        # self.get_logger().info("Takeoff command send")

    # publishes to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7 # altitude value in takeoff command
        msg.command = command # command ID
        msg.target_system = 1 # system which should execute the command
        msg.target_component = 1 # component which should execute the command, 0 for all components
        msg.source_system = 1 # system sending the command
        msg.source_component = 1 # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher.publish(msg)

    # receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):
        # self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        # self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        # self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    # receives Twist commands from Teleop and converts NED -> FLU
    def offboard_velocity_callback(self, msg):
        # X (FLU) is -Y (NED)
        self.velocity.x = -msg.linear.y

        # Y (FLU) is X (NED)
        self.velocity.y = msg.linear.x

        # Z (FLU) is -Z (NED)
        self.velocity.z = -msg.linear.z

        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw = msg.angular.z
        self.pitch = msg.angular.x

    # receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        """
        quaternion to Euler angles
        """

        orientation_q = msg.q

        w = orientation_q[0]
        x = orientation_q[1]
        y = orientation_q[2]
        z = orientation_q[3]

        x_squared = x * x

        self.trueYaw = numpy.arctan2(2 * (z * w + x * y), 1 - 2 * (w * w + x_squared))
        # self.trueRoll = numpy.arctan2(2 * (z * y + w * x), 1 - 2 * (x_squared + y * y))
        self.truePitch = numpy.arcsin(2 * (y * w - z * x))

        # NED -> FLU Transformation
        self.trueYaw = -self.trueYaw


    def vision_enable_callback(self, msg):
        self.vision_enabled = msg.data


    def vision_control_callback(self, msg):
        if self.vision_enabled:
            # NED -> FLU Transformation
            yaw = -msg.angular.z
            # pitch = msg.angular.x
            delta_height = msg.linear.z

            timestamp = int(Clock().now().nanoseconds / 1000)

            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = timestamp
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            offboard_msg.actuator = False

            self.publisher_offboard_mode.publish(offboard_msg)

            # Compute velocity in the world frame
            cos_yaw = numpy.cos(self.trueYaw)
            sin_yaw = numpy.sin(self.trueYaw)
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = timestamp

            # NED local world frame
            trajectory_msg.velocity[0] = velocity_world_x # in meters/second
            trajectory_msg.velocity[1] = velocity_world_y # in meters/second
            trajectory_msg.velocity[2] = self.velocity.z + delta_height # in meters/second
            trajectory_msg.position[0] = float('nan') # in meters
            trajectory_msg.position[1] = float('nan') # in meters
            trajectory_msg.position[2] = float('nan') # in meters
            trajectory_msg.acceleration[0] = float('nan') # in meters/second^2
            trajectory_msg.acceleration[1] = float('nan') # in meters/second^2
            trajectory_msg.acceleration[2] = float('nan') # in meters/second^2

            trajectory_msg.yaw = float('nan') # euler angle of desired attitude in radians -PI..+PI
            trajectory_msg.yawspeed = yaw # angular velocity around NED frame z-axis in radians/second

            self.publisher_trajectory.publish(trajectory_msg)


    # publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if self.offboardMode == True and not self.vision_enabled:
            timestamp = int(Clock().now().nanoseconds / 1000)

            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = timestamp
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            offboard_msg.actuator = False

            self.publisher_offboard_mode.publish(offboard_msg)

            # Compute velocity in the world frame
            cos_yaw = numpy.cos(self.trueYaw)
            sin_yaw = numpy.sin(self.trueYaw)
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = timestamp

            # NED local world frame
            trajectory_msg.velocity[0] = velocity_world_x # in meters/second
            trajectory_msg.velocity[1] = velocity_world_y # in meters/second

            # pitch is [-pi/2, pi/2], thus / by pi/2 to convert to [-1, 1]
            # max velocity = 4.5 m/s
            trajectory_msg.velocity[2] = self.velocity.z + 4.5 * self.pitch * 2 / math.pi # in meters/second

            trajectory_msg.position[0] = float('nan') # in meters
            trajectory_msg.position[1] = float('nan') # in meters
            trajectory_msg.position[2] = float('nan') # in meters
            trajectory_msg.acceleration[0] = float('nan') # in meters/second^2
            trajectory_msg.acceleration[1] = float('nan') # in meters/second^2
            trajectory_msg.acceleration[2] = float('nan') # in meters/second^2

            trajectory_msg.yaw = float('nan') # euler angle of desired attitude in radians -PI..+PI
            trajectory_msg.yawspeed = self.yaw # angular velocity around NED frame z-axis in radians/second

            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
