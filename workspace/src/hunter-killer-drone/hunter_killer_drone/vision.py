import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math
import geometry_msgs.msg
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy
import time

model = YOLO('yolov8n.pt') # use small models for faster detection bc of hardware limitations


class PID():
    def __init__(
            self,
            tau,
            kp,
            ki,
            kd,
            integrator_max,
            integrator_min,
            pid_max,
            pid_min
        ):
        """
        tau: derivative low pass filter time constant
        """

        self.previous_measurement = 0
        self.previous_error = 0
        self.previous_timestamp = 0
        self.i = 0
        self.d = 0
        self.tau = tau
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.pid_max = pid_max
        self.pid_min = pid_min


    def update(self, error, timestamp, measurement):
        """
        discrete domain
        bilinear transform / tustin's method
        """

        delta_t = timestamp - self.previous_timestamp

        p = self.kp * error
        i = self.i + self.ki * delta_t * (error + self.previous_error) / 2

        if i > self.integrator_max:
            i = self.integrator_max
        elif i < self.integrator_min:
            i = self.integrator_min

        # derivative on measurement
        # band limited differentiator
        d = -(2 * self.kd * (measurement - self.previous_measurement) + (2 * self.tau - delta_t) * self.d) / (2 * self.tau + delta_t)

        pid = p + i + d

        if pid > self.pid_max:
            pid = self.pid_max
        elif pid < self.pid_min:
            pid = self.pid_min

        self.previous_error = error
        self.previous_measurement = measurement
        self.previous_timestamp = timestamp
        self.i = i
        self.d = d

        return pid


    def reset(self):
        self.previous_measurement = 0
        self.previous_error = 0
        self.previous_timestamp = 0
        self.i = 0
        self.d = 0


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # receive an Image from the camera topic
        self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            qos_profile
        )

        self.camera_subscription = self.create_subscription(
            Image,
            'depth_camera',
            self.depth_camera_callback,
            qos_profile
        )

        self.offboard_vision_publisher = self.create_publisher(
            geometry_msgs.msg.Twist,
            '/offboard_vision_cmd',
            qos_profile
        )

        self.info_subscription = self.create_subscription(
            geometry_msgs.msg.Twist,
            '/info',
            self.trajectory_info_callback,
            qos_profile
        )

        self.vision_bool_sub = self.create_subscription(
            Bool,
            '/vision_enabled',
            self.vision_enable_callback,
            qos_profile
        )

        # convert between ROS and OpenCV images
        self.cvbridge = CvBridge()
        self.current_depth_image = None
        self.pitch = 0
        self.roll = 0

        self.yaw_pid = PID(0.1, 3, 0.01, 0.1, 0.01, -0.01, math.pi, -math.pi)
        self.pitch_pid = PID(0.1, 4, 0.05, 0.01, 0.05, -0.05, 15, -15)
        self.isPredicting = False


    def vision_enable_callback(self, msg):
        if msg.data:
            self.yaw_pid.reset()
            self.pitch_pid.reset()


    def trajectory_info_callback(self, msg):
        self.pitch = msg.angular.x # rad
        self.roll = msg.angular.y # rad


    def camera_callback(self, data):
        """
        gazebo settings
        depth camera: oak d lite 
        clip min: 0.1, max: 100
        horizontal fov: 1.204
        fps: 30
        resolution: 1920 X 1080

        sensor: IMX214
        effective_focal_length: 3.37 mm
        pixel_size: 1.12 um
        resolution: 13MP (4208x3120)
        """

        # wait to finish processing current frame
        if self.isPredicting:
            return
        self.isPredicting = True

        timestamp = time.time()

        # Convert ROS Image message to OpenCV image
        current_frame = self.cvbridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # object detection
        # .predict() no id
        # .track() has id
        results = model.predict(current_frame, classes=[0]) # 0: people, 2: cars

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.y = 0.0
        twist.angular.x = 0.0
        twist.angular.z = 0.0

        if len(results) > 0:
            image = results[0].plot()

            if len(results[0].boxes) > 0:
                # camera and image information
                effective_focal_length = 3.37 # mm
                width = 4208 # pixels
                resolution_width = 1920 # pixels
                pixel_size = 1.12 # um
                # width is used bc the height is cropped to get a 16:9 aspect ratio for the output image
                pixel_size = width * pixel_size / (resolution_width * 1000) # um to mm

                #----------------------------------------------------

                # computer graphics coordinate grid (origin at the top left, positive y axis goes down)
                img_height_y, img_width_x = results[0].orig_shape
                bounding_box = results[0].boxes[0].xyxy[0]
                # tracking_id = results[0].boxes[0].id
                x1 = bounding_box[0]
                y1 = bounding_box[1]
                x2 = bounding_box[2]
                y2 = bounding_box[3]

                box_midpoint_x = (x2 + x1) / 2
                box_midpoint_y = (y2 + 3 * y1) / 4 # aim for chest

                midpoint_offset_x = 0
                midpoint_offset_y = img_height_y / 16

                img_midpoint_x = img_width_x / 2 + midpoint_offset_x
                img_midpoint_y = img_height_y / 2 + midpoint_offset_y

                length_x = img_midpoint_x - box_midpoint_x # pixels
                length_y = img_midpoint_y - box_midpoint_y # pixels

                #----------------------------------------------------

                # account for roll
                length_x_adjusted = length_x * math.cos(self.roll) + length_y * math.sin(self.roll) # pixels
                length_y_adjusted = -length_x * math.sin(self.roll) + length_y * math.cos(self.roll) # pixels

                #----------------------------------------------------

                # calculate angles of yaw and pitch
                theta_x = math.atan(length_x_adjusted * pixel_size / effective_focal_length) # rad
                theta_y = math.atan(length_y_adjusted * pixel_size / effective_focal_length) # rad

                #----------------------------------------------------

                # take the lowest depth value out of the mean, median, and midpoint
                point_1_y = int(y1 / img_height_y * len(self.current_depth_image))
                point_1_x = int(x1 / img_width_x * len(self.current_depth_image[0]))
                point_2_y = int(y2 / img_height_y * len(self.current_depth_image))
                point_2_x = int(x2 / img_width_x * len(self.current_depth_image[0]))
                point_3_x = int(box_midpoint_x / img_width_x * len(self.current_depth_image[0]))
                point_3_y = int(box_midpoint_y / img_height_y * len(self.current_depth_image))
                depth_box = self.current_depth_image[point_1_y:point_2_y, point_1_x:point_2_x]
                depth = min(numpy.median(depth_box), numpy.mean(depth_box), self.current_depth_image[point_3_y][point_3_x]) # meters

                #----------------------------------------------------

                # account for non zero pitch
                theta_y_adjusted = self.pitch + theta_y
                delta_height_adjusted = -depth * math.sin(theta_y_adjusted) # meters

                #----------------------------------------------------

                # use PID to control yaw and pitch
                yaw_control = self.yaw_pid.update(theta_x, timestamp, theta_x)
                pitch_control = self.pitch_pid.update(delta_height_adjusted, timestamp, delta_height_adjusted)

                #----------------------------------------------------

                # set yaw and pitch in the twist message
                twist.linear.z = float(pitch_control)
                twist.angular.z = yaw_control

                # the pitch if able to be controlled like in fixed wing:
                # pitch_control = self.pitch_pid.update(theta_y_adjusted, timestamp, theta_y_adjusted)
                # twist.angular.x = pitch_control

                #----------------------------------------------------

                # draw visualization
                thickness = 3

                image = cv2.circle(
                    image,
                    center=(int(box_midpoint_x), int(box_midpoint_y)),
                    radius=thickness,
                    color=(0, 0, 255), # b,g,r
                    thickness=-1
                )

                # vertical line
                image = cv2.line(
                    image,
                    pt1=(int(img_midpoint_x), 0),
                    pt2=(int(img_midpoint_x), int(img_height_y)),
                    color=(0, 255, 0), # b,g,r
                    thickness=thickness
                )

                # horizontal line
                image = cv2.line(
                    image,
                    pt1=(0, int(img_midpoint_y)),
                    pt2=(int(img_width_x), int(img_midpoint_y)),
                    color=(0, 255, 0), # b,g,r
                    thickness=thickness
                )

            # Show Results
            cv2.imshow('Detected Frame', image)
            cv2.waitKey(1)
        else:
            self.yaw_pid.reset()
            self.pitch_pid.reset()

        self.offboard_vision_publisher.publish(twist)
        self.isPredicting = False


    def depth_camera_callback(self, data):
        """
        gazebo settings
        depth camera: oak d lite 
        clip min: 0.2, max: 19.1
        horizontal fov: 1.274
        fps: 30
        resolution: 640 X 480

        sensor: OV7251
        effective_focal_length: 1.3 mm
        pixel_size: 3 um
        resolution: 480P (640x480)
        """

        current_frame = self.cvbridge.imgmsg_to_cv2(data, desired_encoding='passthrough') # 32FC1, 32F: float 32, C1: 1 channel
        depth_array = numpy.array(current_frame, dtype=numpy.float32)

        #----------------------------------------------------

        # replace float infinity with the non infinity maximum value
        depth_array[depth_array == numpy.inf] = -1
        max = depth_array.max()
        depth_array[depth_array == -1] = max

        # replace float -infinity with the non infinity minimum value
        depth_array[depth_array == -numpy.inf] = max + 1
        min = depth_array.min()
        depth_array[depth_array == max + 1] = min

        #----------------------------------------------------

        height = len(depth_array)
        width = len(depth_array[0])

        #----------------------------------------------------

        # resize
        original_horizontal_fov = 1.274
        target_horizontal_fov = 1.204
        scale_factor = original_horizontal_fov / target_horizontal_fov * 0.975 # 97.5% of the ratio. 100% was a little too much
        depth_array = cv2.resize(depth_array, None, fx=scale_factor, fy=scale_factor)

        #----------------------------------------------------

        # change aspect ratio by cutting the height of the image
        aspect_ratio_x = 16
        aspect_ratio_y = 9
        new_height = width * aspect_ratio_y / aspect_ratio_x
        starting_height = (height - new_height) / 2
        depth_array = depth_array[int(starting_height):int(starting_height + new_height), 0:width]

        #----------------------------------------------------

        # update the saved depth image for tracking calculations
        self.current_depth_image = depth_array

        #----------------------------------------------------

        # normalize for visualization
        current_frame = (depth_array - min) * 255 / (max - min) # value range: [0, 255]
        current_frame = numpy.round(current_frame)
        current_frame = current_frame.astype(numpy.uint8)

        # Show Results
        cv2.imshow('Depth Frame', current_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
