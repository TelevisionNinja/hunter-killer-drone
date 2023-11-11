import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math
import geometry_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

model = YOLO('yolov8m.pt')


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # receive an Image from the camera topic
        self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.offboard_vision_publisher = self.create_publisher(
            geometry_msgs.msg.Twist,
            '/offboard_vision_cmd',
            qos_profile
        )

        # convert between ROS and OpenCV images
        self.cvbridge = CvBridge()


    def listener_callback(self, data):
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')

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
            if len(results[0].boxes) > 0:
                # computer graphics coordinate grid (origin at the top left, positive y axis goes down)
                img_width_y, img_height_x = results[0].orig_shape
                bounding_box = results[0].boxes[0].xyxy[0]
                # tracking_id = results[0].boxes[0].id
                x1 = bounding_box[0]
                y1 = bounding_box[1]
                x2 = bounding_box[2]
                y2 = bounding_box[3]

                box_midpoint_x = (x2 + x1) / 2
                box_midpoint_y = (y2 + y1) / 2

                midpoint_offset_x = 0
                midpoint_offset_y = img_width_y / 8

                img_midpoint_x = img_height_x / 2 + midpoint_offset_x
                img_midpoint_y = img_width_y / 2 + midpoint_offset_y

                length_x = img_midpoint_x - box_midpoint_x # pixels
                length_y = img_midpoint_y - box_midpoint_y # pixels

                depth = 20 # meters

                theta_x = math.atan(length_x / depth)
                theta_y = math.atan(length_y / depth)

                twist.angular.x = theta_y
                twist.angular.z = theta_x

            image = results[0].plot()

            # Show Results
            cv2.imshow('Detected Frame', image)
            cv2.waitKey(1)

        self.offboard_vision_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
