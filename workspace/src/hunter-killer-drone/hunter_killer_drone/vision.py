import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


model = YOLO('yolov8m.pt')


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # receive an Image from the camera topic
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10
        )

        # convert between ROS and OpenCV images
        self.cvbridge = CvBridge()


    def listener_callback(self, data):
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.cvbridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Object Detection
        results = model.predict(current_frame, classes=[0]) # 0: people, 2: cars
        image = results[0].plot()

        # Show Results
        cv2.imshow('Detected Frame', image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
