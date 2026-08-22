import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        # Publish to the exact topic your detector is listening to
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # The path to your test video
        # Instead of video file path, use 0 for default laptop webcam:
        self.cap = cv2.VideoCapture(0)
        
        # Publish frames at 30 FPS (every 33ms)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # If the video ends, loop it back to the beginning
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # Convert the OpenCV frame into a ROS 2 Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()