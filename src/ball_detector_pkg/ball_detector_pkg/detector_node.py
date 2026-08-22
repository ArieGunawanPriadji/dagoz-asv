import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from msgs.msg import ObstacleArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
#Waah Duce, I am very confused
#You little gremlin you forgot to make this!
class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        # Load YOLO model and bridge
        model_path = '/home/bertrand/dagoz-asv/src/ball_detector_pkg/weights/best.pt'
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.conf_threshold = 0.4

        # ==========================================================
        # 🎯 TASK 1: ROS 2 Communication Setup
        # ==========================================================
        # 1. Create a Subscription to '/camera/image_raw' with message type Image,
        #    calling self.image_callback, with queue size 10.
        self.sub_image  = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 2. Create a Publisher for ObstacleArray on topic '/asv/obstacles' (queue size 10).
        self.pub_obstacles = self.create_publisher(
            ObstacleArray,
            '/asv/obstacles',
            10
        )
        # 3. Create a Publisher for Image on topic '/asv/camera_debug' (queue size 10).
        self.pub_debug = self.create_publisher(
            Image,
            '/asv/camera_debug',
            10
        )
    


    def map_class(self, yolo_cls_id: int) -> int:
        """
        Maps YOLO output class integer to ObstacleArray constants:
        - Class 0 -> RED_BUOY (1)
        - Class 1 -> GREEN_BUOY (2)
        - Class 2 -> BLUE_DOCKING_BUOY (3)
        """
        # ==========================================================
        # 🎯 TASK 2: Class Mapping
        # ==========================================================
        # Return the appropriate ObstacleArray constant based on yolo_cls_id.
        # If unknown, return ObstacleArray.UNKNOWN (0).
        
        # ---> WRITE YOUR TASK 2 CODE HERE <---
        if yolo_cls_id == 0:
            return ObstacleArray.RED_BUOY #This is for classification!
        elif yolo_cls_id == 1: 
            return ObstacleArray.GREEN_BUOY
        elif yolo_cls_id == 2:
            return ObstacleArray.BLUE_DOCKING_BUOY #Uwah Duce we didn't know this existed! Welp Pepperoni, we have to retrain the model.
        else:
            return ObstacleArray.UNKNOWN


    def image_callback(self, msg: Image):
        # Step A: Convert ROS Image -> OpenCV BGR image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Step B: Run YOLO inference
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)[0]

        # Step C: Prepare ObstacleArray message
        obs_msg = ObstacleArray()
        obs_msg.header = msg.header
        debug_img = cv_image.copy()

        for box in results.boxes:
            # Extract box corners, confidence, and class id
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])

            # ==========================================================
            # 🎯 TASK 3: Bounding Box Geometry & Obstacle Packaging
            # ==========================================================
            # 1. Calculate the center pixel coordinates (center_x, center_y).
            c_x = x1 + (x2-x1)/2 #Duuce why did you do this? To stop integer overflow Pepperoni, this is safer
            c_y = y1 + (y2-y1)/2 #Cmon these values will NOT cause integer overflow!
            # 2. Calculate the radius (use half of the maximum dimension: max(w, h) / 2).
            r = max(x2-x1, y2-y1) / 2
            # 3. Create a geometry_msgs/msg/Point with (x=center_x, y=center_y, z=0.0).
            pt = Point(
                x = c_x,
                y = c_y,
                z = 0.0
            )
            # 4. Append the point, radius, confidence, and mapped class to obs_msg:
            #    - obs_msg.positions
            #    - obs_msg.radii
            #    - obs_msg.confidences
            #    - obs_msg.classes
            obs_msg.positions.append(pt) #Append the point to the position! It shows where the buoy is!
            obs_msg.radii.append(r) #This is the radius! Duce why do we even need this? To see how far it is bakaroni! Mmm bakaroni
            obs_msg.confidences.append(conf) #Pepperoni! ML are dumber than us, so we need to have a clear cutoff of when a buoy is not a buoy!
            obs_msg.classes.append(self.map_class(cls_id)) #It's to classify which buoy is which!

            
            # ---> WRITE YOUR TASK 3 CODE HERE <---



            # Optional debug drawing
            cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        # ==========================================================
        # 🎯 TASK 4: Publish Messages
        # ==========================================================
        # 1. Publish obs_msg to the obstacles publisher.
        self.pub_obstacles.publish(obs_msg)
        # 2. Convert debug_img to a ROS Image msg using self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'),
        #    set debug_msg.header = msg.header, and publish it to the debug publisher.
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
        debug_msg.header = msg.header
        self.pub_debug.publish(debug_msg)
        # ---> WRITE YOUR TASK 4 CODE HERE <---


# ==========================================================
# 🎯 TASK 5: ROS 2 Main Boilerplate
# ==========================================================
# Initialize rclpy, instantiate BallDetectorNode, spin it, and handle cleanup.
def main(args=None):
    # ---> WRITE YOUR TASK 5 CODE HERE <---
    rclpy.init(args=args)
    node = BallDetectorNode()
    try:
        rclpy.spin(node) #<-- This guy runs the nodes! Cool isn't it?
    except KeyboardInterrupt:
        pass #Press Ctrl+C! It stands for cancel, not copy
    finally:
        node.destroy_node() #This cleans up memory
        rclpy.shutdown()

if __name__ == '__main__':
    main()