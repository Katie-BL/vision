import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Camera(Node):

    device = None
    bridge = None

    def __init__(self):
        super().__init__('camera')
        self.bridge = CvBridge()
        self.device = cv2.VideoCapture(0)
        self.publisher_ = self.create_publisher(Image, 'camera_frames', 10)

    def capture_frame(self):
        ret, frame = self.device.read()
        if ret:
            grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            msg = self.bridge.cv2_to_imgmsg(grayscale, encoding="passthrough")
            self.publisher_.publish(msg)
        else:
            print("Couldn't read camera frame!")



def main(args=None):
    rclpy.init(args=args)
    cam = Camera()
    
    try:
        while rclpy.ok():
            cam.capture_frame()

    except KeyboardInterrupt:
        # Catch Ctrl-C
        pass

    finally:
        print('Shutting down camera')
        cam.device.release()
        cv2.destroyAllWindows()

    cam.destroy_node()


if __name__ == '__main__':
    main()
