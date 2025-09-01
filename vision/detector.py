import rclpy
import numpy as np
import dt_apriltags as dt
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from apriltags_msgs.msg import AprilTags, AprilTagPose

class Detector(Node):

    bridge = None

    detector = None

    # Intrinsic distortion of the camera used for detection.
    # Follow the instruction in README.md to run the calibration.py
    # script for generating these values.
    camera_params = [4131.063060583949,
                     3770.599043783171,
                     1554.3663114131302,
                     1594.2801245225467
                     ]

    # The true dimensions of the physical AprilTags used for detection.
    # If there are multiple sized tags, create a new detector for each.
    tag_size_meters = 0.2

    family = 'tag36h11'

    def __init__(self):
        super().__init__('detector')
        self.bridge = CvBridge()
        self.detector = dt.Detector(families=self.family, nthreads=8, debug=0)
        self.publisher_ = self.create_publisher(AprilTags, self.family, 10)
        self.subscription = self.create_subscription(
                Image,
                'camera_frames',
                self.check_frame,
                10)

    def check_frame(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        detections = self.detector.detect(cv_img,
                                          estimate_tag_pose=True,
                                          camera_params=self.camera_params,
                                          tag_size=self.tag_size_meters)

        if not detections:
            return

        tags_msg = AprilTags()

        for detection in detections:
            tag = AprilTagPose()

            tag.family.data = detection.tag_family.decode('utf-8')
            tag.tag_id = detection.tag_id
            tag.decision_margin = detection.decision_margin
            tag.pose_err = detection.pose_err
            tag.homography = np.array(detection.homography).flatten().tolist()
            tag.corners = np.array(detection.center).flatten().tolist()
            tag.pose_r = np.array(detection.pose_R).flatten().tolist()
            tag.pose_t = np.array(detection.pose_t).flatten().tolist()
            tag.center = detection.center

            tags_msg.detections.append(tag)

        self.publisher_.publish(tags_msg)


def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)
    detector.destroy_node()


if __name__ == '__main__':
    main()

