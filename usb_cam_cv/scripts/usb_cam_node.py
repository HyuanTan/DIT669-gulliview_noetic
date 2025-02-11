#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml

class CameraPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('camera_publisher', anonymous=True)

        # Retrieve configuration parameters from the parameter server
        self.topic_name = rospy.get_param('~topic_name', '/camera/image_raw')
        self.camera_info_topic_name = rospy.get_param('~camera_info_topic_name', '/camera/camera_info')
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.camera_info_path = rospy.get_param('~camera_info_path', 'config/camera0_params.yaml')

        # Publish image topic
        self.image_pub = rospy.Publisher(self.topic_name, Image, queue_size=2)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic_name, CameraInfo, queue_size=2)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(self.fps)

        self.camera_info_msg = CameraInfo()
        self.camera_id = str(self.camera_id)

        # Open camera device
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            rospy.logerr(f"Failed to open camera at /dev/video{self.camera_index}")
            exit(1)

        rospy.loginfo(f"Camera /dev/video{self.camera_index} opened successfully.")

        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Ensure camera format is MJPG
        if not self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')):
            rospy.loginfo(f"Failed to set MJPG format. Check camera(/dev/video{self.camera_index}) support.")

        rospy.loginfo(f"Initialization(/dev/video{self.camera_index}) complete. Waiting for 1 second...")
        
        self.camera_info_msg = self.load_camera_info(self.camera_info_path)
        rospy.sleep(1)
        
    def load_camera_info(self, file_path):
        """Load camera parameter file and generate CameraInfo message."""
        try:
            with open(file_path, 'r') as f:
                params = yaml.safe_load(f)
            camera_info_msg = CameraInfo()
            camera_info_msg.width = params['image_width']
            camera_info_msg.height = params['image_height']
            camera_info_msg.K = params['camera_matrix']['data']
            camera_info_msg.D = params['distortion_coefficients']['data']
            camera_info_msg.R = params['rectification_matrix']['data']
            camera_info_msg.P = params['projection_matrix']['data']
            camera_info_msg.distortion_model = "plumb_bob"
            rospy.loginfo("Camera info loaded successfully.")
            return camera_info_msg
        except Exception as e:
            rospy.logerr(f"Failed to load camera info from {file_path}: {e}")
            exit(1)

    def publish_images(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()

            if not ret or frame is None:
                rospy.loginfo(f"Failed to read frame or empty frame received(/dev/video{self.camera_index}).")
                continue

            try:
                # Convert image from BGR to RGB format
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Convert image to ROS message
                ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = self.camera_id
                self.image_pub.publish(ros_image)

                self.camera_info_msg.header.stamp = rospy.Time.now()
                self.camera_info_msg.header.frame_id = self.camera_id
                self.camera_info_pub.publish(self.camera_info_msg)

                self.rate.sleep()
            except Exception as e:
                rospy.loginfo(f"Error while publishing image(/dev/video{self.camera_index}): {e}")

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo(f"----Camera(/dev/video{self.camera_index}) resources released.")


if __name__ == '__main__':
    camera_publisher = None
    try:
        camera_publisher = CameraPublisher()
        
        camera_publisher.publish_images()
    except rospy.ROSInterruptException:
        pass
    finally:
        if camera_publisher:
            camera_publisher.cleanup()
