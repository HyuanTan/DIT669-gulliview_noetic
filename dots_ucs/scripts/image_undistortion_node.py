#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import yaml
import apriltag
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf
import tf.transformations
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from stitching import Stitcher
import math

"""
python3 -m pip install apriltag
sudo pip install stitching

"""
class ImageUndistortionNode:
    def __init__(self):
        # Initialize node
        rospy.init_node("image_undistortion_and_apriltag_node", anonymous=True)

        # Get parameters
        self.camera_id = rospy.get_param("~camera_id", 0)
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.camera_info_file = rospy.get_param("~camera_info_file", "camera_params.yaml")
        self.apriltag_info_file = rospy.get_param("~apriltag_info_file", "apriltags.yaml")
        self.output_topic = rospy.get_param("~output_topic", "/camera/image_apriltag")
        self.undistorted_topic = rospy.get_param("~undistorted_topic", "/camera/image_undistorted")
        self.top_view_topic = rospy.get_param("~top_view_topic", "/camera/image_top_view")
        self.odom_topic = rospy.get_param('~odom_topic', '/apriltag/odometry')

        self.target_tag_ids = rospy.get_param("~target_tag_ids", ["0", "1", "2", "3"])  # target ID list
        if isinstance(self.target_tag_ids, str):
            self.target_tag_ids = [int(tag.strip()) for tag in self.target_tag_ids.strip("[]").split(",")]

        # rospy.loginfo(f"Target tag IDs: {self.target_tag_ids}")

        self.get_global_coodinate = False
        self.pub_global_coodinate = False
        self.transform = TransformStamped()
        self.tag_size = 0.15  # 15cm
        self.tag_size, self.tags_data = self.load_tags_from_yaml(self.apriltag_info_file)
        self.counter = 0

        self.last_position = None
        self.last_time = None
        self.last_yaw = None
        self.tf_robot_broadcaster = tf.TransformBroadcaster()
        self.initial = False

        self.main_tag = self.target_tag_ids[0]
        self.homography_matrix = []
        

        # Initialize AprilTag detector
        options = apriltag.DetectorOptions(families="tag25h9")
        self.apriltag_detector = apriltag.Detector(options)

        robot_options = apriltag.DetectorOptions(families="tag36h11")
        self.apriltag_robot_detector = apriltag.Detector(robot_options)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load camera parameters
        self.camera_matrix, self.dist_coeffs = self.load_camera_info(self.camera_info_file)
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logerr("Failed to load camera parameters. Node will shut down.")
            rospy.signal_shutdown("Invalid camera parameters")
            return

        # Initialize image publishers and subscribers
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=5)
        self.visualized_pub = rospy.Publisher(self.output_topic, Image, queue_size=10)
        self.undistorted_pub = rospy.Publisher(self.undistorted_topic, Image, queue_size=10)
        self.top_view_pub = rospy.Publisher(self.top_view_topic, Image, queue_size=10)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)

        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo("Node initialized and running.")

    def load_camera_info(self, yaml_file):
        """
        Load tag size and object points from a YAML file.
        
        :param yaml_file: YAML file path.
        :return: A dictionary containing tag_id and corresponding size and object_points.
        """
        try:
            with open(yaml_file, "r") as file:
                camera_info = yaml.safe_load(file)
                camera_matrix = np.array(camera_info["camera_matrix"]["data"]).reshape(3, 3)
                dist_coeffs = np.array(camera_info["distortion_coefficients"]["data"])
                rospy.loginfo(f"Successfully loaded camera parameters from file:{yaml_file}.")
                return camera_matrix, dist_coeffs
        except Exception as e:
            rospy.logerr(f"Error reading camera info file: {e}")
            return None, None

    def load_tags_from_yaml(self, yaml_file):
        """
        Read tag_size and object_points from a YAML file and store them in a dictionary.

        :param yaml_file: Path to the YAML file
        :return: A dictionary containing tag_id, corresponding size, and object_points
        """
        rospy.loginfo(f"Load tag file from {yaml_file}")
        try:
            with open(yaml_file, "r") as file:
                data = yaml.safe_load(file)
            tags_data = {}
            tag_size = data["tag_size"]
            for tag in data.get("tags", []):
                tag_id = tag["id"]
                center = tag["center"]
 
                half_size = self.tag_size / 2
                object_points = np.array([
                    [center[0]-half_size, center[1]-half_size, 0],  # Top-left corner
                    [center[0]+half_size, center[1]-half_size, 0],  # Top-right corner
                    [center[0]+half_size, center[1]+half_size, 0],  # Bottom-right corner
                    [center[0]-half_size, center[1]+half_size, 0],  # Bottom-left corner
                ], dtype=np.float32)

                # tags_data[tag_id] = {
                #     "position": tag["position"]
                # }
                tags_data[tag_id] = {"position": object_points, "center": center}

            # rospy.loginfo(f"tags_data: {tags_data}")

            return tag_size, tags_data
        except Exception as e:
            rospy.logerr(f"Error loading YAML file: {e}")
            return None, None

    def pixel_to_world(self, u, v, Z, camera_matrix, dist_coeffs, rvec, tvec):
        # Step 1: Construct the extrinsic matrix from rvec and tvec
        R, _ = cv2.Rodrigues(rvec)  # Convert rotation vector to rotation matrix
        RT = np.hstack((R, tvec))  # Combine [R|T]

        # Step 2: Compute the inverse of [R|T]
        RT_inv = np.linalg.inv(np.vstack((RT, [0, 0, 0, 1])))[:3, :]

        # Step 3: Transform pixel coordinates to camera coordinates
        K_inv = np.linalg.inv(camera_matrix)  # Compute the inverse of the intrinsic parameters
        pixel_homogeneous = np.array([u, v, 1])  # Create homogeneous coordinates for the pixel
        camera_coords = np.dot(K_inv, pixel_homogeneous)  # Transform to camera coordinate system

        # Step 4: Determine the scaling factor
        scale = Z / camera_coords[2]
        camera_coords *= scale

        # Step 5: Transform to world coordinates
        world_coords = np.dot(RT_inv, np.append(camera_coords, 1))

        return world_coords[:3]

    def image_callback(self, msg):
        try:
            # ROS image msg to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            h, w = cv_image.shape[:2]

            # Correct the image
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
            map1, map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)
            undistorted_image = cv2.remap(cv_image, map1, map2, cv2.INTER_LINEAR)

            if False:
                output_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_undistore.png"
                cv2.imwrite(output_path, undistorted_image)
                self.counter = self.counter + 1

            undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding="bgr8")
            undistorted_msg.header.stamp = rospy.Time.now()
            self.undistorted_pub.publish(undistorted_msg)

            # if not self.get_global_coodinate:
            if True:
                # Convert to grayscale for AprilTag detection
                gray_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
                detections = self.apriltag_detector.detect(gray_image)
                

                # Visualize detected tags
                visualized_image = undistorted_image.copy()
                
                rospy.loginfo(f"Totally get {len(detections)} detections.......")
                
                image_points_all = []
                world_points = []
                tag_centers = []
                world_tag_centers = []
                for detection in detections:
                    if detection.tag_id in self.target_tag_ids:  # Process only specified tag_id
                    # if True:
                        rospy.loginfo(f"Tag ID: {detection.tag_id}")
                        # rospy.loginfo(f"Tag Family: {detection.tag_family}")
                        # rospy.loginfo(f"Tag Center: {detection.center}")
                        # rospy.loginfo(f"Tag Corners: {detection.corners}")
                        # rospy.loginfo(f"Homography Matrix: {detection.homography}")

                        corners = detection.corners
                            
                        
                        corners_int = [(int(c[0]), int(c[1])) for c in corners]
                        
                        for i in range(4):
                            # rospy.loginfo(f"corners[{i}]: {corners_int[i]}")
                            cv2.line(visualized_image, tuple(corners_int[i]), tuple(corners_int[(i + 1) % 4]), (0, 255, 0), 2)

                        # Draw center point
                        center = (int(detection.center[0]), int(detection.center[1]))
                        rospy.loginfo(f"center: {center}")
                        tag_centers.append(detection.center)
                        cv2.circle(visualized_image, center, 5, (0, 0, 255), -1)
                        # rospy.loginfo(f"center: {center}")

                        # Label ID
                        cv2.putText(visualized_image, f"ID: {detection.tag_id}", (center[0] + 10, center[1] + 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        # rospy.loginfo(f"putText")

                        image_points = np.array([
                            [corners[0][0], corners[0][1]],  # Top-left corner
                            [corners[1][0], corners[1][1]],  # Top-right
                            [corners[2][0], corners[2][1]],  # Bottom-right
                            [corners[3][0], corners[3][1]],  # Bottom-left
                        ], dtype=np.float32)
                        image_points_all.extend(corners)

                        if detection.tag_id in self.tags_data:
                            object_points = np.array(self.tags_data[detection.tag_id]["position"], dtype=np.float32)
                            world_points.extend(object_points.tolist())
                            world_tag_centers.append(self.tags_data[detection.tag_id]["center"])
                        else:
                            rospy.logwarn(f"Tag ID {detection.tag_id} not found in configuration.")
                            continue
                        
                        # Use solvePnP to calculate pose
                        # success, self.rvec, self.tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
                        success, self.rvec, self.tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

                        if success:
                            u, v = 114, 83
                            Z = 0.0
                            world_coords = self.pixel_to_world(u, v, Z, self.camera_matrix, self.dist_coeffs, self.rvec, self.tvec)
                            print("world_coords:", world_coords)

                        '''
                        if success:
                        # if False:
                            self.get_global_coodinate = True
                            self.pub_global_coodinate = True
                            # rvec is a rotation vector, which can be converted to a rotation matrix using cv2.Rodrigues
                            rotation_matrix, _ = cv2.Rodrigues(rvec)
                            # print("Translation Vector:", tvec)
                            # print("Rotation Matrix:", rotation_matrix)

                            # Publish TF
                            self.transform.header.stamp = rospy.Time.now()
                            self.transform.header.frame_id = f"camera_{self.camera_id}"
                            self.transform.child_frame_id = f"tag_{detection.tag_id}"
                            self.transform.transform.translation.x = tvec[0]
                            self.transform.transform.translation.y = tvec[1]
                            self.transform.transform.translation.z = tvec[2]

                            # Convert rotation matrix to quaternion
                            quaternion = tf.transformations.quaternion_from_matrix(
                                np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1]))
                            )
                            self.transform.transform.rotation.x = quaternion[0]
                            self.transform.transform.rotation.y = quaternion[1]
                            self.transform.transform.rotation.z = quaternion[2]
                            self.transform.transform.rotation.w = quaternion[3]

                            self.tf_broadcaster.sendTransform(self.transform)
                        '''
                    else:
                        rospy.logwarn(f"Tag ID: {detection.tag_id} not in list")

                
                if not self.get_global_coodinate:
                    tag_centers = np.array(tag_centers, dtype=np.float32)
                    tag_centers_2d = np.array(tag_centers[:, :2], dtype=np.float32)
                    world_tag_centers = np.array(world_tag_centers, dtype=np.float32)
                    world_tag_centers_2d = np.array(world_tag_centers[:, :2], dtype=np.float32)
                    
                    x_coords = world_tag_centers_2d[:, 0]
                    y_coords = world_tag_centers_2d[:, 1]
                    self.resolution=0.001
                    
                    # Calculate x and y range
                    x_min, x_max = x_coords.min(), x_coords.max()
                    y_min, y_max = y_coords.min(), y_coords.max()

                    target_width = int(x_max / self.resolution)
                    target_height = int(y_max / self.resolution)
                    self.target_size = (target_width, target_height)

                    world_tag_centers_2d = world_tag_centers_2d / self.resolution
                    self.homography_matrix = cv2.getPerspectiveTransform(tag_centers_2d, world_tag_centers_2d)
                    # np.savetxt("/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + "homography_matrix.txt", self.homography_matrix)
                    # print(f"homography_matrix:{homography_matrix}")
                    self.get_global_coodinate = True
                

                # Step 5: Perform perspective transformation
                
                top_view_image = undistorted_image.copy()
                print(f"target_size:{self.target_size}")
                
                # top_view_image = cv2.warpPerspective(top_view_image, M, self.target_size)
                bird_view_image = cv2.warpPerspective(top_view_image, self.homography_matrix, self.target_size)

                detections_robot = self.apriltag_robot_detector.detect(gray_image)
                for tag in detections_robot:
                    center = tag.center
                    rospy.loginfo(f"Robot AprilTag ID: {tag.tag_id}")
                    if tag.tag_id != 5: continue
                    tag_centers_2d = np.array([center], dtype=np.float32).reshape(-1, 1, 2)
                    transformed_points = cv2.perspectiveTransform(tag_centers_2d, self.homography_matrix)
                    transformed_points = transformed_points / self.resolution
                    for pt in transformed_points: # Todo: for multiple robot
                        odom = Odometry()
                        current_position = (pt[0][0], pt[0][1])
                        current_time = rospy.Time.now()
                        rospy.loginfo(f"Robot current_position: {current_position}")
                        if self.last_position is not None:
                            # position change
                            delta_x = current_position[0] - self.last_position[0]
                            delta_y = current_position[1] - self.last_position[1]
                            delta_t = (current_time - self.last_time).to_sec()
                            distance = math.sqrt(delta_x**2 + delta_y**2)

                            # orientation
                            yaw = math.atan2(delta_y, delta_x)
                            if self.last_yaw is not None:
                                delta_yaw = yaw - self.last_yaw
                                self.initial = True
                            else:
                                delta_yaw = 0

                            # calculate velocity 
                            if delta_t > 0:
                                linear_velocity_x = distance / delta_t
                                angular_velocity_z = delta_yaw / delta_t
                            else:
                                linear_velocity_x = 0
                                angular_velocity_z = 0
                        else:
                            # Initialize the first frame data
                            yaw = 0
                            linear_velocity_x = 0
                            angular_velocity_z = 0

                        self.last_position = current_position
                        self.last_time = current_time
                        self.last_yaw = yaw

                        odom.header.stamp = current_time
                        frame_id = "map"
                        child_frame_id = "robot5"
                        odom.header.frame_id = frame_id
                        odom.child_frame_id = child_frame_id

                        odom.pose.pose.position.x = current_position[0]
                        odom.pose.pose.position.y = current_position[1]
                        odom.pose.pose.position.z = 0

                        orientation_quat = self.yaw_to_quaternion(yaw)
                        odom.pose.pose.orientation.x = orientation_quat[0]
                        odom.pose.pose.orientation.y = orientation_quat[1]
                        odom.pose.pose.orientation.z = orientation_quat[2]
                        odom.pose.pose.orientation.w = orientation_quat[3]

                        odom.twist.twist.linear.x = linear_velocity_x
                        odom.twist.twist.linear.y = 0
                        odom.twist.twist.linear.z = 0
                        odom.twist.twist.angular.x = 0
                        odom.twist.twist.angular.y = 0
                        odom.twist.twist.angular.z = angular_velocity_z

                        if self.initial:
                            self.odom_pub.publish(odom)
                            self.tf_robot_broadcaster.sendTransform(
                                (current_position[0], current_position[1], 0),
                                orientation_quat,
                                current_time,
                                child_frame_id,
                                frame_id
                            )


                if False:
                    '''
                    # tag_centers_2d = np.array([[183.41223, 111.15411], [100, 100]], dtype=np.float32)
                    tag_centers_2d = np.array(tag_centers_2d, dtype=np.float32).reshape(-1, 1, 2)
                    transformed_points = cv2.perspectiveTransform(tag_centers_2d, homography_matrix)
                    transformed_points = transformed_points / resolution
                    print("Original Points:")
                    for pt in tag_centers_2d:
                        print(pt[0])  # Original points (x, y)
                    print("\nTransformed Points:")
                    for pt in transformed_points:
                        print(pt[0])  # Transformed points (x', y')
                    for pt in tag_centers_2d:
                        x, y = int(pt[0][0]), int(pt[0][1])
                        cv2.circle(top_view_image, (x, y), 5, (0, 255, 0), -1)  # Green dots for original points
                    # Draw transformed points
                    for pt in transformed_points:
                        x, y = int(pt[0][0]), int(pt[0][1])
                        cv2.circle(top_view_image, (x, y), 5, (0, 0, 255), -1)  # Red dots for transformed points
                    '''
                    output_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_ori.png"
                    output_path2 = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_top_view.png"
                    cv2.imwrite(output_path, bird_view_image)
                    # cv2.imwrite(output_path2, top_view_image)
                    self.counter = self.counter + 1

                bird_view_image_msg = self.bridge.cv2_to_imgmsg(bird_view_image, encoding="bgr8")
                self.top_view_pub.publish(bird_view_image_msg)

                # Publish visualized images
                visualized_msg = self.bridge.cv2_to_imgmsg(visualized_image, encoding="bgr8")
                self.visualized_pub.publish(visualized_msg)

            # if self.pub_global_coodinate:
            if False:
                self.transform.header.stamp = rospy.Time.now()
                self.tf_broadcaster.sendTransform(self.transform)


        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        # except Exception as e:
        #     rospy.logerr(f"Error during image processing: {e}")

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle (radians) to quaternion"""
        qx = 0
        qy = 0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (qx, qy, qz, qw)

if __name__ == "__main__":
    try:
        node = ImageUndistortionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ImageUndistortionNode stopped.")
