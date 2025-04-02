#!/usr/bin/env python3
import tf
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped  
from sensor_msgs.msg import Image, CameraInfo
 
class Orange:
    def __init__(self):
        rospy.init_node("debug_orange", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/realsense/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/realsense/depth/image_rect_raw", Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber("/realsense/color/camera_info", CameraInfo, self.camera_info_callback)
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        self.barrel_waypoint_pub = rospy.Publisher("/barrel_waypoint", PoseStamped, queue_size=10)
        self.latest_depth = None  
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.position = None
        self.orientation = None
        self.x_offset = 0.2
        self.y_offset = 0.0
        self.z_offset = 0.21
        self.barrel_pose_x = None
        self.barrel_pose_y = None
        self.barrel_pose_z = None
        self.barrel_pose = PoseStamped()

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        # rospy.loginfo(f"Position: {self.position}, Orientation: {self.orientation}")

    def camera_info_callback(self, msg):
        # print(msg.K[0], msg.K[4], msg.K[2], msg.K[5])
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        if None in [self.fx, self.fy, self.cx, self.cy]:
            rospy.logwarn("Camera parameters not initialized properly")
        else:
            rospy.loginfo_once(f"Camera info received: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.1f}, cy={self.cy:.1f}")

    def transform_from_camera_to_map(self, cx, cy, depth):
        if depth <= 0 or np.isnan(depth):
            rospy.logwarn(f"no depth")
            return
        if self.position is None:
            rospy.logwarn("no pose")
            return 
        x_robot = self.position.x
        y_robot = self.position.y

        # get robot orientation
        quat = self.orientation
        quaternion = (quat.x, quat.y, quat.z, quat.w)

        # convert quaternion to euler angles
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        # rospy.loginfo(f"Yaw angle: {yaw:.3f} rad")
        theta_robot = yaw
        # rospy.loginfo(f"theta_robot: {theta_robot}")
        if theta_robot is None:
            rospy.logwarn("no orientation")
            return
        
        x_offset = self.x_offset
        y_offset = self.y_offset
        z_offset = self.z_offset
        fx = self.fx
        fy = self.fy
        img_center_x = self.cx
        img_center_y = self.cy

        # RDF camera coordinate system
        X_cam = (cx - img_center_x) * depth / fx
        Y_cam = (cy - img_center_y) * depth / fy
        Z_cam = depth
        # rospy.loginfo(f"X_cam: {X_cam}, Y_cam: {Y_cam}, Z_cam: {Z_cam}")

        # FLU camera coordinate system
        X_FLU = Z_cam
        Y_FLU = -X_cam
        Z_FLU = -Y_cam
        # rospy.loginfo(f"X_FLU: {X_FLU}, Y_FLU: {Y_FLU}, Z_FLU: {Z_FLU}")

        # FLU jackal coordinate system
        X_base = X_FLU + x_offset
        Y_base = Y_FLU + y_offset
        # rospy.loginfo(f"X_base: {X_base}, Y_base: {Y_base}")

        # Map coordinate system
        x_map = x_robot + X_base * np.cos(theta_robot) - Y_base * np.sin(theta_robot)
        y_map = y_robot + X_base * np.sin(theta_robot) + Y_base * np.cos(theta_robot)
        # rospy.loginfo(f"x_map: {x_map}, y_map: {y_map}")

        self.barrel_pose_x = x_map
        self.barrel_pose_y = y_map
        self.barrel_pose_z = 0.0
        # rospy.loginfo(f"barrel_pose_x: {self.barrel_pose_x}, barrel_pose_y: {self.barrel_pose_y}")

    def pub_barrel_waypoint(self):
        self.barrel_pose.header.frame_id = "map"
        self.barrel_pose.header.stamp = rospy.Time.now()
        self.barrel_pose.pose.position.x = self.barrel_pose_x
        self.barrel_pose.pose.position.y = self.barrel_pose_y
        self.barrel_pose.pose.position.z = 0
        self.barrel_waypoint_pub.publish(self.barrel_pose)
        rospy.loginfo(f"Barrel waypoint published: {self.barrel_pose}")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            rospy.logerr("Depth image conversion error: %s", str(e))

    def get_depth(self, x, y):
        if self.latest_depth is None:
            return -1  
        h, w = self.latest_depth.shape
        if not (0 <= x < w and 0 <= y < h):
            return -1  
        depth_value = self.latest_depth[y, x]
        if depth_value <= 0 or np.isnan(depth_value):
            return -1  
        return depth_value  

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([15, 100, 100])
            upper_orange = np.array([25, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            orange_pixels = cv2.countNonZero(mask)

            if orange_pixels > 3:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(largest_contour) > 3:
                        M = cv2.moments(largest_contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            depth = self.get_depth(cx, cy)

                            cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)  
                            cv2.drawContours(cv_image, [largest_contour], -1, (255, 0, 0), 2)  

                            self.transform_from_camera_to_map(cx, cy, depth)
                            if self.barrel_pose_x is not None and self.barrel_pose_y is not None:
                                self.pub_barrel_waypoint()


                            rospy.loginfo(f"object:({cx}, {cy}), depth:{depth}")

            cv2.imshow("Orange", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Image processing error: %s", str(e))

if __name__ == "__main__":
    Orange()
    rospy.spin()
    cv2.destroyAllWindows()