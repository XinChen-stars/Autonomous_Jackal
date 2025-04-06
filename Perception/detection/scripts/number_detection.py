#!/usr/bin/env python3
 # -*- coding: utf-8 -*-
 
import os
import tf
import cv2
import math
import rospy
import easyocr
import tempfile
import warnings
import numpy as np
import message_filters
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.distance import cdist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
warnings.filterwarnings("ignore", category=FutureWarning)
 

class BoxDetectorNode:
    def __init__(self):
        rospy.init_node('box_detector_node', anonymous=True)
        self.bridge = CvBridge()
        

        self.reader = easyocr.Reader(['en'], gpu=False)

        # subscribers
        self.rgb_sub = message_filters.Subscriber("/realsense/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/realsense/depth/image_rect_raw", Image)
        self.pointcloud_sub = message_filters.Subscriber("/realsense/depth/color/points", PointCloud2)
        self.odom_sub = message_filters.Subscriber("/Odometry", Odometry)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.pointcloud_sub, self.odom_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

        # initialize variables
        self.detected_boxes = []  # [(x, y, z, digit), ...] 
        self.digit_dict = {}      # digit -> {"count": int, "coords": [(x, y, z), ...] }
        
        # number appearance statistics
        self.Digit_appear = []   # [(digit, count), ...]
        print("Digit_appear:", self.Digit_appear)
        # point cloud parameters
        self.cluster_tolerance = 0.5  # point cloud clustering tolerance
        self.min_cluster_size = 10    # minimum cluster size
        self.digit_clouds = {}  # digit -> [[point1, point2, ...], ...]
        
        # camera intrinsic parameters
        self.fx = 337.2084410968044
        self.fy = 337.2084410968044
        self.cx = 320.5
        self.cy = 240.5

        # odometry
        self.position = None
        self.orientation = None

        # offset
        self.x_offset = 0.2
        self.y_offset = 0.0
        self.z_offset = 0.21

        # distance threshold for duplicate detection
        self.duplicate_threshold = 1.2  # 

        # initialize digit_appear list
        self.last_display_time = rospy.Time.now()
        
        rospy.loginfo("BoxDetectorNode initialized.")

    def synced_callback(self, rgb_msg, depth_msg, pointcloud_msg, odom_msg):
        # get the odometry data
        self.position = odom_msg.pose.pose.position
        self.orientation = odom_msg.pose.pose.orientation

        if self.position.x <= 10.0:
            return

        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        gray = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
            temp_filename = tmp.name
        cv2.imwrite(temp_filename, enhanced) 
        try:
            results = self.reader.readtext(temp_filename, detail=1)
        except Exception as e:
            rospy.logerr("EasyOCR error: {}".format(e))
            os.remove(temp_filename)
            return
        os.remove(temp_filename)

        for det in results:
            coords, text, confidence = det
            if confidence < 0.6:  # OCR detection threshold
                continue
            
            # only deal with single digit 1-9
            if len(text) == 1 and text.isdigit() and text in [str(n) for n in range(1,10)]:
                detected_digit = text 
            else:
                continue

            # calculate center pixel coordinates
            x0,y0 = coords[0]
            x1,y1 = coords[1]
            x2,y2 = coords[2]
            x3,y3 = coords[3]
            x_c = (x0 + x1 + x2 + x3) / 4.0
            y_c = (y0 + y1 + y2 + y3) / 4.0

            u = int(round(x_c))
            v = int(round(y_c))
            
            if not (0 <= u < cv_depth.shape[1] and 0 <= v < cv_depth.shape[0]):
                continue

            # anomaly check, NaN and not positive depth will be ignored
            center_depth = cv_depth[v, u]
            if center_depth <= 0 or np.isnan(center_depth):
                continue

            # abstract point cloud coordinates of the digit box
            digit_box_points = self.extract_box_points(
                pointcloud_msg, 
                int(min(x0, x1, x2, x3)), 
                int(min(y0, y1, y2, y3)),
                int(max(x0, x1, x2, x3)), 
                int(max(y0, y1, y2, y3))
            )
                
            # PC -> Map
            try:
                global_points = self.transform_points_to_map(digit_box_points)
                if not global_points:
                    continue
            except Exception as e:
                rospy.logwarn(f"point cloud transformation error: {e}")
                continue
                
            # calculate the center of the PC
            center_point = np.mean(global_points, axis=0)
            x_global, y_global, z_global = center_point
            
            rospy.loginfo(
                f"detection number {detected_digit} (conf={confidence:.2f}):" 
                f"\n  point cloud center: ({x_global:.2f}, {y_global:.2f}, {z_global:.2f})"
                f"\n  point cloud number: {len(global_points)}"
            )
            
            # use point cloud to check duplicate box
            is_duplicate = self.is_duplicate_by_pointcloud(
                detected_digit, 
                global_points, 
                self.duplicate_threshold
            )
            
            if is_duplicate:
                rospy.loginfo(f"number {detected_digit} is repeated")
                continue

            self.detected_boxes.append((x_global, y_global, z_global, detected_digit))
            
            if detected_digit not in self.digit_dict:
                self.digit_dict[detected_digit] = {"count": 0, "coords": []}
                self.digit_clouds[detected_digit] = []

            self.digit_dict[detected_digit]["count"] += 1
            self.digit_dict[detected_digit]["coords"].append((x_global, y_global, z_global))
            self.digit_clouds[detected_digit].append(global_points)
            
            # renew digit_appear list
            self.update_digit_appear(detected_digit)

            rospy.loginfo(
                f" new digit box {detected_digit} (conf={confidence:.2f}) added to globlal odom. "
                f"({x_global:.2f}, {y_global:.2f}, {z_global:.2f}). "
                f"总计: {self.digit_dict[detected_digit]['count']}"
            )
            
        # every 5 seconds, display the digit appearance statistics
        current_time = rospy.Time.now()
        if (current_time - self.last_display_time).to_sec() > 2.0: 
            self.display_digit_appear()  # display the digit appearance statistics
            self.last_display_time = current_time
            
    def update_digit_appear(self, digit):
        """update the digit_appear list with the new digit"""
        # check if the digit already exists in the list
        for i, (d, count) in enumerate(self.Digit_appear):
            if d == digit:
                # update the count
                self.Digit_appear[i] = (d, count + 1)
                return
                
        # if the digit is not in the list, add it
        self.Digit_appear.append((digit, 1))
        
        # sort the list by digit
        self.Digit_appear.sort(key=lambda x: int(x[0]))
            
    def display_digit_appear(self):
            
        # calculate total number of digits
        total_count = sum(count for _, count in self.Digit_appear)
            
        rospy.loginfo("\n" + "=" * 40)
        rospy.loginfo("===== Digit Detection Statistics (Updated every 5 seconds) =====")

        # create a table format display
        header = "| number | count | percentage |"
        separator = "|------|----------|--------|"
        
        rospy.loginfo(header)
        rospy.loginfo(separator)

        # Find the digit with minimum count
        min_digit = None
        min_count = float('inf')
        
        for digit, count in self.Digit_appear:
            percentage = (count / total_count * 100) if total_count > 0 else 0
            row = f"|  {digit}   |    {count}    | {percentage:.1f}%  |"
            rospy.loginfo(row)

            # Track minimum count
            if count < min_count:
                min_count = count
                min_digit = digit
            
        rospy.loginfo(separator)
        rospy.loginfo(f"Total: {total_count} digits detected, {len(self.Digit_appear)} different numbers")

        if min_digit is not None:
            rospy.loginfo(f"Least detected digit: {min_digit} (count: {min_count})")

        rospy.loginfo("=" * 40 + "\n")
    
    def extract_box_points(self, pointcloud_msg, x_min, y_min, x_max, y_max):
        """get the point cloud points in the bounding box"""
        box_points = []
        
        for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x_cam, y_cam, z_cam = point[0], point[1], point[2]
            
            if z_cam <= 0:
                continue
                
            u = int(self.fx * x_cam / z_cam + self.cx)
            v = int(self.fy * y_cam / z_cam + self.cy)
            
            if x_min <= u <= x_max and y_min <= v <= y_max:
                box_points.append([x_cam, y_cam, z_cam])
        
        return box_points
    
    def transform_points_to_map(self, points):
        """transform points from camera coordinates to map coordinates"""

        x_robot = self.position.x
        y_robot = self.position.y

        # get robot orientation
        quat = self.orientation
        quaternion = (quat.x, quat.y, quat.z, quat.w)

        # convert quaternion to euler angles
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        theta_robot = yaw
        if theta_robot is None:
            rospy.logwarn("no orientation")
            return

        x_offset = self.x_offset
        y_offset = self.y_offset
        z_offset = self.z_offset

        global_points = []

        # FLU camera coordinate system
        for point in points:
            x_cam = point[0]
            y_cam = point[1]
            z_cam = point[2]

            # RDF camera coordinate system
            X_FLU = z_cam
            Y_FLU = -x_cam
            Z_FLU = -y_cam

            # FLU base coordinate system
            X_base = X_FLU + x_offset
            Y_base = Y_FLU + y_offset

            # Map coordinate system
            x_map = x_robot + X_base * np.cos(theta_robot) - Y_base * np.sin(theta_robot)
            y_map = y_robot + X_base * np.sin(theta_robot) + Y_base * np.cos(theta_robot)
            z_map = z_cam + z_offset
            global_points.append([x_map, y_map, z_map])
        
        return global_points
            
    def is_duplicate_by_pointcloud(self, digit, current_points, threshold):
        if digit not in self.digit_clouds or not current_points:
            return False

        current_points = np.array(current_points)
        current_center = np.mean(current_points, axis=0)
        current_center_xy = current_center[:2]

        for existing_points in self.digit_clouds[digit]:
            existing_points = np.array(existing_points)
            existing_center = np.mean(existing_points, axis=0)
            existing_center_xy = existing_center[:2]

            # Check center distance
            dist_xy = np.linalg.norm(current_center_xy - existing_center_xy)
            if dist_xy < threshold:
                rospy.loginfo(f"Point cloud center distance is {dist_xy:.2f}m, less than threshold {threshold}m")
                return True

            # Check point-to-point minimum distance
            dist_matrix = cdist(current_points[:, :2], existing_points[:, :2])
            min_dist_xy = np.min(dist_matrix)
            if min_dist_xy < threshold:
                rospy.loginfo(f"Minimum point distance on XY plane is {min_dist_xy:.2f}m, less than threshold {threshold}m")
                return True

            rospy.logdebug(f"Digit {digit} not duplicated, minimum distance {min_dist_xy:.2f}m, greater than threshold {threshold}m")

        return False

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = BoxDetectorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass