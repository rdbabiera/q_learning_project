#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
import numpy as np
import moveit_commander
import math
import os

from q_learning_project.msg import Control
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from q_learning_project.srv import StateService, StateServiceResponse

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

# Path of directory to locate where to store q_matrices
qmatrix_path = os.path.dirname(__file__) + "/q_matrix/"

class Arm(object):

    def __init__(self):


        """
        Setup Arm Control
        """
        # initialize this node
        rospy.init_node('turtlebot3_arm_controller')

        # Traffic status subscriber
        #rospy.Subscriber("/traffic_status", Control, self.traffic_dir_received)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

              
    def move_arm(self, joints):
        
        arm_joint_goal = [math.radians(joints[0]),
                          math.radians(joints[1]),
                          math.radians(joints[2]),
                          math.radians(joints[3]),]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()


    # open_grip() causes the arm to open its grip
    def open_grip(self):

        # gripper_joint_goal is a list of 2 distance values
        gripper_joint_goal = [0.012,0.0012]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        
    # close_grip() causes the arm to close its grip
    def close_grip(self):

        # gripper_joint_goal is a list of 2 radian values, 1 for the left 
        # gripper and 1 for the right gripper. for instance,
        #     gripper_joint_goal = [-0.009,0.0009]
        #     gripper_joint_goal = [0.0, 0.0]
        gripper_joint_goal = [-0.002, -0.002]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()


class Robot(object):

    def __init__(self):

        """
        Setup open cv image recognition and RGB Camera hardware
        """

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        # cv2.namedWindow("window", 1)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.last_image = None
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # Movement Publisher/Subscriber
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.last_scan = None

        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        """
        Initialize Arm object
        """

        self.arm = Arm()
        self.movement = Twist()

        """
        Set Up Action Array, Colors to HSV
        """""""""
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))
        self.color_codes = {}
        self.color_codes["pink"] = {'high': [165, 192, 217], 'low': [152, 102, 63]}
        self.color_codes["green"] = {'high': [45, 204, 217], 'low': [30, 114, 63]}
        self.color_codes["blue"] = {'high': [105, 192, 217], 'low': [95, 102, 63]}

        """
        Sleep for Warmup
        """
        
        rospy.sleep(5)


    # updates last scan on received data
    def scan_callback(self, data):
        self.last_scan = data


    # updates last image on received data
    def image_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.last_image = image


    # sends a request to the state controller for the next action
    def get_action(self):
        print("[CLIENT] In get_action()")
        rospy.wait_for_service('action_request_service')
        try:
            run = True
            while run:
                print("[CLIENT] Requesting Action")
                receive_action = rospy.ServiceProxy('action_request_service', StateService)
                response = receive_action("Ready")
                if response.action == -1:
                    run = False
                    print("There's Nothing Left to Do, Shutting Down")
                    exit(1)
                print(f"[CLIENT] Received action {response.action}")
                self.perform_action(response.action)
        except rospy.ServiceException as e:
            print("Service Call Failed: %s"%e)
        return

    
    # given an action, perform all basic steps
    def perform_action(self, action):
        print(f"[CLIENT] Performing action {action}")
        ### Preliminary Steps
        act_data = self.actions[action]
        color = act_data["object"]
        tag_id = act_data["tag"]
        color_codes = self.color_codes[color]
        print(f"[CLIENT] Looking for tag id {tag_id}, Color: {color}")
        print(f"[CLIENT] '{color}' HSV Data: {color_codes}")

        ### Main Steps
        # 1. Go To Object
        self.navigate_to_object(color, color_codes)
        # 2. Pickup Object
        self.pickup()
        self.lift()
        
        # 3. Locate Tag
        # 4. Goto Tag
        self.backup_robot()
        self.navigate_to_tag(tag_id)
        # 5. Drop Object
        self.drop()
        self.lift()

        # 6. Back Away from Object
        self.backup_robot()
    

    # after picking up or putting down an object, back up to avoid collision
    def backup_robot(self):
        print("[CLIENT] ROBOT BACKING UP")
        command = Twist()
        rate = rospy.Rate(3)

        command.linear.x = -0.10
        for i in range(0, 10):
            self.move_pub.publish(command)
            rate.sleep()
        command.linear.x = 0
        for i in range(0, 5):
            self.move_pub.publish(command)
            rate.sleep()
        print("[CLIENT] ROBOT STOPPED")


    # given a color, navigate to said object
    def navigate_to_object(self, color, color_codes):
        print(f"[CLIENT] Navigating to {color} object")
        
        # State Variables
        found = False
        rate = rospy.Rate(2)
        command = Twist()

        while self.last_image is None:
            rate.sleep()
            continue
        
        print("[CLIENT] Received Images!")

        while self.last_scan is None:
            rate.sleep()
            continue

        print("[CLIENT] Received LiDAR Data!")

        # Find Object
        while not found:
            img = self.last_image
            h, w, d = img.shape
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array(color_codes['low']), np.array(color_codes['high']))
            M = cv2.moments(mask)
            if M['m00'] > 0:
                found = True
                command.angular.z = 0
                for i in range(0, 5):
                    self.move_pub.publish(command)
                    rate.sleep()
            else:
                command.angular.z = 0.25

            self.move_pub.publish(command)
            rate.sleep()
        print(f"[CLIENT] Finished Finding {color} object")

        # Align to Object
        aligned = False
        while not aligned:
            img = self.last_image
            h, w, d = img.shape
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array(color_codes['low']), np.array(color_codes['high']))
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                if (cx >= ((w/2) - (w * .05))) and (cx <= ((w/2) + (w * .05))):
                    aligned = True
                    command.angular.z = 0
                else:
                    command.angular.z = (-cx + (w/2)) * 0.001

            self.move_pub.publish(command)
            rate.sleep()
        print(f"[CLIENT] Aligned with {color} object!")

        
        # Navigate to Object
        at_object = False
        while not at_object:
            command = Twist()
            # Determine Linear Speed:
            scan = self.last_scan
            ranges = scan.ranges
            lowest_index = 0
            lowest_distance = 4.5
            for r in range(355, 366):
                count = 0
                avg_distance = 0
                ind = r % 360
                for r2 in range(r-3, r+4):
                    ind2 = r2 % 360
                    if ranges[ind2] != np.inf and ranges[ind2] > 0:
                        avg_distance += ranges[ind2]
                        count += 1
                if count > 0:
                    avg_distance /= count
                    if avg_distance < lowest_distance:
                        lowest_distance = avg_distance
                        lowest_index = ind

            if lowest_distance > 0 and lowest_distance < 0.30:
                command.linear.x = 0
                command.angular.z = 0
                at_object = True
            else:
                command.linear.x = 0.075
                if lowest_index > 180:
                    command.angular.z = (lowest_index - 360) * 0.01
                else:
                    command.angular.z = (lowest_index - 0) * 0.01
            
            self.move_pub.publish(command)
            rate.sleep()

        command.linear.x = 0
        command.angular.z = 0
        for i in range(0, 5):
            self.move_pub.publish(command)
            rate.sleep()
        print(f"[CLIENT] Finished Navigating to {color} object")


    # given a tag, navigate to it
    def navigate_to_tag(self, tag_id):
        print(f"[CLIENT] Navigating to tag {tag_id}")
        found = False
        rate = rospy.Rate(2)
        command = Twist()
        command.linear.x = 0

        # Spin until Tag in View
        while not found:
            img = self.last_image
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected_points = cv2.aruco.detectMarkers(gray, 
                self.aruco_dict)
            if ids is not None:
                for tag in ids:
                    if tag == tag_id:
                        found = True
                        command.angular.z = 0
                        for i in range(0, 10):
                            self.move_pub.publish(command)
            if not found:
                command.angular.z = 0.25

            self.move_pub.publish(command)
            rate.sleep()
        
        # Found Tag, Navigate to It
        at_object = False
        index = 0
        while not at_object:
            img = self.last_image
            h, w, d = img.shape
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected_points = cv2.aruco.detectMarkers(gray, 
                self.aruco_dict)
            if ids is not None:
                for i, tag in enumerate(ids):
                    if tag == tag_id:
                        coords = corners[i][0]
                        cx = np.sum(coords, axis=0)[0] / 4
                        command.angular.z = (-cx + (w/2)) * 0.001

            scan = self.last_scan
            ranges = scan.ranges
            average_distance = 0
            count = 0
            for r in range(358, 363):
                ind = r % 360
                if ranges[ind] != np.inf and ranges[ind] > 0:
                    average_distance += ranges[ind]
                    count += 1
            if count > 0:
                average_distance /= count
            if (average_distance > 0) and (average_distance < 0.50):
                command.linear.x = 0
                at_object = True
            else:
                command.linear.x = 0.10
            self.move_pub.publish(command)
            rate.sleep()
        
        command.linear.x = 0
        command.angular.z = 0
        for i in range(0, 10):
            self.move_pub.publish(command)
            rate.sleep()
        print(f"[CLIENT] Finished Navigating to tag {tag_id}")
        


    # pickup an object
    def pickup(self):
        rate = rospy.Rate(5)
        arm_straight_goal = [0, 28, 4, -31]
        
        self.arm.open_grip()
        for i in range(0, 5):
            self.arm.move_arm(arm_straight_goal)
            rate.sleep()

        self.movement.linear.x = 0.01
        self.movement.angular.z = 0
        for i in range(0, 30):
            self.move_pub.publish(self.movement)
            rate.sleep()

        self.movement.linear.x = 0.00
        self.movement.angular.z = 0
        for i in range(0, 15):
            self.move_pub.publish(self.movement)
            rate.sleep()

        self.arm.close_grip()


    # lift an object when in position
    def lift(self):
        rate = rospy.Rate(5)
        arm_lift_goal = [0, -12, -22, -56]
        self.arm.move_arm(arm_lift_goal)


    # drop an object when in position
    def drop(self):
        rate = rospy.Rate(5)
        arm_straight_goal = [0, 28, 4, -31]

        self.arm.move_arm(arm_straight_goal)
        rospy.sleep(2)
        self.arm.open_grip()


    def run(self):
        self.get_action()
        rospy.spin()
        

if __name__ == "__main__":
    robot = Robot()
    rospy.sleep(1)
    robot.run()

