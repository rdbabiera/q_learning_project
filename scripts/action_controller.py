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
        
        # Tips:
        # arm_joint_goal is a list of 4 radian values, 1 for each joint
        # for instance,
        #           arm_joint_goal = [0.0,0.0,0.0,0.0]
        #           arm_joint_goal = [0.0,
        #               math.radians(5.0),
        #               math.radians(10.0),
        #               math.radians(-20.0)]
    

        # wait=True ensures that the movement is synchronous
        # arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
        arm_joint_goal = [math.radians(joints[0]),
                          math.radians(joints[1]),
                          math.radians(joints[2]),
                          math.radians(joints[3]),]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

    def open_grip(self):

        # gripper_joint_goal is a list of 2 radian values, 1 for the left gripper and 1 for the right gripper
        # for instance,
        #           gripper_joint_goal = [-0.009,0.0009]
        #           gripper_joint_goal = [0.0, 0.0]
        gripper_joint_goal = [-0.009,0.0009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        
    def close_grip(self):

        # gripper_joint_goal is a list of 2 radian values, 1 for the left gripper and 1 for the right gripper
        # for instance,
        #           gripper_joint_goal = [-0.009,0.0009]
        #           gripper_joint_goal = [0.0, 0.0]
        gripper_joint_goal = [0.0, 0.0]
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
        self.color_codes["pink"] = {'high': [165, 255, 217], 'low': [161, 171, 161]}
        self.color_codes["green"] = {'high': [37, 255, 230], 'low': [30, 192, 137]}
        self.color_codes["blue"] = {'high': [100, 225, 232], 'low': [94, 94, 122]}

        """
        Sleep for Warmup
        """
        
        rospy.sleep(5)
    
    def scan_callback(self, data):
        self.last_scan = data

    def image_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.last_image = image
        cv2.imshow("window", image)
        cv2.waitKey(3)
        pass

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
        #--- 2.1 Verify Object Picked Up
        #while not self.verify_object_picked_up(color, color_codes):
        #    self.navigate_to_object(color, color_codes)
        #    self.pickup()
        
        # 3. Locate Tag
        # 4. Goto Tag
        self.backup_robot()
        self.navigate_to_tag(tag_id)
        # 5. Drop Object

        self.drop()
        #--- 5.1 Verify Dropoff
        # 6. Back Away from Object
        self.backup_robot()
    
    def backup_robot(self):
        print("[CLIENT] ROBOT BACKING UP")
        command = Twist()
        rate = rospy.Rate(3)

        command.linear.x = -0.10
        for i in range(0, 10):
            self.move_pub.publish(command)
            rate.sleep()
        command.linear.x = 0
        for i in range(0, 10):
            self.move_pub.publish(command)
            rate.sleep()
        print("[CLIENT] ROBOT STOPPED")

    def navigate_to_object(self, color, color_codes):
        print(f"[CLIENT] Navigating to {color} object")
        
        # State Variables
        found = False
        command = Twist()

        # Find Object
        while not found:
            img = self.last_image
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, np.array(color_codes['low']), np.array(color_codes['high']))
            M = cv2.moments(mask)
            if M['m00'] > 0:
                found = True
                command.angular.z = 0
                for i in range(0, 10):
                    self.move_pub.publish(command)
            else:
                command.angular.z = 0.60

            self.move_pub.publish(command)
        
        # Navigate to Object
        at_object = False
        while not at_object:
            # Image Da
            img = self.last_image
            h, w, d = img.shape
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array(color_codes['low']), np.array(color_codes['high']))
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                command = Twist()
                # Determine Linear Speed:
                scan = self.last_scan
                ranges = scan.ranges
                average_distance = 0
                count = 0
                for r in range(357, 364):
                    ind = r % 360
                    if ranges[ind] != np.inf and ranges[ind] > 0:
                        average_distance += ranges[ind]
                        count += 1
                if count > 0:
                    average_distance /= count
                if (average_distance > 0) and (average_distance < 0.35):
                    command.linear.x = 0
                    at_object = True
                else:
                    command.linear.x = 0.10
                
                command.angular.z = (-cx + (w/2)) * 0.001
                self.move_pub.publish(command)

        command.linear.x = 0
        command.angular.z = 0
        for i in range(0, 10):
            self.move_pub.publish(command)
        print(f"[CLIENT] Finished Navigating to {color} object")


    def navigate_to_tag(self, tag_id):
        print(f"[CLIENT] Navigating to tag {tag_id}")
        found = False
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
                command.angular.z = 0.60

            self.move_pub.publish(command)
        
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
            for r in range(357, 364):
                ind = r % 360
                if ranges[ind] != np.inf and ranges[ind] > 0:
                    average_distance += ranges[ind]
                    count += 1
            if count > 0:
                average_distance /= count
            if (average_distance > 0) and (average_distance < 0.55):
                command.linear.x = 0
                at_object = True
            else:
                command.linear.x = 0.20
            self.move_pub.publish(command)
        
        command.linear.x = 0
        command.angular.z = 0
        for i in range(0, 10):
            self.move_pub.publish(command)
        print(f"[CLIENT] Finished Navigating to tag {tag_id}")
        


    def pickup(self):

        arm_straight_goal = [0, 0, 0, 0]

        arm_lift_goal = [81, 45, 39, 58]
        
        self.movement.linear.x = 0.01
        self.movement.angular.z = 0

        self.arm.move_arm(arm_straight_goal)
        self.arm.open_grip()

        self.move_pub.publish(self.movement)
        rospy.sleep(0.2)

        self.arm.close_grip()

        self.arm.move_arm(arm_lift_goal)

    def drop(self):

        arm_straight_goal = [0, 0, 0, 0]

        self.arm.move_arm(arm_straight_goal)

        self.arm.open_grip()


    def verify_object_picked_up(self, color, color_codes):
        print(f"[CLIENT] Verifying {color} object picked up")
        img = self.last_image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, np.array(color_codes['low']), np.array(color_codes['high']))
        M = cv2.moments(mask)
        if M['m00'] > 0:
            print(f"[CLIENT] Verification Failed... Robot can still see {color} object")
            return False
        print(f"[CLIENT] Verification Successful!")
        return True


    def run(self):
        self.get_action()
        rospy.spin()

if __name__ == "__main__":
    robot = Robot()
    robot.run()

