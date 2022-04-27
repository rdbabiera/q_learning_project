#!/usr/bin/env python3

import rospy
import numpy as np
import os

from q_learning_project.msg import QLearningReward, QMatrix, RobotMoveObjectToTag

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

# Path of directory to locate where to store q_matrices
qmatrix_path = os.path.dirname(__file__) + "/q_matrix/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        

        #--- Initialize Q Matrix Publishers and Subscribers
        self.q_update_pub = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
        self.q_action_pub = rospy.Publisher('/q_learning/robot_action', RobotMoveObjectToTag, queue_size=10)

        self.q_reward_sub = rospy.Subscriber('/q_learning/reward', QLearningReward, self.update_q_matrix)

        print("about to sleep")
        rospy.sleep(5)
        print("time to go!")

        # Initialize Q-Matrix, Constants
        self.q_matrix = np.zeros((64, 9), dtype='f')
        self.q_matrix.fill(-1)

        self.q_convergence = False
        self.iterations = 0
        self.curr_state = 0
        self.curr_action = 0

        self.epsilon = 0.001
        self.alpha = 1.0
        self.gamma = 0.8

        # Run Q Learning Algorithm
        self.init_q_matrix()


    def update_q_matrix(self, data):
        # Update Q Matrix after Retrieving Reward
        print(f"in callback, data iteration is {data.iteration_num}")
        if data.iteration_num > self.iterations:
        # if True:
            next_state = 1
            max_reward_next_state = 1
            #for action in self.q_matrix[next_state]:
            #    if action > max_reward_next_state:

            self.q_matrix[self.curr_state, self.curr_action] += (self.alpha * max_reward_next_state)
            self.curr_state = self.get_new_state(int(self.curr_state), self.curr_action)
            self.iterations = data.iteration_num
            print(f"Set Iteration to {self.iterations}")


            q_matrix = QMatrix()
            q_matrix.q_matrix = self.q_matrix
            r = rospy.Rate(5)
            for i in range(0, 5):
                self.q_update_pub.publish(q_matrix)
                r.sleep()
            
            self.select_random_action()
        print("exiting callback")


    def init_q_matrix(self):
        
        # For every state (row) in the action matrix,
        #   for every column, if column is not -1, set that value in the qmatrix to 0 (represents valid action)
        for i, c in enumerate(self.action_matrix):
            for action in c:
                if action != -1:
                    self.q_matrix[i, int(action)] = 0

        self.select_random_action()
        
        # expected_iteration = 0

        # while not self.q_convergence:
        #     print(f"expecting {expected_iteration}")
        #     while (self.iterations != expected_iteration):
        #         continue
        #     # Set Lock
        #     print(f"set lock, curr state {self.curr_state}")
        #     expected_iteration += 1
        # 
        #     # Randomly Generate Action
        #     action = np.random.randint(0, 9)
        #     while self.q_matrix[int(self.curr_state), action] == -1:
        #         action = np.random.randint(0, 9)
        #     self.curr_action = action
        #     print(f"set action to {action}")
# 
        #     # Create Object
        #     command = RobotMoveObjectToTag()
        #     command.robot_object = self.actions[self.curr_action]['object']
        #     command.tag_id = self.actions[self.curr_action]['tag']
        #     print(f"Command: object: {command.robot_object}, tag: {command.tag_id}")
# 
        #    # Publish To Topic
        #     for i in range(0, 1):
        #         self.q_action_pub.publish(command)
    
    def get_new_state(self, state, action):
        #print(f"State {state}")
        #print(self.action_matrix[state])
        for s, a in enumerate(self.action_matrix[state]):
            if a == action:
                return s
        return -1

    def select_random_action(self):
        # Randomly Generate Action
        action = np.random.randint(0, 9)
        while self.q_matrix[self.curr_state, action] == -1:
            action = np.random.randint(0, 9)
        self.curr_action = action

        # Create Object
        command = RobotMoveObjectToTag()
        command.robot_object = self.actions[self.curr_action]['object']
        command.tag_id = self.actions[self.curr_action]['tag']
        print(f"Command: object: {command.robot_object}, tag: {command.tag_id}")

        # Publish To Topic
        r = rospy.Rate(5)
        for i in range(0, 5):
            self.q_action_pub.publish(command)
            r.sleep()
        print("exiting publisher")


    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        numpy.savetxt(qmatrix_path + "q_matrix.csv", self.q_matrix, delimiter=",")
        return

if __name__ == "__main__":
    node = QLearning()
    rospy.spin()
