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

        rospy.sleep(5)

        # Debugging Tools
        self.debug = True

        # Initialize Q-Matrix, Constants
        self.q_matrix = np.zeros((64, 9), dtype='f')
        self.q_matrix.fill(-1)

        self.q_convergence = False
        self.curr_state = 0
        self.curr_action = 0

        self.flag = False
        self.curr_reward = 0

        self.epsilon = 0.0001
        self.alpha = 1.0
        self.gamma = 0.8

        self.previous_updates = []

        # Run Q Learning Algorithm
        self.init_q_matrix()
        self.run_q_learning()


    # Primary Method: init_q_matrix() creates the self.q_matrix object
    def init_q_matrix(self):
        # For every state (row) in the action matrix,
        #   for every column, if column is not -1, set that value in the qmatrix to 0 (represents valid action)
        for i, c in enumerate(self.action_matrix):
            for action in c:
                if action != -1:
                    self.q_matrix[i, int(action)] = 0

    # Primary Method: run_q_learning() fills out the self.q_matrix object until 
    # convergence
    def run_q_learning(self):
        expected_iteration = 0
        while not self.q_convergence:
            self.flag = False
            self.select_random_action()

            # Wait for Reward
            while not self.flag:
                continue

            # Update Q Matrix
            reward = self.reward
            # Update Q Matrix after Retrieving Reward
            next_state = self.get_new_state(int(self.curr_state), self.curr_action)
            max_reward_next_state = 0
            for action_reward in self.q_matrix[next_state]:
                if action_reward > max_reward_next_state:
                    max_reward_next_state = action_reward

            print(f"{reward} + {self.gamma * max_reward_next_state} - {self.q_matrix[self.curr_state, self.curr_action]}")
            update = self.alpha * (reward + (self.gamma * max_reward_next_state) - self.q_matrix[self.curr_state, self.curr_action])
            self.q_matrix[self.curr_state, self.curr_action] += update

            if len(self.previous_updates) == 1000:
                self.previous_updates.pop(0)
            self.previous_updates.append(update)

            print(f"Update: {update}, Reward: {reward} at state {self.curr_state}")

            if self.check_reset(next_state):
                self.curr_state = 0
                if (len(self.previous_updates) == 1000) and (sum(self.previous_updates) / 1000) < self.epsilon:
                    print(f"Update: {float(update)}")
                    print(f"Converged at state {next_state}!")
                    self.q_convergence = True
            else:
                self.curr_state = next_state
            
            q_matrix = QMatrix()
            q_matrix.q_matrix = self.q_matrix
            self.q_update_pub.publish(q_matrix)

        self.save_q_matrix()
        exit(0)


    # Helper Method: check_reset(state) sets state back to zero if all three 
    # objects are not at the origin
    def check_reset(self, state):
        for s in self.states[state]:
            if s == 0:
                return False
        return True


    # Callback Method: update_q_matrix(data) updates the q_matrix on each robot 
    # action followed by a reward
    def update_q_matrix(self, data):
        # print("Callback")
        self.reward = data.reward
        self.flag = True


    # Helper Method: get_new_state(state, action) retrieves the next state 
    # given the current state and the action taken.
    def get_new_state(self, state, action):
        for s, a in enumerate(self.action_matrix[state]):
            if a == action:
                return s
        return -1

    # Helper Method: select_random_action() selects a random possible action 
    # and publishes it to the robot.
    def select_random_action(self):
        # Randomly Generate Action
        action = np.random.randint(0, 9)
        curr_state = self.curr_state
        while self.q_matrix[curr_state, action] == -1:
            action = np.random.randint(0, 9)
        self.curr_action = action

        # Create Object
        command = RobotMoveObjectToTag()
        command.robot_object = self.actions[self.curr_action]['object']
        command.tag_id = self.actions[self.curr_action]['tag']

        # Publish To Topic
        self.q_action_pub.publish(command)


    def save_q_matrix(self):
        # You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        np.savetxt(qmatrix_path + "q_matrix.csv", self.q_matrix, delimiter=",")
        return

if __name__ == "__main__":
    node = QLearning()
    rospy.spin()
