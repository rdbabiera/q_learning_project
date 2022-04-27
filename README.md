# q_learning_project
## Implementation Plan

### Team Members (github/cnet):
RD Babiera (rdbabiera), Diogo Viveiros(diogoviveiros)

### Questions (1-2 sentences on implementation, 1-2 sentences on testing)
1. Q-Learning Algorithm
- Executing the Q-Learning Algorithm

In order to execute the Q-Learning Algorithm, we will create a set of rules and rewards that matches the desired output that we will then want the robot to take during the action phaseMore specifically, we will have the following rewards for these specific states: 

    - If the robot moves a certain object to the right tag, reward it with a positive value.
    - If the robot moves the object to a place without a tag, don't reward it with anything
    - If the robot moves the an object to the wrong tag, penalize it with a negative value. 

We will then repeat these actions until the robot has figured out the rewards through the 
Q-Matrix converging. We will test whether this is working or not by having the robot in 
the training phase, and moving objects around, while printing the overall reward when they move 
an object to the tag. If the reward given matches what we wrote above, then it will have worked!

- Determining when the Q-Matrix has converged

As we keep training the Q-Learning algorithm in the Training phase, the rate at which thequality 
changes will reduce over time as the matrix gets closer and closer to its most "optimal" states. 
However, with so many different combinations of states and possible "trajectories", it may take 
a really long time to make the Q-Matrix completely optimal. This will require some testing and 
experimentaton, but if the time to test the Q-Matrix is too long, then we should introduce a 
"delta threshold" variable, where if the improvement from theprevious execution of the algorithm 
is fairly low, then we state that the matrix has converged to a statistically good degree and we 
can stop the Training phase. We will test this by measuring the time it takes to go through these trajectories and states. 

- Once the Q-Matrix has converged, how how to determine which actions the robot 
should take to maximize expected reward

Once the convergence has happened, we should program the robot to do an analysis at the 
beginning of the Action phase to analyze its environment and see whether there are certain paths 
which will lead it to make a better reward. We can test this by executing the robot during the 
Action phase and seeing if its logical position makes sense by also observing the converged 
Q-Matrix. 

2. Robot Perception
- Determining the identities and locations of the three colored objects

In order to determine the color of each of the objects, we can utilize the raw 
camera output (/camera topic) and openCV to recognize the three different colors 
of the objects, and location of each object can be determined as a position relative 
to the robot's starting location. We can update the robot's current position using 
odometery data, and we can test our behavior by measuring the true distance from 
the origin to the object physically and comparing it to the robot's estimate of 
each object once all are seen.

- Determining the identities and locations of the three AR tags

Similar to the colored objects, we will use openCV and the camera to identify tags, 
however for the tags we will use the aruco dictionary to compare if the 
camera feed correctly identifies a tag, and we will use similar logic as above 
using LiDAR scan data relative to the robot's position in order to calculate the 
location of the center of each tag. We can test similar to the objects, in which 
we compare the robot's location estimate of each tag relative to the true distance 
from the start point.

3. Robot Manipulation & Movement
- Picking up and putting down the colored objects with the OpenMANIPULATOR arm

In order to pick and and put down objects, we can utilize the MoveIt package 
to set arm joint and gripper goals for the positions corresponding to moving 
the arm down, gripping the object, moving the arm up out of the LiDAR's way, setting 
the arm back down, and unclamping the object. We can test if this functionality 
works visually by making sure the set angle and grip goals are sufficient in 
picking up and putting the colored object delicately, and ensuring the object does 
not fall out while moving to target positions.

- Navigating to the appropriate locations to pick up and put down the colored 
objects

Given the location of the colored object, we can send cmd_vel commands to the 
robot to move it towards the location of the object. We can use LiDAR data to 
stop the robot when close and rotate it directly towards the object, and we can 
also use LiDAR data to redirect in case the robot would collide with other objects. 
We can use similar logic to position the robot right in front of the tags, and 
the LiDAR will be used to set a stopping distance right in front of the goal. 
We can test this behavior by navigating the robot to selected relative locations 
and evaluating the amount of noise in these measurements to fine-tune our 
method.

### Timeline
4/26: Meet up, talk about ideas, and deduce an implementation plan. 

4/30: Finishing pseudocode for Q-Learning Algorithm

5/03: Implementing Q-Learning Algorithm fully and testing

5/06: Understanding Action Phase better and working on robot perception and manipulation

5/09: Finish robot perception and manipulation code and begin testing, optimizing, and finding bugs

5/11: Deliver!

## Writeup
### Objectives
#TODO 


### High Level Description
#TODO 


### Q-Learning Algorithm
#TODO - Describe how you accomplished each of the following components of the 
Q-learning algorithm in 1-3 sentences, and also describe what functions / 
sections of the code executed each of these components (1-3 sentences per 
function / portion of code): 

1. Selecting and executing actions for the robot (or phantom robot) to take



2. Updating the Q-Matrix



3. Determining when to stop iterating through the Q-learning algorithm



4. Executing the path most likely to lead to receiving a reward after the 
Q-matrix has converged on the simulated Turtlebot3 robot


### Robot Perspective
#TODO - Describe how you accomplished each of the following components of the 
perception elements of this project in 1-3 sentences, any online sources of 
information/code that helped you to recognize the objects, and also describe what 
functions / sections of the code executed each of these components (1-3 sentences 
per function / portion of code): 

1. Identifying the locations and identities of each of the colored objects



2. Identifying the locations and identities of each of the AR tags



### Robot Manipulation and Movement
#TODO - Describe how you accomplished each of the following components of the 
robot manipulation and movement elements of this project in 1-3 sentences, and 
also describe what functions / sections of the code executed each of these 
components (1-3 sentences per function / portion of code): 

1. Moving to the right spot in order to pick up a colored object



2. Picking up the colored object



3. Moving to the desired destination (AR tag) with the colored object



4. Putting the colored object back down at the desired destination



### Challenges

### Future Work

### Takeaways
