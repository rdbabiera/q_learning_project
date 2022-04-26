# q_learning_project
## Implementation Plan

### Team Members (github/cnet):
RD Babiera (rdbabiera), Diogo Viveiros(diogoviveiros)

### Questions (1-2 sentences on implementation, 1-2 sentences on testing)
1. Q-Learning Algorithm
- Executing the Q-Learning Algorithm
    In order to execute the Q-Learning Algorithm, we will create a set of rules and rewards that matches the desired output that we will then want the robot to take during the action phase. More specifically, we will have the following rewards for these specific states: 
      - If the robot moves a certain object to the right tag, reward it with a positive value.
      - If the robot moves the object to a place without a tag, don't reward it with anything/
      - If the robot moves the an object to the wrong tag, penalize it with a negative value. 
    We will then repeat these actions until the robot has figured out the rewards through the Q-Matrix converging. We will test whether this is working or not by having the robot in the Training phase, and moving objects around, while printing the overall reward when they move an object to the tag. If the reward given matches what we wrote above, then it will have worked!

a

- Determining when the Q-Matrix has converged
  As we keep training the Q-Learning algorithm in the Training phase, the rate at which the quality changes will reduce over time as the matrix gets closer and closer to its most "optimal" states. However, with so many different combinations of states and possible "trajectories", it may take a really long time to make the Q-Matrix completely optimal. This will require some testing and experimentaton, but if the time to test the Q-Matrix is too long, then we should introduce a "delta threshold" variable, where if the improvement from the previous execution of the algorithm is fairly low, then we state that the matrix has converged to a statistically good degree and we can stop the Training phase. We will test this by measuring the time it takes to go through these trajectories and states. 

a

- Once the Q-Matrix has converged, how how to determine which actions the robot 
should take to maximize expected reward
  Once the convergence has happened, we should program the robot to do an analysis at the beginning of the Action phase to analyze its environment and see whether there are certain paths which will lead it to make a better reward. We can test this by executing the robot during the Action phase and seeing if its logical position makes sense by also observing the converged Q-Matrix. 

a

2. Robot Perception
- Determining the identities and locations of the three colored objects

a

- Determining the identities and locations of the three AR tags

a

3. Robot Manipulation & Movement
- Picking up and putting down the colored objects with the OpenMANIPULATOR arm

a

- Navigating to the appropriate locations to pick up and put down the colored 
objects

a


### Timeline
4/26: Meet up, talk about ideas, and deduce an implementation plan. 

4/30

5/03

5/06

5/09

5/11

## Writeup
### Objectives
#TODO 

### High Level Description
#TODO 

### Demonstration
#TODO 

### Q-Learning Algorithm
#TODO - Describe how you accomplished each of the following components of the 
Q-learning algorithm in 1-3 sentences, and also describe what functions / 
sections of the code executed each of these components (1-3 sentences per 
function / portion of code): 

1. Selecting and executing actions for the robot (or phantom robot) to take

a

2. Updating the Q-Matrix

a

3. Determining when to stop iterating through the Q-learning algorithm

a

4. Executing the path most likely to lead to receiving a reward after the 
Q-matrix has converged on the simulated Turtlebot3 robot

a

### Robot Perspective
#TODO - Describe how you accomplished each of the following components of the 
perception elements of this project in 1-3 sentences, any online sources of 
information/code that helped you to recognize the objects, and also describe what 
functions / sections of the code executed each of these components (1-3 sentences 
per function / portion of code): 

1. Identifying the locations and identities of each of the colored objects

a

2. Identifying the locations and identities of each of the AR tags

a

### Robot Manipulation and Movement
#TODO - Describe how you accomplished each of the following components of the 
robot manipulation and movement elements of this project in 1-3 sentences, and 
also describe what functions / sections of the code executed each of these 
components (1-3 sentences per function / portion of code): 

1. Moving to the right spot in order to pick up a colored object

a

2. Picking up the colored object

a

3. Moving to the desired destination (AR tag) with the colored object

a

4. Putting the colored object back down at the desired destination

a

### Challenges

### Future Work

### Takeaways
