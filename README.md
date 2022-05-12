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

As we keep training the Q-Learning algorithm in the Training phase, the rate at which the quality 
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
The first goal of this project is to implement Q-Learning in order to teach 
TurtleBot where to place different colored objects given unique tags. Because 
the tag locations should be randomized, the next big goal in this project 
is to both detect which colored object is which as well as its location, and 
to identify and locate the tags using camera perception. The last goal is to 
practice using robot manipulation in order to lift objects and carry them to 
their respective goalposts.


### High Level Description
Our system first runs a Q-Learning algorithm in order to find the best path(s) 
in placing each object in front of its respective tag. After initializing the 
Q-Matrix, a loop runs until the average update value of the previous 1000 
timesteps evaluates to a declared epsilon. Within this loop, a valid action 
for the current state is randomly selected, a command is published to the 
TurtleBot, and when a reward is received via a Subscriber, the Q Matrix is 
updated according to the optimization formula and the updated Q Matrix is 
published to its respective publisher topic. After convergence, the Q Matrix 
is exported to a csv file.

Given a Q-Matrix, a state controller maintains the current state of the 
objects' placements relative to the origin and the tags. An action controller 
receives actions, and the state control returns an index corresponding to an 
object and a tag. The action controller navigates the robot towards the object, 
lifts it up, and drops it in front of the tag. Once an action is completed, actions 
are requested until the state controller determines that all possible actions 
for the maximum value path have been achieved.

### Demonstration

[![Demo Video](https://img.youtube.com/vi/_2BB4v-6K34/0.jpg)](https://www.youtube.com/watch?v=_2BB4v-6K34)

### Q-Learning Algorithm

1. Selecting and executing actions for the robot (or phantom robot) to take
The algorithm which we use in order to select and execute actions for the robot is present in the "select_random_action" function present in the QLearning class. Here, we choose a random number between 0 and 8, representing each of the possible actions. Then we check if that action is possible within the current state. If we determine that it is not possible (because the q_matrix returns -1), then we will choose another random action, until we find one that is possible. Once the random action is determined, we create a "RobotMoveObjectToTag" object, set the robot's action and tag values according to the random action that we selected, and then publish it. 


2. Updating the Q-Matrix

We update the Q-Matrix in two functions: "run_q_learning" and "update_q_matrix". In "update_q_matrix", we simply wait to receive the reward data from the Rospy Subscriber, where we then collect the reward data and set a flag to true in order to tell "run_q_learning" that there is a reward ready to be executed upon. It is in this latter function that the majority of the functionality is implemented. After we have ensured that we have retrieved a reward, we then retrieve the next staten given the current state and the action taken, ensuring that the action remains the same. We then check what the maximum reward in the next state is. We then calculate the Q-Learning equation that we learned in class. Then, we are storing the latest one thousand updates. We then check for convergence by ensuring that our training model has run for atleast 1000 iterations. If the average rate of change for the last 10000 ierations is less than our epsilon value (of 0.0001), then we choose to converge, change the invlaid q_matrices back to 0, and then write the QMatrix. 


3. Determining when to stop iterating through the Q-learning algorithm

In order to determine when to stop iterating through the Q-learning algorithm, 
we take advice from the following article: https://stats.stackexchange.com/questions/322933/q-learning-when-to-stop-training.
We implement the idea of taking the average performance of the algorithm for 
the previous 1000 episodes; this can be found in the QLearning class' 
run_q_learning() method. The algorithm runs for a minimum of 1000 time steps, 
and afterwards checks to see if the average of the 1000 most recent updates 
is less than epsilon - if so, the algorithm stops running. The 1000 most recent 
update values are held in a 1000-entry queue, in which old updates are popped 
from the start of the array and new ones are appended onto it.


4. Executing the path most likely to lead to receiving a reward after the 
Q-matrix has converged on the simulated Turtlebot3 robot

In order to execute each command most likely to receive a reward after convergence, 
we split our codebase into two scripts - state_controller.py and action_controller.py. 
To communicate through these two scripts, the StateController keeps track of the 
current state of the q_matrix. When the action controller wants an action, the 
state_controller determines the action that produces the highest reward by looking 
in the q_matrix for the current state, progressing to the next state through the 
action matrix, and communicating this state to the ActionController. The 
service's message parameters are defined in srv/StateService.srv; the state controller 
provides an action when the ActionController sends a ready status.


### Robot Perspective

1. Identifying the locations and identities of each of the colored objects

In order to identify the locations and identities of each of the colored objects, 
the action controller first maps the action received from the state controller 
to a color, and HSV high and low values are assigned to each color. The Robot 
object's navigate_to_object() method uses a mask to filter all values outside 
the HSV range, and if a mask exists the robot stops; otherwise, the robot spins 
indiscriminantly. Next, the robot aligns itself with the center of the filtered 
color using the camera feed again, and stops when this is acheived with a 10% 
image width tolerance; proportional control is used to determine angular speed. 

2. Identifying the locations and identities of each of the AR tags

Identifying the AR tags is performed in the Robot object's navigate_to_tag() 
method. Similar to identifying the locations and identities of the objects, 
this method begins by spinning indiscriminantly until the tag id provided by 
the object is in view. This is done using the cv2 aruco library, and if 
any tags are found each tag is checked to make sure it is the tag we are looking 
for.

### Robot Manipulation and Movement
#TODO - Describe how you accomplished each of the following components of the 
robot manipulation and movement elements of this project in 1-3 sentences, and 
also describe what functions / sections of the code executed each of these 
components (1-3 sentences per function / portion of code): 

1. Moving to the right spot in order to pick up a colored object

Continuing the locations and identities of the colored objects above, when the 
robot centers the object in its camera view, the robot then follows the closest 
object in front of it in a 10 degree cone through LiDAR laser scans. This is also 
performed in the Robot object's navigate_to_object() method, and implements an 
average of a 7 degree spread for each degree within the cone to account for noise. 
Slight proportional control is also provided in case the robot drifts away from the 
object by recentering the robot to have the 0 degree reading be the closest. Once
at a designated stopping distance, the robot stops.

2. Picking up the colored object

In order to make coding of the project slightly easier, we decided to divide the code
in action_controller.py into two objects: Robot and Arm. The arm object was created in 
order for us to be cognizant of possible conccurency issues between the robot's wheels
moving and the arm moving at the same time, essentially giving us a reminder queue to always
use sleep and rate commands to try and make each motion deliberate. Within the Arm class, we 
initialized the necessary nodes and move_it commanders within the initialization function, 
and then created three helper functions. "move_arm" would take in as parameters a list of 
joints that the robot arm should move to in degrees, convert those values to radians, and 
then move the arm to that desired position. "open_grip" would extend the gripper such that it
could pick up an object, and then "close_grip" would make the gripper narrower again such that 
it could pinch an object and lift it up. These helper functions made the rest of the arm control
extremely easy to implement, test, and tweak. 
The code that actually picks up the object is contained within the Robot class within the "pickup"
function. Here, we first make sure to open the gripper. Then, we move the arm first to a "straight" 
position where it can easily reach out and grab the dumbbell, while sleeping for a few seconds in order 
to ensure the arm is in the right position. We then move the robot forwards extremely slowly so that the
gripper is completely around the dumbbell. Afterwards, we close the gripper so the arm is now grabbing our
object, and we the raise the arm to a heightened position in the "lift" function to make sure the object is
not obscuring the camera!

3. Moving to the desired destination (AR tag) with the colored object

In order to move to the desired destination with the colored object, the 
Robot object's navigate_to_tag() method provides the behavior necessary to 
move the robot towards the goal. Using the camera feed, the robot uses proportional 
control to steer the center of the AR box into directly in front of the camera, and 
using LiDAR scans, the robot stops once at a set distance from the tag. The distance 
from the tag/wall is determined using the average distance of a 5 degree cone in 
front of the robot to account for noise.

4. Putting the colored object back down at the desired destination
Dropping an object is extremely similar to pickup, and used the same helper functions
from the Arm class that we discussed in that previous section. Here, we move the robot
arm from the lifted position to its original "straight" position, and then open the grip.
Initially, we had some issues where the gripper would open too soon while the arm was
still lowering, but a longer sleep session of 2 seconds ensured a smoother drop where 
the dumbbell was more stable and was much more likely to stay standing. 



### Challenges

The biggest challenges within this project related to transferring our work 
from the simulator into the real world. While in Gazebo, we could see the camera 
feed in real time, be guaranteed that the robot would always stop in front of 
objects at the correct distance with no overshoot in addition to lining up correctly, 
and lastly see objects as soon as they enter the camera, these were not present 
when testing with physical Turtlebots. Because other students could be testing 
at the same time, the robot could perceive the center of a colored object to 
the average location of two colored objects if another group's happened to be 
in view. As a matter of fact, RD's shoes were even mistaken for AR tags due to 
black and white patterns. This was a quick fix performed by using different 
classroom objects as barricades such that no other objects would attract our 
Turtlebot and vice versa. In order to account for network lag, we decided to make 
our robot slower in general so that it had more time to process new image callbacks 
and so that it would not overshoot alignment in terms of angle and distance to 
objects and tags; if this happened, then objects would be missed overall or the 
arm planner would determine that an object could not be placed at all. Lastly, 
one of the largest physical Turtlebot challenges related to working with the 
OpenManipulator. Clock desynchronization has been a problem across multiple 
projects already, but the OpenManipulator was very sensitive to state changes. 
This was solved by syncing the clocks on the Pi and the computer running the 
code before trials. Otherwise, sometimes when we sent commands to the arm, it simply wouldn't move.


### Future Work

In the future, we think that it would be great if we could use the OpenCV library to not only 
identify the color, but also the shape of the dumbbell such that it is much more accurate.
Every once in a while, we were faced with the issue that the turtlebot's camera would find some of
the blue or green colored bricks at a distance and go in that direction instead of the actual object
we wanted it to pickup, which was sometimes frustrating! We think being able to train and classify the
dumbbell and make the turtlebot actually recognize the item would be a good way to get rid of this problem.
We also think that it would be really interesting if we could add some sort of spatial tracking ability in the 
future, where the robot could localize where it picked up a specific dumbbell, and then move it back to that 
location after delivering it to the tag. That sort of object permanence on a robot would be a cool feature, we 
think. 


### Takeaways

1. Be aware of the tradeoffs of each sensor on the Turtlebot. The camera is a 
great way to align objects because segmentation guarantees that you'll be looking 
for the right object, but it very much comes at the cost of runtime. Switching to 
LiDAR whenever possible makes the robot more responsive, and a combination of the 
two can be used in a way that the camera sets an object in frame such that the 
LiDAR becomes less suspectible to noise and other nearby objects.

2. Lag is always going to be present, so thinking around how to make control 
as robust as possible is a must. This can mean slowing down the robot to make 
sure that significant overshooting occurs less frequently and ensuring that the 
robot is still able to achieve baseline behavior in these instances. For example, 
even when the robot goes to pick up an object and accidentally pushes into it 
with the claw, the robot moves so slowly such that the object becomes just shy of 
tipping over and can still be picked up and put down without any real issues.
