###Computational Robotics 2017 WarmUp Project
In this project we wrote code on a Neatos vacuum cleaner equipped with a Ras-Pi to make it do various tasks. These includes stopping when it bumps into something, driving in a square of custom size, approaching a wall and following it, being a pet (following a person in front of it) and avoiding obstacles while going to a goal position. In the end, we combined several behaviors using a finite state controller so that the robot can be smart and run for a long period of time autonomously. Although we used the finite state controller to combine person follow, obstacle avoidance and emergency stopping, it can be used to combine any set of behaviors we wrote. The goal of this project is to get familiar with ROS and the neatos hardware, and learn basic robot navigation techniques and algorithms. 

####Drive Square:
Behavior: Drive the neatos in a square of 1m by 1m.
We used the Odometry of the neatos to determine it's orientation and position. Because we did not have a function to convert the Odometry's quaternions to Euler angles, we recorded four quartonion values that correspond to four orientations at right angles. Then, we rotated our robot until it reached the first orientation, then we made it go forward 1m, then rotate it to the second orientation, and so on. This strategy worked well as long as we gave an error range for the orientation values and turned the robot slowly (angular z = 0.3). The robot was able to drive in a square and return to its original position.

####Wall follow:
Behavior: Approach a wall and turn to follow it.
We implemented this behavior in two parts: first, find the wall; Second, follow the wall. To find the wall we used a proportional controller that moved the robot to a certain distance from the wall and then stopped. To follow the wall we used a proportional controller using the difference between the closest obstacle directly to the left of the robot and our target distance. Once the robot found the wall, it sees nothing on its left-hand side, so it turns until it sees the wall on its left-hand side. Then it goes forward, adjusting so it keeps the same distance to the wall. This approach worked very well and the robot can follow the wall in almost a straight line. We did not test corners. In the beginning we also tried to control using the angle with the wall but that did not work well.

####Person follow:
Behavior: If there is a person within a cone area in front of the robot, it will follow the person at a certain distance, keeping the person directly in front of it. 
The robot looks at all the laser scan distances in a 90-degree cone area in front of it. It determines the closest distance and its angle. If the closest distance is within a certain range, it is identified as a person. The laser scans around this closest distance is looked at. If the points around it have a similarly short distance (within 0.1 of the closest distance), they are also part of the person. When we have a collection of all the points of the person, we find the center of mass of those points. Then, we use two proportional controls: one on angle and one on distance to keep this center of mass directly in front of the robot and at a fixed distance. This works well generally but the robot travels slowly and so it is only able to follow the person if he/she walks slowly.

![Person Following diagram](https://github.com/xiaozhengxu/warmup_project_2017/blob/master/Person%20Following.png)

####Obstacle Avoidance:
Behavior: Move towards the goal position, and if there is an obstacle within a certain distance of the path, change directions to avoid/get around the obstacle.
The robot determines its position relative to where it started using odometry and converts that from quarternions into x and y coordinates as well as its direction in degrees. If there are no values for the x and y initial positions, they are set as the current x and y positions. The current x and y positions are then subtracted from the intiial x and y positions to find the existing change in x and y. The robot then looks at the laser scan distances all around it. If any of the distances are less than two meters, the robot will use trigonometry to alter change and x and y enough to move out of the way of the obstacle. It uses then the arctangent of change in x and change in y to find the new change in the angle of the robot's direction. We use a proportional controls for the angle to determine the speed that the robot turns to avoid obstacles. The robot moves forward at a set rate. This means that if the angle constant is too low the robot will not have enough time to move out completely out of the obstacles' way.


####Finite State Controller

![Finite State Diagram] (https://github.com/xiaozhengxu/warmup_project_2017/blob/master/Comp-robo-warmup-finite-state-diagram.png)

Behavior: The robot avoids objects unless it has found a person to follow. If the bump sensor is triggered, the robot stops.
States: 
- OBSTACLE_AVOID_STATE: runs an obstacle_avoid function that unless the robot if bumped or has found a master, uses the angle proportional control and change in angle to avoid objects (does what Obstacle Avoidance does)
- PERSON_FOLLOW_STATE: runs a person_follow function that unless the robot is bumped or no longer has a master, finds the center of mass and uses the angle and distance proprotional controls to keep the center of mass in front at a fixed distance (does what Person Following does)
- STOPPED_STATE: if the robot bumped, it runs a stop function 
To combine person following with object avoidance and add in an emergency bump stop, we subscribed to laser scan, bump, and odometry and had functions to process inputs from each of them. We included the process_odom function we used in object avoidance to determine where the robot was in relation to its starting point using odometry. We combined the contents from the process_scan functions from each program into one function that process all the laser scan data needed to run either obstacle_avoid or person_follow function depending on the state of the robot. This function also determines whether or not the robot has found a master which helps determine the state of the robot. We included a process_bump function that determines whether or not the bump sensor is triggered and also publishes the current state of the robot. Our run function runs the function corresponding to the robot's current state. These functions, described above in States, change the robot's state depending on whether or not the robot finds its master or triggers the bump sensor. 

####Code Structure
We created classes for each of our behaviors. The _init_ functions initialized the defined the rosnode where the program would run, what inputs from the sensors of the robot it subscribed to (e.g. LaserScan, Odometry), where it would publish values from its functions to, and what property variables of our classes we want to initialize or have inputed as user arguments. Within the class we created functions that used the message data coming in from each of our subscribed inputs to publish messages to programs that we were using to debug the robot (e.g. rviz). They then include a run function that determines what messages are published to the robot to control its movement while it is running to complete its task. 

####Challenges we faced along the way 
It was difficult to find places outside of the CompRobo classroom with good enough signal strength to test code on the robot. The lack of open spaces we could use to move the robot around was frustrating. We ran into a lot of errors when running our programs. The laser scan wasn't always consistent which can be attributed to hardware but was also annoying. One of the most common and confusing was the "publishing to a closed topic" error which came form the VisualizationMarker topic. We had difficult maintaining simple, modular, readable code. The more we wrote, the harder it was to read; although, some of our later programs were more readable than our initial ones.


####Future Improvements 
- We could learn SMACH and use it for finite state control. Alternatively to reduce copied code in finite state controller, we can import classes from each behavior script. 
- We could improve the obstacle avoidance behavior to change the robot's linear speed based on the distance to objects in front of it. 
- We cound improve person follow to not be intefered by walls and detect legs. (Right now it only detects a person if he/she stands feet together)
- We could improve drive square by converting quartonions to Euler angles so the robot don't have to rotate to a specific position before starting the square.


####Key takeaways and what we learnt
1. Add an emergency bump stop to any program so the robot stops if it hits an obstacle.
2. Print out useful information when running your program.
3. Use visualizations to help you. 
4. Try simple methods first, if it works, don't make it complicated. Keep code as simple as possible. 
5. When necessary, lift the robot. Be careful not to let it ruin cables. 

Judy Xu and Katie Butler,
Feb 16, 2017
