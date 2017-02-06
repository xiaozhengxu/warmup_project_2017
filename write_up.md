Writeup: in your ROS package create a file to hold your project writeup. Any format is fine (markdown, word, pdf, etc.). Your writeup should answer the following questions.  I expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio.  As such, please make sure to write the report so that it is understandable to an external audience.  Consider adding pictures to your report, or links to youtube videos of your robot programs in action.  You have no idea how persuasive a project like this can be to a perspective employer!

####For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

####Drive Square:
Behavior: Drive the neatos in a square of 1m by 1m.
We used the Odometry of the neatos to determine it's orientation and position. Because we did not have a function to convert the Odometry's quartonions to Euler angles, we recorded four quartonion values that correspond to four orientations at right traingles. Then, we rotated our robot until it reached the first orientation, then we made it go forward 1m, then rotate it to the second orientation, and so on. This strategy worked well as long as we gave an error range for the orientation values and turned the robot slowly (angular z = 0.3). The robot was able to drive in a square and return to its original position.

####Wall follow:
Behavior: Approach a wall and turn to follow it.
We implemented this behavior in two parts: first, find the wall; Second, follow the wall. To find the wall we used a proportional controller that moved the robot to a certain distance from the wall and then stopped. To follow the wall we used a proportional controller using the difference between the closest obstacle directly to the left of the robot and our target distance. Once the robot found the wall, it sees nothing on its left-hand side, so it turns until it sees the wall on its left-hand side. Then it goes forward, adjusting so it keeps the same distance to the wall. This approach worked very well and the robot can follow the wall in almost a straight line. We did not test corners. In the beginning we also tried to control using the angle with the wall but that did not work well.

####Person follow:
Behavior: If there is a person within a cone area in front of the robot, it will follow the person at a certain distance, keeping the person directly in front of it. 
The robot looks at all the laser scan distances in a 90-degree cone area in front of it. It determines the closest distance and its angle. If the closest distance is within a certain range, it is identified as a person. The laser scans around this closest distance is looked at. If the points around it have a similarly short distance (within 0.1 of the closest distance), they are also part of the person. When we have a collection of all the points of the person, we find the center of mass of those points. Then, we use two proportional controls: one on angle and one on distance to keep this center of mass directly in front of the robot and at a fixed distance. This works well generally but the robot travels slowly and so it is only able to follow the person if he/she walks slowly.


####Obstacle Avoidance:
Behavior: Move towards the goal position, and if there is an obstacle within a certain distance of the path, change directions to avoid/get around the obstacle.


####For the finite state controller, what was the overall behavior. What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.

####How was your code structured?  Make sure to include a sufficient detail about the object-oriented structure you used for your project.
We created classes for each of our behaviors. The _init_ functions initialized the defined the rosnode where the program would run, what inputs from the sensors of the robot it subscribed to (e.g. LaserScan, Odometry), where it would publish values from its functions to, and what property variables of our classes we want to initialize or have inputed as user arguments. Within the class we created functions that used the message data coming in from each of our subscribed inputs to publish messages to programs that we were using to debug the robot (e.g. rviz). They then include a run function that determines what messages are published to the robot to control its movement while it is running to complete its task. 

####What if any challenges did you face along the way? 

####What would you do to improve your project if you had more time? 
- We could learn SMACH and use it for finite state control.
- We could improve the obstacle avoidance behavior to change the robot's linear speed based on the distance to objects in front of it. 
- We cound improve person follow to not be intefered by walls and detect legs. (Right now it only detects a person if he/she stands feet together)
- We could improve drive square by converting quartonions to Euler angles so the robot don't have to rotate to a specific position before starting the square.


####What are the key takeaways from this assignment for future robotic programming projects?
1. Add an emergency bump stop to any program so the robot stops if it hits an obstacle. We implemented this by running a separate script while running another program. However, we still need to stop the other program manually when the robot hits something.
2. Print out useful information when running your program.
3. Use visualizations to help you. 
4. When necessary, lift the robot. Be careful not to let it ruin cables. 
