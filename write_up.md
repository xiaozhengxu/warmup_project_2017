Writeup: in your ROS package create a file to hold your project writeup. Any format is fine (markdown, word, pdf, etc.). Your writeup should answer the following questions.  I expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio.  As such, please make sure to write the report so that it is understandable to an external audience.  Consider adding pictures to your report, or links to youtube videos of your robot programs in action.  You have no idea how persuasive a project like this can be to a perspective employer!

###For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

####Drive Square:
Behavior: Drive the neatos in a square of 1m by 1m.
We used the Odometry of the neatos to determine it's orientation and position. Because we did not have a function to convert the Odometry's quartonions to Euler angles, we recorded four quartonion values that correspond to four orientations at right traingles. Then, we rotated our robot until it reached the first orientation, then we made it go forward 1m, then rotate it to the second orientation, and so on. This strategy worked well as long as we gave an error range for the orientation values and turned the robot slowly (angular z = 0.3). The robot was able to drive in a square and return to its original position.

####Wall follow:
Behavior: Approach a wall and turn to follow it.
We implemented this behavior in two parts: first, find the wall; Second, follow the wall. To find the wall we used a proportional controller that moved the robot to a certain distance from the wall and then stopped. To follow the wall we used a proportional controller using the difference between the closest obstacle directly to the left of the robot and our target distance. Once the robot found the wall, it sees nothing on its left-hand side, so it turns until it sees the wall on its left-hand side. Then it goes forward, adjusting so it keeps the same distance to the wall. This approach worked very well and the robot can follow the wall in almost a straight line. We did not test corners. In the beginning we also tried to control using the angle with the wall but that did not work well.

####Person follow:
Behavior: If there is a person within a cone area in front of the robot, it will follow the person at a certain distance, keeping the person directly in front of it. 



####Obstacle Avoidance:


For the finite state controller, what was the overall behavior. What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.

How was your code structured?  Make sure to include a sufficient detail about the object-oriented structure you used for your project.

What if any challenges did you face along the way? 

What would you do to improve your project if you had more time? 

What are the key takeaways from this assignment for future robotic programming projects?

