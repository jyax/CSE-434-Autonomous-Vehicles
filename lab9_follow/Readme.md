# Lab 9: Line Following
## ECE-CSE 434

Make sure to complete Lab 6 by Nov 2nd.  Since Lab 9 is a bit involved, we'll delay Lab 7 another week.

------
Lab 9 builds on Lab 8.  In Lab 8 you built a ROS node to detect the image pixel coordinates where the green line intersects the bottom of the image.  This is published to a topic `/line_point`.  A second node subscribes to this topic and publishes `/ground_point`, the ground location of this point.  In Lab 9, your assignment it so create two different line followers that will enable your Turtlebot to follow the green line.  Then you can optionally run the code on the real Turtlebots.

To set up your environment on HPCC, start by running the Greenline 2 World:
```
ros2 launch greenline2 greenline2.launch.py
```
Also run your Lab 8 nodes that publish `/line_point`:
```
ros2 run line_detect detect
```
And `/ground_point`:
```
ros2 run line_detect ground_spot
```

Create a ROS Python package called `line_follow` in your `<student_repo>/lab9_follow` folder.  All your nodes in this lab should be added to this package.  

# Exercise 1

Create a ROS node that follows the green line using Pure Pursuit.  Your node should be called with:
```
ros2 run line_follow pure_pursuit --speed <val>
```
Here `--speed <val>` is an optional linear speed at which the Turtlebot should travel.  (See the `pid.py` code for an example of arguments for Python code.)  If this argument is not provided, the your Turtlebot should travel at a default speed.  Your code should print out the speed it is running at when it starts.

If the line is lost from view, then your robot should stop, rotate until it recovers the line, and then proceed. 

Hint: your code should subscribe to `/ground_point` and for each callback calculate an arc that takes the robot through this point.  Then convert this into a Twist that you will publish to `/cmd_vel`.

# Exercise 2
Create a ROS node that follows the green line using a PID controller.  Your node should be called with:
```
ros2 run line_follow pid_run --speed <val>
```
Here `--speed <val>` is an optional linear speed at which the Turtlebot should travel.  If it is not provided, the your Turtlebot should travel at a default speed.  Your code should print out the speed it is running at when it starts.

If the line is lost from view, then your robot should stop, rotate until it recovers the line, and then proceed. 

Your controller could subscribe to either `/line_point` or `/ground_point`.  You are free to choose whatever error function you think is most suitable for following the line.  Also, feel free to use the `pid.py` code from `AV_23`.  

# Exercise 3

Modify the code from Lab 8 to work on the real Turtlebots, so that your Turtlebot can follow a green line.  Whichever teams have this working, can demonstrate their bots doing line following during lab on Thursday Nov 2nd. We'll use green masking tape to make a course.

It's not too complicated, and will involve the following:
- Copy `detect.py` and `ground_spot.py` from Lab 8 to Lab 9 and rename them `detect4.py` and `ground_spot4.py`.  
- Change the camera topic and camera info topic names to use the OAK-D preview image and its info topic.
- Extract an uncompressed image instead of compressed image, see AV_23, Section 3.4 for examples.
- Adjust the image height.  
- The color detection may work as is, or may need adjustments (retraining on an actual image of the tape).  There will be green masking tape stored near the Turtlebots for use.  Please don't remove the tape rolls from the room.
- The PID control may work as-is or could be tuned.


# Submit

As usual, submit your ROS package `line_follow` containing all the necessary code. 



