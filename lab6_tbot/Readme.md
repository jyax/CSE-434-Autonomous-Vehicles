# Lab Assignment 6 - Turtlebot Navigation
## ECE-CSE 434 - Automated Vehicles

# Prerequisites

Before starting this lab:
* Join a team.  You'll be given a team ID
* Collect your Turtlebot
* Follow instructions on setting up your Turtlebot and laptop: 
 **[AV 23 / 3.5 Turtlebots](https://gitlab.msu.edu/av/av_23/-/blob/main/ROS/Turtlebot4.md)**


## Assignment Preparation

You will need to create a ROS workspace on your Ubuntu instance on your laptop.  Use the same file organization as on HPCC.  As explained in Lab 3, clone your `<student_repo>` into the `src` folder of your ROS workspace.  Cd to this folder and then create your `lab6_tbot` folder as follows:
```
R:~/av/ros_ws/src/<student_repo>$ mkdir lab6_tbot
R:~/av/ros_ws/src/<student_repo>$ cd lab6_tbot
```
As usual, replace `<student_repo>` with the name of your individual repo in all commands.


# Exercise 1: Circle Drive

For this exercise you will create a package called `bot_move` in your `lab6_tbot` folder.  (Note: this have to be a different package name than `bot_drive` from lab 4).   Create the package with:
```
R:~/av/ros_ws/src/<student_repo>/lab6_tbot$ ros2 pkg create --build-type ament_python bot_move
```

This exercise will be an enhanced circle drive node called `circle_closed`.  If you recall from Lab 4, your circle drive often did not do a complete circle.  The problem there was that commands were all open-loop.  Here the task is to create a closed-loop circle drive node that monitors its progress around the circle and stops when it has exactly completed one loop.  

To do closed-loop control, have your node subscribe to `/odom` and make adjustments to the trajectory based on this including stopping when done.  Also recall that to keep the Turtlebot moving, you will need to keep publishing to `/cmd_vel`.  One way to achieve this is to have the publishing done from within the subscriber callback.

Update the `setup.py` configuration file and then build your code with:
```
R:~/av/ros_ws$ colcon build --symlink-install --packages-select bot_move
```
You will also need to source the overlay whenever you first build a new node using:
```
R:~/av/ros_ws$ source install/setup.bash
```
When you have completed it, you should be able to run your node with:
```
R:~$ ros2 run bot_move circle_drive
```
Always make sure your code runs with the exact command specified in the exercise, as the grader will use that command in grading your assignment.

## Note on Quality of Service (QoS)
If you create a subscription to `/odom` as was done in Lab 4, you will likely get this warning:
```
[WARN] [1696688665.626791128] [circle_drive]: New publisher discovered on topic '/odom', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY
```
This has to do with QoS requirements, explained [here](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html).  The topic `/odom` is published with `Best effort` setting, which allows for dropped messages, but the default subscription is `Reliable`, which will guarantee each published message is read.  The subscriber cannot be `Reliable` if the publisher is `Best effort`, hence the incompatibility.  The solution is to set the subscriber to `Best effort`.  You can do this as follows.  At the top of your new `circle_drive.py` file, import these:
```
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
```
Then, in your Node init function, where you define your subscriber, use this code:
```
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.callback,
            qos_profile=qos_profile)
```
Here we pass in a `Best effort` QoS policy and set the queue size to 1 (depth=1).  This replaces the queue argument -- do not pass in a separate queue size.  Now your code should be able to successfully subscribe to `/odom`.


# Exercise 2: Actions

Now behaviors of the turtlebot can be encapsulated as actions.  There are a few pre-configured actions including `DriveDistance`, `DriveArc`, and `RotateArc`.  Find details how to command these actions here:  https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html.  Note: you will need to look up the action definition to determine the parameters it needs from the command line.

Try out these commands.  What does the robot do if you stand in its way before it reaches the target distance?  Try it.

In this exercise you will learn how to define an action and how to create an action server and client.  

**(a)** Complete the ROS 2 tutorial for defining and creating an action. https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html.  Make sure this package is located within your `lab6_tbot` folder (not at the top level of ros_ws/src)

**(b)** Complete the ROS 2 tutorial for creating an action server and client.  https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html. 


**(c)** Now that you know how to create action clients, write an action client that commands the Turtlebot with the `DriveArc` action to complete a full circle.  This client node should be called `circle_action` and be part of the `bot_move` package from Exercise 1.


# Submitting this lab
Your code and should all be inside the `<student_repo>/lab6_tbot` folder.  To submit do:
```
cd <student_repo>/lab6_tbot
git add .
git commit -m "Add a comment here"
git push
```
Do not forget to `push`, or your lab will not be submitted.



