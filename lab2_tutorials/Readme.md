# Lab 2: ROS Setup and Tutorials

The goal of this lab is to configure your ROS 2 environment on the HPCC, and to explore basic ROS functionality.  It is intended as a low-effort lab which you can mostly finish during the lab session.  It will be due in one week at the same time as Lab 1.  

## HPCC Environment

First follow setup instructions in [AV_23/Setup/HPCC](https://gitlab.msu.edu/av/av_23/-/blob/main/Setup/HPCC.md).  Start up an On-Demand interactive environment and make sure you can open a terminal window.  Also, configure your VSCode so that you can open files in your HPCC environment.  You can skip the Python in CentOS instructions, as future labs will use Python in Ubuntu.

## ROS 2 Install

Next, follow the instructions in [AV_23/Setup/HPCC_ROS](https://gitlab.msu.edu/av/av_23/-/blob/main/Setup/HPCC_ROS.md).  This will take you through the steps for pulling an Ubuntu/ROS Docker image and how to configure and run it.  It will also get you started with Python in Ubuntu including installing a virtual environment.


# ROS Client Tools Tutorials

Next do nine of the [Beginner CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) tutorials; all except the first tutorial "Configuring environment", as you just configured your ROS environment.  Of course you will do these in your Ubuntu/ROS environment.

All the packages should be pre-installed, so there is no need to do any `sudo apt install` commands, as shown in some of the tutorials.

While it is possible to copy and paste the ROS commands, do not do this.  Instead, **type out each ROS command** in your terminal.  That will do much more to make you comfortable working with ROS than pasting commands.  

# Submission

Do the following submission after you have completed all nine tutorials.  Make sure you have cloned your `<student_repo>` onto your M drive, as described in the [labs_23/Readme.md](https://gitlab.msu.edu/labs/labs_23/-/blob/main/README.md).

Create a folder inside your `<student_repo>` called `lab2_tutorials` and do the following steps: 
1. Start a fresh `turtlesim_node` simulation
2. Start recording a rosbag called `figure_eight.bag` that records only the topic `/turtle1/cmd_vel`. 
3. Navigate the turtle around in a rough figure eight shape using teleoperation 
4. Close the rosbag  
5. Make a screen capture of the turtle window called `figure_eight.png` which should look approximately like this:

![figure_eight](.Images/figure_eight.png)

6. Restart your `turtlesim_node` 
7. Play the rosbag you just recorded.  
8. When it is done, make a screen capture of the replayed turtle path called `figure_eight_replay.png`.

Note: to do a screen capture of a window, press `Windows-Alt-Print Screen`.  This will save the current window as an image to your `Documents\Videos\Captures` folder.  Alternatively, you can use `Windows-Shift-S` to capture a window or region, and then save it.

## What to submit:
- `figure_eight.bag`: the rosbag you recorded.  Normally one would not add a rosbag to a git repo as they are usually huge, but here the rosbag will be quite small so in this case it is okay to add the rosbag.
- `figure_eight.png`: the screen capture of the first figure eight run
- `figure_eight_replay.png`: the screen capture of a replay of the rosbag controlling the turtle.

When these 3 files are inside your `lab2_tutorials` folder, submit it as follows:
```
cd <student_repo>\lab2_tutorials
git add .
git commit -m "Add a comment here"
git push
```
Do not forget to `push`, or your lab will not be submitted.

# Due Date:

This lab is due **3pm EDT Thursday September 14th, 2023**.  This is the same time as lab 1 is due. Note this is probably the easiest lab and so a good way to accumulate points.
