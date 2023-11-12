# Lab 10: Turtlebot Camera
## ECE-CSE 434

Create a ROS Python package called `tbot_cam` in your `<student_repo>/lab10_cam` folder.  All your nodes in this lab should be added to this package.  

# Exercise 1 (Previously Lab 9 Exercise 3)

Modify the code from Lab 8 and Lab 9 to work on the real Turtlebots, so that your Turtlebot can follow a green line.  It is recommended that you adjust your line detection code using ROS bags.  Once you are able to reliably detect lines in ROS bags, then switch to the real robot and adjust your pure pursuit and PID controller.  

ROS bags are on HPCC.  Create a symbolic link to the folder containing the bag files:
```
cd ~/av
ln -s /mnt/research/ece_cse_434/greenlines
```
See the bag files with:
```
ls greenlines/
```
Then play one of the ROS bags in your greenlines folder with:
```
ros2 bag play --loop -r 0.5 greenlines/bag_line_1
```
Here `--loop` causes the playback to repeatedly loop, `-r 0.5` sets playback rate to half, and `bag_line_1` is one of the recorded ROS bags.  You can view the bag with:
```
rqt
```
Select Plugins / Visualization / Image View, and then choose the topic `/oakd/rgb/preview/image_raw` and press refresh.

Adjusting your code to work on the Turtlebot is not too complicated, and will involve the following:
- Start by copying code from Lab 9 to Lab 10.  
- Change the camera topic and camera info topic names to use the OAK-D preview image and its info topic.
- Extract an uncompressed image instead of compressed image, see AV_23, Section 3.4 for examples.
- Adjust the image height.  
- The color detection will likely need to be retrained on an actual image of the tape.  There will be green masking tape stored near the Turtlebots for use.  Please don't remove the tape rolls from the room.
- Note that the lighting can affect the green color and in certain lighting it can saturate
- The PID control may work as-is or could be tuned.

Whichever teams have this working, can demonstrate their bots doing line following during lab on Thursday Nov 9th. We'll use green masking tape to make a course for the Turtlebot to drive along.

# Exercise 2: Sign Detection

In this exercise you will integrate a deep convolutional neural network for doing traffic sign detection onto your Turtlebot.  YOLOv5 has been trained on 4 types of traffic signs in this dataset: https://www.kaggle.com/datasets/andrewmvd/road-sign-detection/, and sample results are shown in the `trained` folder.  The goal is to use this trained network to detect signs visible from the Turtlebot camera.  

Start by trying out the provided code that loads the pre-trained model, applies it to an image and displays the output. First clone the YOLOv5 repo to your ROS workspace `src` folder:
```
git clone https://github.com/ultralytics/yolov5
```
Create a virtual environment called `signs` and activate it.   On HPCC do:
```
cd ~/av/venvs
python3 -m venv signs
act signs
```
On the `turtlebot-control-##` workstation, this space will be in your M Drive, and since it is large it may exceed your quota.  To save space you can put it in `/tmp` and create a symbolic link to it as follows:
```
cd /tmp
python3 -m venv signs
cd ~/av/venvs
ln -s /tmp/signs
```
Then `cd` to the `yolov5` folder and install the required packages with:
```
python -m pip install -r requirements.txt
```
Copy the `code` folder into `lab10_cam` folder. Then from this folder run the demo:
```
python detect_signs.py
```
This should load an image and the pre-trained model, apply the model and display the result. Press any key over the window to quit.

(a) Create a ROS node called `signs` in your `tbot_cam` package that performs sign detection.  The node should subscribe to the Turtlebot `/oakd/rgb/preview/image_raw` topic, do sign detection on each image, and display the results in a window.  Feel free to extract the relevant portions of the demo code.  Note, that you will want to load the model weights **only once** (not every time you get an image).  The output should be a window that shows live video with bounding boxes on detected signs.

Use HPCC for this and test it out on a ROS bag playback, such as:
```
ros2 bag play --loop -r 0.5 greenlines/bag_line_2
```
Your code should run with:
```
ros2 run tbot_cam signs
```

(b) Transition your code to your `control-workstation`. You will need to repeat some of the install steps from (a) for this.  Test that you can run your code live while your Turtlebot is navigating.  If you wish you could have your turtlebot follow a green line while it is detecting signs.  There are model signs available by the Turtlebots, although the network is only trained on 4 kinds of signs.

Submit a screen capture called `sign_success.png` of it successfully detecting a sign.

Note: do **NOT** add the `signs_256.pt` file to your repo.  This is 14GB and will majorly slow down pushing and pulling.  It is available on the HPCC and will be made available on the control workstations. 

# Submit

As usual, submit your ROS package `tbot_cam` containing all the necessary code, plus the screen capture: `sign_success.png`.



