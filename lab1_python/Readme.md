# Lab 1: Repository Setup and Python Familiarity

___
## Contents

* [Python: What is expected for this course](#python-what-is-expected-for-this-course)
* [Setup](#Setup)
* [Assignment](#Assignment)
* [Submitting this lab](#Submitting-this-lab)

# Python: What is expected for this course

This class will involve programming robots to perform complex tasks, and so inevitably will require programming skills. Fortunately, advanced tools (namely ROS) and modern languages (such as Python) make this fairly straight forward. All the teaching examples will use Python, and it is expected that you will can follow and program in Python yourself.

This lab assesses Chapters 1 to 16 of [Python Introduction Notes](https://github.com/dmorris0/python_intro/blob/main/README.md).  Its goal is to ensure that you are familiar with the basics of Python so that you can follow lecture examples, and that you have a working Python environment.  You can use the lab Windows computers or your own laptop.  

This assignment should not be hard, although if you are new to Python it may entail a fair bit of effort to get it working properly.  Feel free to use online documentation and web searches, but **do not copy code from other people in the class**.  If you are unable to get a high score in this assignment, then you are strongly recommended to take a programming class before taking this course. 

# Setup

1. Create a your own respository on Gitlab as explained in the [labs_23/README.md](../README.md) and give permissions to the instructors and clone it onto your computer.  I'll refer to this as `<student_repo>`
2. Clone the the lab assignment repo [https://gitlab.msu.edu/labs/labs_23](https://gitlab.msu.edu/labs/labs_23) onto your computer.
3. Copy the `lab1_python` folder and its contents into your personal `<student_repo>`.  

You will now have a your assignment in your submission repo.

# Assignment

This lab comes with 
* 2 python files: [src\lab1_review.py](src/lab1_review.py) and [src\lab1_student_score.py](src/lab1_score.py)
* A sample image: [data\tail_lights.jpg](data/tail_lights.jpg)
* Ground truth results in folder [gt](gt)

The functions in `lab1_review.py` are incomplete, but their inputs and outputs are specified.  Your task is to complete all of the functions in this file accoring to the specifications in each function's documentation.  

The `lab1_student_score.py` function is provided as a means for you to score your work.  You can run it by first `cd` to the lab1_python folder:
```
cd lab1_python
```
And then run it with:
```python
# Install useful libraries:
python -m pip install numpy scipy opencv-python

# This command for running the evaluation script
python src\lab1_student_score.py
```
Or else run it directly in Visual Studio Code, which enable you to step through the code and figure out whey your code isn't giving you the expected results.

If you run this without making any changes to `lab1_review.py`, you should get a series of test failures that end with:
```
----------------------------------------------------------------------
Ran 20 tests in 0.086s

FAILED (failures=12, errors=8)
```
Also, you should see the raw color image with the back of the truck in a window.  Close this to end your test.

This scoring unit test will tell you how many errors or failures there are in your `lab1_review.py` code.  Each function you get correct will reduce either the failures or errors by 1.  When there are no failures or errors reported, then likely you have got everything right -- I say likely because those are not exhaustive tests, and the grader may use different tests.  But if your functions follow the directions then you should be good.  Here is the output I got when I got all the functions working:
```
....................
----------------------------------------------------------------------
Ran 20 tests in 0.159s

OK
```

Additionally, when you have completed everything correctly, you should see three windows like this showing the detected tail lights overlaid with rectangles:
<p align="left">
<img src="data/tail_lights.png" width="300">
<img src="gt/tail_lights_mask.png" width="300">
<img src="gt/tail_lights_rectangles.png" width="300">
</p>

## Debugging
I recommend that you test the VSCode debugger on this assignment. 

# Submitting this lab
Your code should all be inside the `<student_repo>/lab1_python` folder.  To submit do:
```
cd <student_repo>/lab1_python
git add .
git commit -m "Add a comment here"
git push
```
Do not forget to `push`, or your lab will not be submitted.

# Due Date:

Since this is lab 1, you will have 2 weeks to complete it.  The remaining labs will be due 1 week after submission.  This lab is due **3pm EDT Thursday September 14th, 2023**.
