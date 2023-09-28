# Lab Assignment 5 – Color Target Detection
## ECE-CSE 434 - Autonomous Vehicles


# Introduction

The `scikit` toolbox will be used for logistic regression.  Install this in your Python virtual environment as follows:
```bash
R:~$ act av
(av) R:~$ python -m pip install scikit-learn
```
Start by creating a `lab5_logr` folder in your `<student_repo>`.  Then copy the `src` and `data` folders and their contents into your `lab5_logr` folder.  Run all the code from within the `src` folder.

# Exercise 1: Exploring Higher Dimensions for Linear Classifiers

Logistic regression provides a powerful way to train a linear classifier.  In some situations this finds a solution quickly and reliably. The result is a classifier that can separate targets from clutter.  However, sometimes the measurement data of targets and clutter are not linearly separable.  In these cases logistic regression will be unable to produce a clean separating hyperplane.  Now one way to address this problem is to expand the dimensionality of the data, either by measuring additional data or simply by transforming the already measured data.  The goal is to make the target points linearly separable from the clutter points.  This exercise explores the latter approach: transforming measured data and adding it as additional dimensions so that the targets can be identified with a linear classifier.

Two datasets are provided, each with training and testing data in the `data` folder.  In addition, code for reading the data, plotting the data and running a logistic regression classifier on it are provided.  Start with the `set_1` dataset.  The script `test_set_1.py` shows how to read in the data, plot the points, fit a classifier to the training data, apply the classifier to the test data, and plot the results.  Try it out, and make sure you understand each step of the process.

Now look at the contents of the data class (either `train` or `test`) that were loaded.  Notice that the data are organized in N rows, with each row having 2 channels and also having a corresponding label which indicates whether it is a target or clutter point.  We can think of the 2 channels as two dimensions, and so we can plot the points on a two-dimensional plot.  

In this dataset, the points are very nearly linearly separable and so our logistic regression classifier does a good job separating target from clutter.  Notice that the precision and recall values are both high (>90%), and the Average Precision is over 99%.

**(a)** Now use the logistic regression classifier on the `set_2` dataset.  You'll need to train it on `set_2_train.csv` and report results on `set_2_test.csv`.  You should get roughly 57% Average Precision.  Output the classifier results to an image named `set_2.png` by adding the argument `filesave='../set_2.png'` to the classifier `plot_results()` function.  

* For this part, submit this `set_2.png` in the top folder of your `lab5_logr` folder.

**(b)** Your main assignment is to add two channels to the data that enable you to obtain over 95% Average Precision (and over 90% precision and 90% recall).  An example of adding channels is in the `add_data_channels.py` file.  Notice that the channels you add must be functions of the first two channels, `x` and `y`.  In that example, the added channels are linear functions of `x` and `y`, and do not help the overall classifier.  To improve classification you'll need to add non-linear functions of `x` and `y`.  For example, `x*x` or `(x-2)*y` etc.  Think about how you might transform the data to make two channels where the target points are linearly separable from the clutter points.  Create a new function in `add_data_channels.py` that adds your own two channels. When you try out various ideas, both the `plot_all_points()` and `plot_results()` functions will show your extra two channels as a separate 2D plot.  (Ideally we would have a 4D plot, but that is hard to visualize, so you'll get just two 2D plots.)  Experiment with various transformations until your Average Precision is over 95%, and precision and recall both over 90%.

* Submit a `set_2_channels_4.png` in your `lab5_logr` folder with your result.  Also submit your updated `add_data_channels.py` file containing your function that adds 2 new channels.  


# Exercise 2: Image-Based Target Detector

The goal is to train a color-target detector that will find the centroid of a colored region in an image.  For this we'll use Logistic Regression to find pixels on the target and then connected components to find the target.

**(a)** Find target pixels using Logistic Regression.  Generally one does not train on test data, so both a training and testing image are provided in the `data` folder, along with masks identifying the targets in each.  Have a look as the images and masks to see what you'll be up against.  

Most of the code to do pixel classification and visualization is provided in `logist_reg.py`.  To run it from inside the `src` folder type:
```
$ python logist_reg.py ../data/train_img.png ../data/train_mask.png ../data/test_img.png --testmask ../data/test_mask.png
```
Try running this and you'll notice that it is unable to detect the target.  The reason is that the `apply()` function is incomplete.  Your task is to update this function to output the Logistic Regression score.  If you do this correctly, then you should see the above code will detect the sphere pixels.

**(b)** It may be possible to improve the detection performance by modifying and/or supplementing the image channels.  Notice that the code outputs the Average Precision.  Your task here is to complete the `modify_img_channels()` function and achieve an Average Precision that is at least 5% larger than using just BGR channels.
```
$ python logist_reg.py ../data/train_img.png ../data/train_mask.png ../data/test_img.png --testmask ../data/test_mask.png --mod-channels
```

**(c)** Finally, complete the `find_largest_target()` function so that you can run the code with the `--find-target` option.  This should use connected components to find the largest target in the image and return its centroid.
```
$ python logist_reg.py ../data/train_img.png ../data/train_mask.png ../data/test_img.png --testmask ../data/test_mask.png --mod-channels --find-target
```

* For (a), (b) and (c), submit your updated code `logist_reg.py` as well as a folder called `sphere` with the output target detection images.  (Note: do not import any other Python libraries.)  To output images use the `--outfolder ../sphere` option:
```
$ python logist_reg.py ../data/train_img.png ../data/train_mask.png ../data/test_img.png --testmask ../data/test_mask.png --mod-channels --find-target --outfolder ../sphere
```

# Submitting this lab
Your code and saved image should all be inside the `<student_repo>/lab5_logr` folder.  To submit do:
```
cd <student_repo>/lab5_logr
git add .
git commit -m "Add a comment here"
git push
```
Do not forget to `push`, or your lab will not be submitted.

