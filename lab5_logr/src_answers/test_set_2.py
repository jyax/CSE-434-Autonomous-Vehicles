''' test_set_2.py

    Demo script to train and test a point-based classifier for dataset 2
    
'''

import os
from labeled_data import LabeledData
from classifier import Classifier
from add_data_channels import add_rotated_vectors

train = LabeledData( '../data/set_2_train.csv')
test = LabeledData( '../data/set_2_test.csv')

model = Classifier()

# Fit classifier parameters to training data:
model.fit(train)

# Plot target and clutter points from test set:
model.plot_all_points(test, fignum='Input_2',
                      title='Test Data 2', block=False, filesave='../images/set_2.png')

# Classify test points:
scores = model.classify(test)

# Plot classification results:
model.plot_results(test, scores, fignum='Result_2', block=True, filesave='../images/set_2_classify.png')
