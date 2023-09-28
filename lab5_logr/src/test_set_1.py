''' test_set_1.py

    Demo script to train and test a point-based classifier for dataset 1
    
    Folders should be organized:
    src / test_set_1.py  (+ other python files)
    data / set_1_train.csv (+ other data files)

'''
from pathlib import Path
from labeled_data import LabeledData
from classifier import Classifier

path = Path(__file__).parents[1] / 'data'  # Get path to data folder

train = LabeledData( str( path / 'set_1_train.csv' ) )  # Load train data
test = LabeledData( str( path / 'set_1_test.csv' ) )    # Load test data

model = Classifier()

# Fit classifier parameters to training data:
model.fit(train)

# Plot target and clutter points from test set:
model.plot_all_points(test, fignum='Input_1', title='Test Data 1', block=False)

# Classify test points:
scores = model.classify(test)

# Plot classification results:
model.plot_results(test, scores, fignum='Result_1', block=True, filesave='./set_1.png')

