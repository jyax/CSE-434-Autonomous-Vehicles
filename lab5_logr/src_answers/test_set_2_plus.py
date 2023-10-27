''' test_set_2_plus.py

    Demo script to train and test a point-based classifier for dataset 2
    
'''

import os
from labeled_data import LabeledData
from classifier import Classifier
from add_data_channels import add_rotated_vectors, add_rvec

path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')

train = LabeledData( '../data/set_2_train.csv')
test = LabeledData( '../data/set_2_test.csv')


# ---------------------------------------------------
# Create your own function in add_data_channels.py that adds extra channels and
# use that instead of add_rotated_vectors()

train_plus = add_rvec(train)
test_plus = add_rvec(test)

# ---------------------------------------------------

# Now train with 4 channels:
model_plus = Classifier()

model_plus.fit(train_plus)

nchannels = train_plus.data.shape[1]
model_plus.plot_all_points(test_plus, fignum='Input_3',
                           title=f'Augmented Test Data 2, Nchannels: {nchannels}', block=False)

scores = model_plus.classify(test_plus)

model_plus.plot_results(test_plus, scores, fignum='Result_3', block=True)
