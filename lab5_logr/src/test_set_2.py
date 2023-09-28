''' test_set_2.py
'''
from pathlib import Path
from labeled_data import LabeledData
from classifier import Classifier
from add_data_channels import multiply_rotated_vectors as mult

path = Path(__file__).parents[1] / 'data'

train = LabeledData( str( path / 'set_2_train.csv' ) )
train_mult = mult(train)
test = LabeledData( str( path / 'set_2_test.csv' ) )
test_mult = mult(train)

model = Classifier()

model.fit(train_mult)

model.plot_all_points(test_mult, fignum='Input_2', title='Test Data 2', block=False)

scores = model.classify(test_mult)

model.plot_results(test_mult, scores, fignum='Result_2', block=True, filesave='./set_2.png')
