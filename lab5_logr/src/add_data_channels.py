''' add_data_channels.py

    Add extra channels to data with the goal of improving a linear classifier

'''

from labeled_data import LabeledData

def add_rotated_vectors(measurements: LabeledData):
    ''' Adds 2 extra channels to measurements.data
        Assumes measurements has already been initialized with data having at least 2 dimensions

        It is not permitted to add new information, only to rearrange the data already in measurements
        Hence, the channels must be functions of x and y (the first two channels of data)
        This example adds a rotated and scaled versions of x and y
        Since this is just a linear transformation of the data, it will not actually help the classifier.

        Your task is to create a similar function, but to add two non-linear operations on x and y so that the 
        classifier can better separate the targets from clutter

        Example usage:
          train = LabeledData( <path_to_data> )
          train_plus = add_rotated_vectors(train)

          test = LabeledData( <path_to_data> )
          test_plus = add_rotated_vectors(test)
    '''
    x = measurements.get_x()
    y = measurements.get_y()
    measurements.add_data_channels( x + y )
    measurements.add_data_channels( -x + y )

    return measurements

