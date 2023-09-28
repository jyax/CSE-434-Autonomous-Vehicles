''' labeled_data.py

    Data Point class used for testing Logistic Regression

    Daniel Morris, Nov 2022

'''
from re import L
import numpy as np
import csv
import matplotlib.pyplot as plt

class LabeledData:

    def __init__(self, other=None) -> None:
        ''' 
            other: Can initialize with another LabeledData (and makes a copy) or with a csv filename and reads it.
            self.data: [N x M] for N points each with M channels
            self.labels: length N vector with 1 for target and 0 for clutter
        '''
        if isinstance(other,LabeledData):
            self.data = other.data.copy()
            self.labels = other.labels.copy()
        elif isinstance(other,str):
            self.read(other)
        else:
            self.data  = np.array([])
            self.labels = np.array([])

    def get_x(self):
        ''' Returns x channel '''
        return self.data[:,0]

    def get_y(self):
        ''' Returns y channel'''
        return self.data[:,1]


    def append( self, other): 
        if self.data.size==0:
            self.data = other.data.copy()
            self.labels = other.labels.copy()
        else:
            self.data = np.concatenate( (self.data, other.data))
            self.labels = np.concatenate( (self.labels, other.labels))       

    def get_subset(self, selected):
        subset = LabeledData()
        subset.data = self.data[selected,...]
        subset.labels = self.labels[selected]
        return subset

    def add_data_channels(self, data_channels):
        ''' Adds one or more channels to data
            data_channel: NxM where N is number of points and M is number of channels
                          N must be the same as self.data
        '''
        if self.data.shape[0] != data_channels.shape[0]:   # Check that there are N points
            print(f'data_channels must be defined for N={self.data.shape[0]} samples')
            raise ValueError
        if len(data_channels.shape)==1:   # If a vector, then reshape to be Nx1
            data_channels = data_channels[:,None]
        self.data = np.concatenate( (self.data, data_channels), axis=1 )


    def write(self, name):
        with open(name, 'w', newline='') as csvfile:
            ptwriter = csv.writer(csvfile, delimiter=',')
            for label, pt in zip(self.labels,self.data):
                ptwriter.writerow((label,*pt))

    def read(self, name):
        with open(name, newline='') as f:
            ptreader = csv.reader(f, delimiter=',')
            vals = np.array(list(ptreader))
        self.labels = vals[:,0].astype(int)
        self.data = vals[:,1:].astype(float)

    def plot_points(self, xchan=0, ychan=1, label=None, c=None, block=True):
        ''' Plots 2 channels of points
            xchan: which channel to be on x axis
            ychan: which channel to be on y axis
        '''
        plt.grid(True, linestyle='--')
        plt.gca().set_aspect('equal')

        plt.scatter(self.data[:,xchan], self.data[:,ychan], c=c, label=label)

        plt.xlabel(f'chan: {xchan}')
        plt.ylabel(f'chan: {ychan}')

        if block:
            plt.legend()
        plt.show(block=block)
