#!/usr/bin/env python
# coding: utf-8
import numpy as np
import cv2 as cv
import os
# Do not import any more packages than the above
'''
    La1 1 Assignment 
    Based on Python Introduction Notes: https://github.com/dmorris0/python_intro

    Complete the following functions by replacing the pass command and/or variables set equal to None
    Functions need to return the specified output.  In most cases only a single line of code is required.  
    To test your functions and get a score out of 20, call:
      python lab1_student_score.py
    Or run lab1_score.py in VSCode.  When you have everything correct, the output should be:
....................
----------------------------------------------------------------------
Ran 20 tests in 0.100s

OK
    Also, you'll see 3 images displayed which you can close.
'''

####################
# Chapter 4: Strings

def find_warning(message: str) -> str:    
    '''
    Returns the index of the first instance of the substring "warning" or "Warning" (or any other variation on the capitalization)
    If there is no "warning" substring, returns -1
    Hint: don't forget to use a "return" statement
    '''
    
    return message.lower.find("warning")

def every_third(message: str) -> str:
    '''
    Returns every third letter in message starting with the second letter
    '''
    new_message = ""
    count = 0
    for c in message:
        if count == 2:
            new_message+=c
            count = 0
        count+=1
    return new_message

def all_words(message: str) -> str:
    '''
    Breaks message up at each space (" ") and puts the substrings into a list in the same order
    (Don't worry about punctuation)
    '''
    return message.split()[]
    
    
def half_upper_case(message: str) -> str:
    '''
    Returns new_message, where new_message has the same letters as message, but the first half
        of the letters are upper case and the rest lower case.  
        If there are an odd number of letters, round down, that is the first half will have one fewer letters
    '''
    message_new = message
    message_new[0,int(len(message)/2)-1].upper
    message_new[int(len(message)/2), len(message)-1].lower
    return message_new

#############################
# Chapter 5: Numbers and Math

def c_to_f(degree_c: float) -> float:    
    '''
    Converts Celcius to Fahrenheit using the formula
    °F = °C * 9/5 + 32 
    Returns output in Fahrenheit
    '''
    ans = (degree_c * (9/5)) + 32
    return ans
    
def exp_div_fun(a: int, b: int, c: int) -> int:
    '''
    Return the integer remainder you get when you multiply a times itself b times and then divide by c
    '''
    return int(a**b / c)
    
 
 #################################
# Chapter 6: Functions and Loops
    
    
def lcm(x: int, y: int) -> int:
    '''
    Return lowest common multiple of x and y
    Method: let m be the larger of x and y
    Let testval start at m and in a loop increment it by m while testval is not divisible by both x and y
    return testval
    Hint: % will be useful
    '''
    pass               

##################################################
# Chapter 8: Conditional Logic and Control Flow

def cond_cum_sum(a: int, b: int) -> int:
    '''
    Find the cumulative sum of numbers from 0 to a-1 that are not divisible by b
    Hint: % will be useful
    '''
    cnt = 0
    for num in range(0, a-1):
        if num % b == 0:
            cnt+= 1
    return cnt

def divide_numbers(a: float, b: float) -> float:
    ''' 
    Return a / b
    Perform exception handling for ZeroDivisionError, and in this
    case return signed infinity that is the same sign as a
    Hint: np.sign() and np.inf will be useful
    '''
    
    pass

##################################################
# Chapter 9: Tuples, Lists and Dictionaries    

def inc_list(a: int, b: int) -> list:
    '''
    Return a list of numbers that start at a and increment by 1 to b-1
    '''
    pass

def make_all_lower( string_list: list ) -> list:
    ''' Use a single line of Python code for this function
        string_list: list of strings
        returns: list of same strings but all in lower case
        Hint: list comprehension
    '''
    pass

def decrement_string(mystr: str) -> str:
    ''' Use a single line of Python code for this function (hint: list comprehension)
        mystr: a string
        Return a string each of whose characters has is one ASCII value lower than in mystr
        Hint: ord() returns ASCII value, chr() converts ASCII to character, join() combines elements of a list
    '''
    pass

def list2dict( my_list: list ) -> dict:
    ''' 
    Return a dictionary corresponding to my_list where the keys are elements of my_list
    and the values are the square of the key
    '''
    pass

def concat_tuples( tuple1: tuple, tuple2: tuple ) -> tuple:
    ''' 
    Return a tuple that concatenates tuple2 to the end of tuple1
    '''
    pass


##################################################
# Chapter 13: Numpy 
    
def matrix_multiplication(A: np.array,B: np.array) -> np.array:
    ''' 
    A, B: numpy arrays
    Return: matrix multiplication of A and B
    '''
    pass

def largest_row(M: np.array) -> np.array:
    ''' 
    M: 2D numpy array
    Return: 1D numpy array corresponding to the row with the greatest sum in M
    Hint: use np.argmax
    '''
    pass   

def column_scale( A: np.array, vec: np.array) -> np.array:
    '''
    A: [M x N] 2D array
    vec: lenth N array
    return [M x N] 2D array where the i'th column is the corresponding column of A * vec[i]
    Hint: use broadcasting to do this in a single line
    '''
    pass

def row_add( A: np.array, vec: np.array) -> np.array:
    '''
    A: [M x N] 2D array
    vec: lenth M array
    return [M x N] 2D array where the i row is the corresponding row of A + vec[i]
    Hint: use broadcasting to do this in a single line
    '''
    pass

##################################################
# Chapter 14: scipy  

def rotate_90_y(A: np.array) -> np.array:
    '''
    A: [Mx3] array of M 3D points
    return: [Mx3] corresponding to points in A rotated by 90 degrees around Y axis
    Hint: use the scipy rotation operation
    '''
    pass


##################################################
# Chapter 15: OpenCV

class TailLights:

    def __init__(self, impath: str):
        self.impath = impath
        self.img = cv.imread(impath)      
        if self.img is None:
            print('')
            print('*'*60)  # If we can't find the image, check impath
            print('** Current folder is:      ',os.getcwd())
            print('** Unable to read image:   ',impath)  
            print('** Pass correct path to data folder during initialization')
            print('*'*60)
            self.init_succeed = False
        else:
            self.init_succeed = True

    def find_lights_pix(self, show=False) -> np.array:
        ''' Returns a binary mask image for self.img.  This should be 1 for pixels that meet the following conditions on 
            their red, green and blue channels, and zero otherwise.
            red > 220 AND blue > 70 AND blue <= 180 AND green > 120 AND green <= 190
            Note: do NOT iterate over pixels.  
            Hint: The mask can be created in one line of Python code using multiplication operations.  
                  Think about how to implement an AND operation for arrays of 1s and 0s.
            show: if True, then shows the mask in a window called 'Lights Pix'
        '''
        pass

    def find_lights_bbox(self) -> np.array:    
        ''' Finds bounding box around lights 
            returns [Mx4] bounding boxes, one for each light.  Each row is [left, top, width, height] in pixels
            Hint: use cv.connectedComponentsWithStats, see Python_Intro documentation
        '''
        pass

    def draw_lights(self, show=False) -> tuple:
        ''' Draw red rectangles with thickness 2 pixels on the image around each detected light
            Returns image with rectangles draw on it.
            show: if True, then displays output image in a window called 'Detected Lights'
        '''
        pass



if __name__=="__main__":

    # A simple way to debug your functions above is to create tests below.  Then
    # you can run this code directly either in VSCode or by typing: 
    # python lab1_review.py

    # For example, you could do
    print( find_warning("Here is a warning") )
