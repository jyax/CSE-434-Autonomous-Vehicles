## exericise 1
#
#  Unit test to evaluate completed lab 1.  If everything is working you should get output similar to
#  the below as well as see three images.
#   ..........
#   ----------------------------------------------------------------------
#   Ran 20 tests in 0.199s
#
#   OK
#
#  The actual grading will be done with a similar function to this
#
import unittest
import numpy.testing as npt
import numpy as np
import os
import cv2 as cv

from lab1_review import find_warning, every_third, all_words, half_upper_case
from lab1_review import c_to_f, exp_div_fun, lcm, cond_cum_sum, divide_numbers
from lab1_review import inc_list, make_all_lower, decrement_string, list2dict, concat_tuples
from lab1_review import matrix_multiplication, largest_row, column_scale, row_add
from lab1_review import rotate_90_y, TailLights

class TestChapter4(unittest.TestCase):

    def test_find_warning(self):
        message = "This is a warning"
        index = 10
        self.assertEqual( find_warning(message), index )
        message = "This is another WARNING"
        index = 16
        self.assertEqual( find_warning(message), index )

    def test_every_third(self):
        message = "The earth is round"
        out = 'hetirn'
        self.assertEqual( every_third(message), out )
        message = "Today is a great day"
        out = 'oys e y'
        self.assertEqual( every_third(message), out )

    def test_all_words(self):
        message = "The earth is round"
        out = ['The', 'earth', 'is', 'round']
        self.assertEqual( all_words(message), out )
        message = "Today is a great day!"
        out = ['Today', 'is', 'a', 'great', 'day!']
        self.assertEqual( all_words(message), out )

    def test_half_upper(self):
        message = "The earth is round"
        out = 'THE EARTH is round'
        self.assertEqual( half_upper_case(message), out )
        message = "Jupiter"
        out = 'JUPiter'
        self.assertEqual( half_upper_case(message), out )

class TestChapter5(unittest.TestCase):

    def test_c_to_f(self):
        input = 21.23
        output = 70.214
        self.assertAlmostEqual( c_to_f(input), output )
        input = 10
        output = 50.0
        self.assertAlmostEqual( c_to_f(input), output )

    def test_exp_div_fun(self):
        self.assertEqual( exp_div_fun(2,3,3), 2 )
        self.assertEqual( exp_div_fun(9,9,4), 1 )

class TestChapter6(unittest.TestCase):

    def lcm(self):
        self.assertEqual( lcm(13,14), 182)        
        self.assertEqual( lcm(6,10), 30)        

class TestChapter8(unittest.TestCase):

    def test_cond_cum_sum(self):
        self.assertEqual( cond_cum_sum(5,2), 4 )
        self.assertEqual( cond_cum_sum(7,2), 9 )

    def test_divide_numbers(self):
        self.assertEqual( divide_numbers(1, 2), 0.5 )
        self.assertEqual( divide_numbers(5, 0), np.inf )
        self.assertEqual( divide_numbers(-10, 0), -np.inf )

class TestChapter9(unittest.TestCase):

    def test_inc_list(self):
        self.assertEqual( inc_list(3,7), [3,4,5,6])
    
    def test_make_all_lower(self):
        self.assertEqual( make_all_lower(['Hello', "Fred!", "How", "are", "you?"]),
                           ['hello', 'fred!', 'how', 'are', 'you?'] )

    def test_decrement_string(self):
        self.assertEqual( decrement_string("Umbrella"), 'Tlaqdkk`')

    def test_list2dict(self):
        self.assertEqual( list2dict( [5, 9, 3, 2]), {5: 25, 9: 81, 3: 9, 2: 4} )

    def test_concat_tuples(self):
        self.assertEqual( concat_tuples( (5,4,3), (2,1) ), (5,4,3,2,1))

class TestChapter13(unittest.TestCase):

    def test_matrix_multiplication(self):
        A = np.array([[1.,2.,3.],[4.,5.,6.]])
        B = np.array([[2.,4.,1.],[-2.,3.,2.]]).T
        C = np.array([[13.,10.],[34.,19.]])
        npt.assert_allclose( matrix_multiplication(A, B), C )
        


    def test_largest_row(self):
        A = np.arange(15).reshape((3,5))
        B = np.arange(10,15)
        npt.assert_allclose( largest_row(A), B)
        

    def test_column_scale(self):
        A = np.arange(12).reshape( (3,4) )
        vec = np.arange(6,10)
        output = np.array( [[0,7,16,27],[24,35,48,63],[48,63,80,99]] )
        npt.assert_allclose( column_scale(A, vec), output)
        

    def test_row_add(self):
        A = np.arange(12).reshape( (3,4) )
        vec = np.arange(7,10)
        output = np.array([[ 7,  8,  9, 10], [12, 13, 14, 15], [17, 18, 19, 20]])
        npt.assert_allclose( row_add(A, vec), output)

class TestChapter14(unittest.TestCase):

    def test_rotate_90_y(self):
        A = np.reshape(np.arange(12),(4,3))
        rot_gt = np.reshape([ 2.,  1.,  0.,  5.,  4., -3.,  8.,  7., -6., 11., 10., -9.], (4,3))
        B = rotate_90_y(A)
        npt.assert_allclose( B, rot_gt)


class TestOpenCV(unittest.TestCase):

    def setUp(self):
        self.path_to_folder = ''  # If run in VSCode folder, this path may need to be: 'labs_2022/lab1_python'
        self.data_folder = 'data'
        self.gt_folder = 'gt'
        path_to_img = os.path.join(self.path_to_folder,self.data_folder,'tail_lights.png')
        self.lights = TailLights(path_to_img)
        cv.imshow('Tail Lights', self.lights.img)
        self.assertTrue( self.lights.init_succeed )

    def test_find_lights_pix(self):
        path_to_img = os.path.join(self.path_to_folder,self.gt_folder,'tail_lights_mask.png')
        gt = cv.imread(path_to_img, cv.IMREAD_GRAYSCALE )
        if isinstance(gt,type(None)):
            print('Unable to read GT lights from:',path_to_img)
            print('Current path:',os.getcwd())
        npt.assert_equal( self.lights.find_lights_pix(True)>0, gt>0)

    def test_draw_lights(self):
        path_to_gt = os.path.join(self.path_to_folder,self.gt_folder,'tail_lights_rectangles.png')
        gt = cv.imread(path_to_gt, cv.IMREAD_COLOR )
        light_img = self.lights.draw_lights(True)
        npt.assert_allclose(light_img, gt, atol=2)
        
if __name__=='__main__':
    # Do unit testing on lab 1
    unittest.main(exit=False)  # Do not exit to keep displaying images

    cv.waitKey()              # Press a key to quit
    cv.destroyAllWindows()
