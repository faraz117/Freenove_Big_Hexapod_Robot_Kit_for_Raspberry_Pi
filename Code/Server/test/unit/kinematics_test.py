import unittest
import sys
import os


sys.path.append(os.path.abspath('../../'))
import kinematics

class TestStringMethods(unittest.TestCase):

    def test_upper(self):
        self.assertEqual('foo'.upper(), 'FOO')

    def test_isupper(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())

    def test_split(self):
        s = 'hello world'
        self.assertEqual(s.split(), ['hello', 'world'])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)

    def test_coordinate_to_angle(self):
        a,b,c = kinematics.coordinateToAngle(1,2,3)
        self.assertEqual(a,34)
        self.assertEqual(b,-124)
        self.assertEqual(c,169)

    #   boundary conditions 
    def test_coordinate_to_angle_max(self):
        a,b,c = kinematics.coordinateToAngle(1,2,3)
        self.assertEqual(a,34)
        self.assertEqual(b,-124)
        self.assertEqual(c,169)    
    
    def test_coordinate_to_angle_min(self):
        a,b,c = kinematics.coordinateToAngle(1,2,3)
        self.assertEqual(a,34)
        self.assertEqual(b,-124)
        self.assertEqual(c,169)

if __name__ == '__main__':
    unittest.main()