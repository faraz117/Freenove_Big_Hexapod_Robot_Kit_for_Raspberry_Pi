import pytest
import sys
import os


sys.path.append(os.path.abspath('../../'))
import kinematics

def test_navigate():
    a,b,c = kinematics.coordinateToAngle(1,2,3)
    assert a == 34
    assert b == -124
    assert c == 169