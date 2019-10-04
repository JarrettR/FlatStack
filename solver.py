'''
Takes layer data and uses translational/rotation parameters
to tranform the 2D points into three dimensional space
'''

from layer import Layer
from vectors import Point, Vector


class Solver(object):
    def __init__(self, joints, layers):
        self.joints = joints
        self.layers = layers
        
    def solve(self):
        joints = self.joints
        layers = self.layers
        
        return layers
        