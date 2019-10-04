# from vpython import canvas, color, curve, vec, extrusion, checkbox, rate, radians, arrow, radians
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path
from solver import Solver
from constraints import Constraints
from layer import Layer
from vectors import Point, Vector


class Layers(object):
    def __init__(self):
        self.clear()

    def clear(self):
        self.layers = []

    def load_layers(self, paths, attributes):
        joints = {}
        layers = []
        for p, a in zip(paths, attributes):
            if 'fsjoint' in a:
                if a['fsjoint'] in joints:
                    joints[a['fsjoint']].append(p)
                else:
                    joints[a['fsjoint']] = [p]
            else:
                layers.append(Layer(p,a))
                
        self.joints = joints
        self.layers = layers
        
        self.solve()

    def solve(self):
        #See Commandments for 3D Constraint Solving
        s = Solver(self.joints, self.layers)
        self.layers = s.solve()
        
        # s = Constraints(self.joints, self.layers)
        # self.layers = s.solve()
        #for i in range(len(layers)):
        #    self.parse_vector(layers[i][0], layers[i][1])

