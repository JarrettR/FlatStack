'''
Takes layer data and uses translational/rotation parameters
to tranform the 2D points into three dimensional space
'''

from layer import Layer
from vectors import Point, Vector
from svgpathtools import paths2svg



class Solver(object):
    def __init__(self, joints, layers):
        self.joints = joints
        self.layers = layers
        
    def solve(self, solveJointless = False):
        joints = self.joints
        layers = self.layers
        
        for i, layer in enumerate(layers):
            if solveJointless == False and layer.joint == False:
                print("Solving for ", layer.id)
                layers[i].volume = self.solve_layer(layer)
            elif layer.joint == True:
                print("Joint ", layer.id)
        
        return layers
        
    def solve_layer(self, layer):
        origin = self.find_origin(layer)
        translate_x = layer.translate.x
        translate_y = layer.translate.y
        translate_z = layer.translate.z
        solved = []
        for i, point in enumerate(layer.straight_pairs):
            print(point)
            x = (point[0] - origin[0]) + translate_x
            y = (point[1] - origin[1]) + translate_y
            z =  translate_z
            solved.append(self.point_transform(x, y, z, 0))
        return solved
            
            
    def point_transform(self, x, y, z, axis):
        #Transform from rotation axis
        base = Vector(1,0,0)
        pnt = Vector(x,y,z)
        an = base.angle(pnt)
        print('a - ', base)
        print('b - ', pnt)
        print('c - ', an)
        return pnt
    
    def find_origin(self, layer):
        #Bounding box to find path origin and translate to global origin
        # xmin, xmax, ymin, ymax = paths2svg.big_bounding_box(path)
        # origin = [(xmax + xmin) / 2, (ymax + ymin) / 2]
        # return origin
        
        max_x = 0
        max_y = 0
        min_x = 0
        min_y = 0
        first_run = True
        for point in layer.straight_pairs:
            if first_run == True:
                max_x = point[0]
                max_y = point[1]
                min_x = point[0]
                min_y = point[1]
                first_run = False
            else:
                if point[0] > max_x:
                    max_x = point[0]
                elif point[0] < min_x:
                    min_x = point[0]
                    
                if point[1] > max_y:
                    max_y = point[1]
                elif point[1] < min_y:
                    min_y = point[1]
            # print(point)
        return [(max_x + min_x) / 2, (max_y + min_y) / 2]