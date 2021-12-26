'''
Takes layer data and uses translational/rotation parameters
to tranform the 2D points into three dimensional space
'''

from layer import Layer
# from vectors import Point, Vector
from svgpathtools import paths2svg
from geosolver.geometric import GeometricProblem, GeometricSolver, DistanceConstraint,AngleConstraint, FixConstraint, RightHandedConstraint
from geosolver.vector import vector
from geosolver.diagnostic import diag_select, diag_print



class Solver(object):
    def __init__(self, joints, layers):
        self.joints = joints
        self.layers = layers
        self.problem = GeometricProblem(dimension=3)

    def solve(self):
        joints = self.joints
        layers = self.layers

        # print(joints)
        # print(layers)
        self.solve_joints(joints, layers)

    def solve_joints(self, joints, layers):

        for i, layer in enumerate(layers):
            print("Solving for ", layer.id)
            i = 0
            for point in layer.jointed_pairs:
                name = layer.id + "_p" + str(i)
                self.problem.add_point(name, vector([layer.straight_pairs[i][1], layer.straight_pairs[i][1],0]))

                i += 1
            # layers[i].volume = self.solve_layer(layer)

        for joint in joints:
            i = 2
            while i < len(joint):
                self.problem.add_constraint(DistanceConstraint(joint[1], joint[i], 0.0))
                i += 1

        self.test(self.problem)

        return layers

    def test(self, problem):
        """Test solver on a given problem"""
        #diag_select(".*")
        print("problem:")
        print(problem)
        print("Solving...")
        solver = GeometricSolver(problem)
        print("...done")
        print("drplan:")
        print(solver.dr)
        print("top-level rigids:",list(solver.dr.top_level()))
        result = solver.get_result()
        print("result:")
        print(result)
        print("result is",result.flag, "with", len(result.solutions),"solutions")
        check = True
        if len(result.solutions) == 0:
            check = False
        diag_select("(GeometricProblem.verify)|(satisfied)")
        for sol in result.solutions:
            print("solution:",sol)
            check = check and problem.verify(sol)
        if check:
            print("all solutions valid")
        else:
            print("INVALID")


    def solve_layer(self, layer):
        origin = self.find_origin(layer)
        translate_x = layer.translate.x
        translate_y = layer.translate.y
        translate_z = layer.translate.z
        solved = []
        for i, point in enumerate(layer.straight_pairs):
            x = (point[0] - origin[0])# + translate_x
            y = (point[1] - origin[1])# + translate_y
            z =  0#translate_z
            print("point: ", point, x, y)
            # solved.append(self.point_transform(x, y, z, 0))
        return solved


    def point_transform(self, x, y, z, axis):
        #Transform from rotation axis
        base = Vector(0,1,0)
        pnt = Vector(x,y,z)
        # an = base.angle(pnt)
        # print('a - ', base)
        # print('b - ', pnt)
        # print('c - ', an)
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