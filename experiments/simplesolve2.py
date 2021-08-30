#!/usr/bin/env python
"""This module provides some tests for the GeoSolver.
These tests are concerned with 3D solving.
The tests are also simple examples of how to use of the GeomSolver API"""

import random
import math
from test_generic import test
from geosolver.geometric import GeometricProblem, GeometricSolver, DistanceConstraint,AngleConstraint, FixConstraint,RightHandedConstraint
from geosolver.vector import vector
from geosolver.randomproblem import random_triangular_problem_3D, random_distance_problem_3D
from geosolver.diagnostic import diag_select, diag_print
from geosolver.intersections import distance_2p, angle_3p
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path
from vectors import Point, Vector

# ---------- 3D problems -----

def fix3_problem_3d():
    """A problem with a fix constraint"""
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.0, 0.0, 1.0]))
    #problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    #problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    #problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(FixConstraint('v1', vector([0.0,0.0,0.0])))
    problem.add_constraint(FixConstraint('v2', vector([10.0,0.0,0.0])))
    problem.add_constraint(FixConstraint('v3', vector([5.0,5.0,0.0])))
    return problem

def fix2_problem_3d():
    """A problem with a fix constraint"""
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.0, 0.0, 1.0]))
    #problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(FixConstraint('v1', vector([0.0,0.0,0.0])))
    problem.add_constraint(FixConstraint('v2', vector([10.0,0.0,0.0])))
    return problem

def fix1_problem_3d():
    """A problem with a fix constraint"""
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.0, 0.0, 1.0]))
    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(FixConstraint('v1', vector([0.0,0.0,0.0])))
    return problem



def double_banana_problem():
    """The double banana problem"""
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.5, 0.5, 1.0]))
    problem.add_point('v5', vector([0.5, 0.5,-1.0]))
    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v5', 10.0))

    problem.add_point('w1', vector([0.0, 0.0, 0.0]))
    problem.add_point('w2', vector([1.0, 0.0, 0.0]))
    problem.add_point('w3', vector([0.0, 1.0, 0.0]))
    problem.add_constraint(DistanceConstraint('w1', 'w2', 10.0))
    problem.add_constraint(DistanceConstraint('w1', 'w3', 10.0))
    problem.add_constraint(DistanceConstraint('w2', 'w3', 10.0))
    problem.add_constraint(DistanceConstraint('w1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('w2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('w3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('w1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('w2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('w3', 'v5', 10.0))

    return problem

def double_banana_plus_one_problem():
    """The double banana problem, plus one constraint (well-constrained)"""
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.5, 0.5, 1.0]))
    problem.add_point('v5', vector([0.5, 0.5,-1.0]))
    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v5', 10.0))

    problem.add_point('w1', vector([0.0, 0.0, 0.0]))
    problem.add_point('w2', vector([1.0, 0.0, 0.0]))
    problem.add_point('w3', vector([0.0, 1.0, 0.0]))
    problem.add_constraint(DistanceConstraint('w1', 'w2', 10.0))
    problem.add_constraint(DistanceConstraint('w1', 'w3', 10.0))
    problem.add_constraint(DistanceConstraint('w2', 'w3', 10.0))
    problem.add_constraint(DistanceConstraint('w1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('w2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('w3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('w1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('w2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('w3', 'v5', 10.0))

    problem.add_constraint(DistanceConstraint('v1', 'w1', 10.0))

    return problem


def double_tetrahedron_problem():
    """The double tetrahedron problem"""
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.5, 0.5, 1.0]))
    problem.add_point('v5', vector([0.5, 0.5,-1.0]))
    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v5', 10.0))
    return problem


def dad_tetrahedron_problem():
    """The double tetrahedron problem with an angle"""
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.5, 0.5, 1.0]))
    problem.add_point('v5', vector([0.5, 0.5,-1.0]))
    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(AngleConstraint('v2', 'v1','v3', 60.0*math.pi/180.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v5', 10.0))
    return problem

def ada_tetrahedron_problem():
    """The double tetrahedron problem with an angle"""
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.5, 0.5, 1.0]))
    problem.add_point('v5', vector([0.5, 0.5,-1.0]))
    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(AngleConstraint('v3', 'v1','v2', 60.0*math.pi/180.0))
    problem.add_constraint(AngleConstraint('v1', 'v2','v3', 60.0*math.pi/180.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v5', 10.0))
    return problem

def ada_3d_problem():
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([random.random() for i in [1,2]]))
    problem.add_point('v2', vector([random.random() for i in [1,2]]))
    problem.add_point('v3', vector([random.random() for i in [1,2]]))
    problem.add_constraint(DistanceConstraint('v1','v2',distance_2p(problem.get_point('v1'), problem.get_point('v2'))))
    problem.add_constraint(AngleConstraint('v3', 'v1', 'v2',
       angle_3p(problem.get_point('v3'), problem.get_point('v1'), problem.get_point('v2'))
    ))
    problem.add_constraint(AngleConstraint('v1', 'v2', 'v3',
       angle_3p(problem.get_point('v1'), problem.get_point('v2'), problem.get_point('v3'))
    ))
    return problem




def overconstrained_tetra():
    problem = GeometricProblem(dimension=3)
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.5, 0.5, 1.0]))
    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    # overconstrain me!
    problem.add_constraint(AngleConstraint('v1', 'v2', 'v3', math.pi/3))
    #problem.add_constraint(AngleConstraint('v1', 'v2', 'v3', math.pi/4))
    return problem

def diamond_3d():
    """creates a diamond shape with point 'v1'...'v4' in 3D with one solution"""
    # Following should be well-constraint, gives underconstrained (need extra rule/pattern)
    L=10.0
    problem = GeometricProblem(dimension=3, use_prototype=False)      # no prototype based selection
    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([-5.0, 5.0, 0.0]))
    problem.add_point('v3', vector([5.0, 5.0, 0.0]))
    problem.add_point('v4', vector([0.0, 10.0, 0.0]))
    problem.add_constraint(DistanceConstraint('v1', 'v2', L))
    problem.add_constraint(DistanceConstraint('v1', 'v3', L))
    problem.add_constraint(DistanceConstraint('v2', 'v3', L))
    problem.add_constraint(DistanceConstraint('v2', 'v4', L))
    problem.add_constraint(DistanceConstraint('v3', 'v4', L))
    # this bit of code constrains the points v1...v4 in a plane with point p above it
    problem.add_point('p', vector([0.0, 0.0, 1.0]))
    problem.add_constraint(DistanceConstraint('v1', 'p', 1.0))
    problem.add_constraint(AngleConstraint('v2','v1','p', math.pi/2))
    problem.add_constraint(AngleConstraint('v3','v1','p', math.pi/2))
    problem.add_constraint(AngleConstraint('v4','v1','p', math.pi/2))
    return problem

# ----------- 3d tests ----------

def test_ada_3d():
    problem = ada_3d_problem()
    diag_select("nothing")
    print "problem:"
    print problem
    solver = GeometricSolver(problem)
    print "drplan:"
    print solver.dr
    print "number of top-level rigids:",len(solver.dr.top_level())
    result = solver.get_result()
    print "result:"
    print result
    print "result is",result.flag, "with", len(result.solutions),"solutions"
    check = True
    if len(result.solutions) == 0:
        check = False
    diag_select(".*")
    for sol in result.solutions:
        print "solution:",sol
        check = check and problem.verify(sol)
    diag_select("nothing")
    if check:
        print "all solutions valid"
    else:
        print "INVALID"

def selection_test():
    problem = GeometricProblem(dimension=3,use_prototype=False)

    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.5, 0.5, 1.0]))
    problem.add_point('v5', vector([0.5, 0.5,-1.0]))

    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v5', 10.0))

    s1 = RightHandedConstraint('v1','v2','v4','v5')

    # add selection con
    problem.add_constraint(s1)

    # solve
    solver = GeometricSolver(problem)
    print len(solver.get_solutions()), "solutions"

    # remove and add constraint
    print "removing selection-constraint"
    problem.rem_constraint(s1)

    # solve again
    print len(solver.get_solutions()), "solutions"

    # remove and add constraint
    print "re-adding selection constraint"
    problem.add_constraint(s1)

    # solve again
    print len(solver.get_solutions()), "solutions"

    # remove distance
    print "removing and re-adding distance v1-v5"
    problem.rem_constraint(problem.get_distance("v1","v5"))
    problem.add_constraint(DistanceConstraint('v1', 'v5', 10.0))

    # solve again
    print len(solver.get_solutions()), "solutions"

def selection_problem():
    """The double tetrahedron problem with selection constraints"""

    problem = GeometricProblem(dimension=3, use_prototype=False)  # no prototype based selection

    problem.add_point('v1', vector([0.0, 0.0, 0.0]))
    problem.add_point('v2', vector([1.0, 0.0, 0.0]))
    problem.add_point('v3', vector([0.0, 1.0, 0.0]))
    problem.add_point('v4', vector([0.5, 0.5, 1.0]))
    problem.add_point('v5', vector([0.5, 0.5,-1.0]))

    problem.add_constraint(DistanceConstraint('v1', 'v2', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v3', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v4', 10.0))
    problem.add_constraint(DistanceConstraint('v1', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v2', 'v5', 10.0))
    problem.add_constraint(DistanceConstraint('v3', 'v5', 10.0))

    #problem.add_constraint(SelectionConstraint(is_right_handed, ['v1','v2','v4','v5']))
    problem.add_constraint(RightHandedConstraint('v1','v2','v4','v5'))

    return problem


def test3d():
    #diag_select("clsolver")
    #test(double_tetrahedron_problem())
    #test(ada_tetrahedron_problem())
    #test(double_banana_problem())
    #test(double_banana_plus_one_problem())
    #test(random_triangular_problem_3D(10,10.0,0.0,0.5))
    #test(random_distance_problem_3D(10,1.0,0.0))
    test(fix1_problem_3d())
    test(fix2_problem_3d())
    test(fix3_problem_3d())
    #test(selection_problem())
    #selection_test()
    #test(overconstrained_tetra())
    #test(diamond_3d())


class Solver(object):
    def __init__(self, joints, layers):
        self.joints = joints
        self.layers = layers
        self.solver = GeometricProblem(dimension=3)
        self.variables = {}

    def solve(self):
        joints = self.joints
        layers = self.layers


        # for l in layers:
            #Todo: check that shape actually needs constraints before defining
            # l = self.define_shape(l)

            #Todo: Finish / test
            # if l.fixed == True:
            #     self.add_fixed(l)

        print(joints)
        for jointlayer in joints:
            print(jointlayer)
            for jp in joints[jointlayer]:
                for l in layers:
                    print(l.name)
                    #Eliminate duplicates
                    list
                    straight_pairs = []
                    [straight_pairs.append(x) for x in l.straight_pairs if x not in straight_pairs]
                    # straight_pairs = list(dict.fromkeys(l.straight_pairs))
                    #Need at least 3 points to calculate plane
                    if straight_pairs < 3:
                        pass
                    #print(l)
                    i = 0
                    for p in straight_pairs:
                        # print(p)
                        if(self.encloses(p, jp)):
                            print(jointlayer, l.name, i, p)
                            self.add_joint(jointlayer, l.name, i, p)
                            #joint_assoc[jointlayer].append(l.name)
                        i += 1

    def add_joint(self, jname, lname, pname, p):

        #path4581_p1_x
        layername = lname + '_p' + str(pname) + '_'

    #Naive bounding box implementation
    def encloses(self, point, joint):
        box = joint.bbox()
        if(point[0] >= box[0]):
            if(point[0] <= box[1]):
                if(point[1] >= box[2]):
                    if(point[1] <= box[3]):
                        return True;
        return False



class Layer(object):
    def __init__(self, loadPath = False, loadAttrib = False):
        self.clear()
        self.interpolation_points = 10
        if loadPath is not False:
            self.load_path(loadPath)
        if loadAttrib is not False:
            self.load_attributes(loadAttrib)

    def clear(self):
        self.straight_pairs = []
        self.interpolated_pairs = []
        self.translate = Vector(0,0,0)
        self._axis = Vector(0,0,0)
        self._angle = 1
        self.extrude = 2
        self.color = False
        self.fixed = False
        self.joint = False
        self.showAxis = False
        self.id = ''

    @property
    def position(self):
        return self.translate

    @property
    def axis(self):
        return self._axis

    @property
    def angle(self):
        return self._angle

    @property
    def path(self):
        return self.interpolated_pairs

    @property
    def depth(self):
        return self.extrude

    @property
    def name(self):
        return self.id

    def load_attributes(self, attributes):
        #todo: abstract more
        for key in attributes:
            if key == 'fsjoint':
                self.joint = True
            elif key == 'fsextrude':
                self.extrude = int(attributes[key])
            elif key == 'fsangle':
                self._angle = int(attributes[key])
            elif key == 'fsaxis':
                self._axis = self.str_to_vec(attributes[key])
                # self._axis.x = radians(self._axis.x)
                # self._axis.y = radians(self._axis.y)
                # self._axis.z = radians(self._axis.z)
            elif key == 'fsposition':
                self.translate = self.str_to_vec(attributes[key])
            elif key == 'fsshowaxis':
                self.showAxis = bool(attributes[key])
            elif key == 'fscolour' or key == 'fscolor':
                self.color = int(attributes[key])
            elif key == 'fsfixed':
                self.fixed = attributes[key]
            elif key == 'id':
                self.id = attributes[key]

    def str_to_vec(self, inputStr):
        elements = inputStr.split(',')
        outVec = Vector(float(elements[0]), float(elements[1]), float(elements[2]))
        return outVec

    def load_path(self, path):
        straight_pairs = []
        interpolated_pairs = []
        #Interpolated number of points:
        points = 10

        for segment in path:
            if isinstance(segment, Line):
                straight_pairs.append([segment.start.real, segment.start.imag])
                interpolated_pairs.append([segment.start.real, segment.start.imag])
            elif isinstance(segment, CubicBezier) or isinstance(segment, Arc) or isinstance(segment, QuadraticBezier):
                start = [segment.start.real, segment.start.imag]
                straight_pairs.append(start)
                interpolated_pairs.append(start)

                for i in range(1, points):
                    end = [segment.point(i/points).real, segment.point(i/points).imag]
                    interpolated_pairs.append(end)

                straight_pairs.append(end)
            else:
                print('Unknown SVG path type!', segment)
        try:
            straight_pairs.append(straight_pairs[0])
            interpolated_pairs.append(interpolated_pairs[0])
        except:
            print('No segments in path')

        self.straight_pairs = straight_pairs
        self.interpolated_pairs = interpolated_pairs

    def explode(self):
        return self.straight_pairs, [self.id,
                    self.translate.x, self.translate.y, self.translate.z,
                    self.extrude, self._angle,
                    self.color, self.showAxis, self.fixed]


if __name__ == '__main__':

    filename = 'simplesolve.svg'


    paths, attributes, svg_attributes = svg2paths2(filename)

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

    s = Solver(joints, layers)
    s.solve()

    # test3d()
