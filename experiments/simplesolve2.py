
import random
import math
from test_generic import test
from geosolver.intersections import translate_3D, rotate_3D_x, rotate_3D_y, rotate_3D_z,scale_3D, id_transform_3D
from geosolver.geometric import MateConstraint, RigidConstraint, CoincidenceConstraint
from geosolver.geometric import GeometricProblem, GeometricSolver
from geosolver.geometric import DistanceConstraint,AngleConstraint, FixConstraint,RightHandedConstraint
from geosolver.geometric import Point
from geosolver.vector import vector
from geosolver.randomproblem import random_triangular_problem_3D, random_distance_problem_3D
from geosolver.diagnostic import diag_select, diag_print
from geosolver.intersections import distance_2p, angle_3p
from geosolver.configuration import Configuration
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path
from vectors import Point, Vector


class Solver(object):
    def __init__(self, joints, layers):
        self.joints = joints
        self.layers = layers
        self.solver = GeometricProblem(dimension=3)
        self.variables = {}
        self.connectedJoints = {}

    def add_layers(self, layers):
        #
        # for l in layers:
        #Todo: check that shape actually needs constraints before defining
        # l = self.define_shape(l)
        for l in layers:
            straight_pairs = []
            [straight_pairs.append(x) for x in l.straight_pairs if x not in straight_pairs]


            #Need at least 3 points to calculate plane
            if straight_pairs < 3:
                pass

            conf = {}
            i = 0
            for p in straight_pairs:
                # print(p)
                #path4581_p1
                layername = l.name + '_p' + str(i)
                #rigid objects
                print('df', layername)
                self.solver.add_point(layername, vector(p + [0.0]))
                conf[layername] = vector([0,0,0])
                i += 1

            self.solver.add_constraint(RigidConstraint(Configuration(conf)))

            #Todo: Finish / test
            # if l.fixed == True:
            #     self.add_fixed(l)

    def add_mates(self, layers, joints):
        print(joints)
        for jointlayer in joints:
            print(jointlayer)
            for jp in joints[jointlayer]:
                for l in layers:
                    print(l.name)
                    #Eliminate duplicates
                    straight_pairs = []
                    [straight_pairs.append(x) for x in l.straight_pairs if x not in straight_pairs]


                    #Need at least 3 points to calculate plane
                    if straight_pairs < 3:
                        pass
                    #print(l)
                    p_p = []
                    p_ln = ''
                    i = 0
                    for p in straight_pairs:
                        # print(p)
                        #path4581_p1
                        layername = l.name + '_p' + str(i)


                        #joint associations
                        if(self.encloses(p, jp)):
                            print(jointlayer, l.name, i, p)
                            self.add_joint(jointlayer, layername)
                            #joint_assoc[jointlayer].append(l.name)
                        p_p = p
                        p_ln  = layername
                        i += 1

                    #Connected joints
                    # p.append(0.0)
            print(self.connectedJoints)
            for n in self.connectedJoints:
                #todo more than two joints in a
                j = self.connectedJoints[n]
                print(j)
                self.solver.add_constraint(CoincidenceConstraint(Point(j[0]), Point(j[1])))

    def solve(self):
        joints = self.joints
        layers = self.layers

        self.add_layers(layers)

        self.add_mates(layers, joints)

    def add_joint(self, jname, layername):

        if jname in self.connectedJoints:
            self.connectedJoints[jname].append(layername)
        else:
            self.connectedJoints[jname] = [layername]
        # self.solver.add_point(layername, vector(p))

    #Naive bounding box implementation
    def encloses(self, point, joint):
        box = joint.bbox()
        if(point[0] >= box[0]):
            if(point[0] <= box[1]):
                if(point[1] >= box[2]):
                    if(point[1] <= box[3]):
                        return True;
        return False

    def test(self):
        """Test solver on a given problem"""
        problem = self.solver
        # problem = fix3_problem_3d()
        #diag_select(".*")
        print ("problem:")
        print (problem)
        print ("Solving...")
        solver = GeometricSolver(problem)
        print ("...done")
        print ("drplan:")
        print (solver.dr)
        print ("top-level rigids:",list(solver.dr.top_level()))
        result = solver.get_result()
        print ("result:")
        print (result)
        print ("result is",result.flag, "with", len(result.solutions),"solutions")
        check = True
        if len(result.solutions) == 0:
            check = False
        diag_select("(GeometricProblem.verify)|(satisfied)")
        for sol in result.solutions:
            print ("solution:",sol)
            check = check and problem.verify(sol)
        if check:
            print( "all solutions valid")
        else:
            print ("INVALID")

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
    s.test()

    # test3d()
