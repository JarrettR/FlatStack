# from vpython import canvas, color, curve, vec, extrusion, checkbox, rate, radians, arrow, radians
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path, path_encloses_pt
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
        # print("paths: ", paths)
        # print("attrib: ", attributes)
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

        # print("joints: ", joints.keys())
        joint_assoc = {}

        for joint in self.joints: # collection of ovals
            for oval in self.joints[joint]: #individual ovals
                # print("Solving for ", joint, oval)
                for a, layer in enumerate(self.layers):
                    i = 0
                    for point in layer.straight_pairs:
                        # print("point: ", point)
                        if self.encloses(oval, point):
                            # print("Constraint: ", layer.id, point)
                            name = layer.id + "_p" + str(i)
                            layer.jointed_pairs.append([name, point])
                            if joint in joint_assoc:
                                joint_assoc[joint].append(name)
                            else:
                                joint_assoc[joint] = [name]
                        i += 1
        # print("layers: ", layers.names)

        s = Solver(joint_assoc, self.layers)
        self.layers = s.solve()


    def encloses(self, joint, point):
        point = complex(point[0],point[1])
        # for path in joint:
        # print("enc: ", path, point)
        if path_encloses_pt(point, -100+0j, joint):
            return True
        return False

    def solve(self):
        s = Solver(self.joints, self.layers)
        self.layers = s.solve()

