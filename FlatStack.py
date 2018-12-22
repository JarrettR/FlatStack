from vpython import canvas, color, curve, vec, extrusion, checkbox, rate, radians
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path
import os, platform

class Layers(object):
    def __init__(self):
        self.clear()

    def clear(self):
        self.layers = []

    def load_layers(self, paths, attributes):
        joints = {}
        layers = []
        for p, a in zip(paths, attributes):
            #print(a)
            if 'fsjoint' in a:
                print('Joint at ', p.bbox())
                if a['fsjoint'] in joints:
                    joints[a['fsjoint']].append(p)
                else:
                    joints[a['fsjoint']] = [p]
            else:
                layers.append(Layer(p,a))

        self.layers = self.translate_joints(joints, layers)
        #for i in range(len(layers)):
        #    self.parse_vector(layers[i][0], layers[i][1])

    def translate_joints(self, joints, layers):
        return layers


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
        self.translate = vec(0,0,0)
        self.rotate = vec(0,0,0)
        self.rotationHat = 1
        self.extrude = 2
        self.joint = False

    @property
    def position(self):
        return self.translate

    @property
    def rotation(self):
        return self.rotate

    @property
    def path(self):
        return self.interpolated_pairs

    @property
    def depth(self):
        return self.extrude

    def load_attributes(self, attributes):
        #todo: abstract more
        for key in attributes:
            if key == 'fsjoint':
                self.joint = True
            elif key == 'fsextrude':
                self.extrude = int(attributes[key])
            elif key == 'fsrotate':
                #self.rotate = split(attributes[key],',')
                self.rotate = vec(0,0,1)
                self.rotationHat = 1
            elif key == 'fsposition':
                #self.translate = int(attributes[key])
                self.translate = vec(0,0,0)
            elif key == 'fsshowaxis':
                self.showAxis = attributes[key]
            elif key == 'fscolour' or key == 'fscolor':
                self.color = int(attributes[key])
            elif key == 'fsfixed':
                self.fixed = attributes[key]

    def load_path(self, path):
        straight_pairs = []
        interpolated_pairs = []

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



class Scene(object):
    def __init__(self):
        self.scene = canvas()
        self.scene.background = color.gray(0.8)
        self.scene.forward = vec(0,-0.2,-1)
        self.scene.fov = 0.2
        self.set_range()
        self.scene.caption = """Right button drag or Ctrl-drag to rotate "camera" to view scene.
        To zoom, drag with middle button or Alt/Option depressed, or use scroll wheel.
            On a two-button mouse, middle is left + right.
        Touch screen: pinch/extend to zoom, swipe or two-finger rotate.\n"""
        self.clear()

    def clear(self):
        self.objects = []
        for a in self.scene.objects:
            a.visible = False
            del a
        self.layers = None

    def set_range(self):
        #Todo: range to enclose bounding box of all points
        self.scene.range = 100

    def rate(self, sceneRate):
        rate(sceneRate)

    def apply(self):
        for l in self.layers.layers:
            extr = extrusion(path=[vec(0,0,0), vec(0,0,2)], color=color.cyan, shape=[ l.path ], pos=l.position)
            extr.rotate(angle=l.rotationHat, axis=l.rotation)

    def load_svg(self, filename):
        paths, attributes, svg_attributes = svg2paths2(filename)
        self.layers = Layers()
        self.layers.load_layers(paths, attributes)