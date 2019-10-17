from vpython import canvas, color, curve, vec, extrusion, checkbox, rate, radians, arrow, sphere, radians
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path
import os, platform
from database import Database
from layers import Layers, Layer
from vectors import Point, Vector

class Joint(object):
    def __init__(self):
        self.clear()

    def clear(self):
        self.layers = []
        self.jointlayers = []



class Scene(object):
    def __init__(self):
        self.db = Database('flatstack.db')
        self.db.create_default_tables()
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
            extr = extrusion(path=[vec(0,0,l.depth), vec(0,0,0)],
                color=color.cyan,
                shape=[ l.path ],
                pos=self.vector_to_vec(l.position),
                angle=l.angle,
                axis=self.vector_to_vec(l.axis))
            if l.showAxis:
                self.draw_axis(l)
            if l.showPoints:
                self.draw_points(l)

    def draw_axis(self, layer):
        position = self.vector_to_vec(layer.position)
        mArrow = arrow(angle=layer.angle,
                axis=self.vector_to_vec(layer.axis),
                color=color.orange,
                length=40,
                pos=position)
        rArrow = arrow(axis=vec(1,0,0), color=color.red, length=50, pos=position, shaftwidth=1)
        gArrow = arrow(axis=vec(0,1,0), color=color.green, length=50, pos=position, shaftwidth=1)
        bArrow = arrow(axis=vec(0,0,1), color=color.blue, length=50, pos=position, shaftwidth=1)

    def draw_points(self, layer):
        print(layer.path)
        for p in layer.path:
            pBall = sphere(pos=vec(p[0],p[1], 0), radius=5)

    def load_svg(self, filename):
        paths, attributes, svg_attributes = svg2paths2(filename)
        self.layers = Layers()
        self.layers.load_layers(paths, attributes)
        
    def populate_db(self, filename):
        paths, attributes, svg_attributes = svg2paths2(filename)
        layers = Layers()
        #layers.load_layers(paths, attributes)
        #for l in layers.layers:
        #    print(l)
        joints = {}
        layerlist = []
        for p, a in zip(paths, attributes):
            if 'fsjoint' in a:
                if a['fsjoint'] in joints:
                    joints[a['fsjoint']].append(p)
                else:
                    joints[a['fsjoint']] = [p]
            else:
                #hacky. gross.
                el, ea = Layer(p,a).explode()
                self.db.insert_layer(ea)

        #self.layers = self.translate_joints(joints, layers)
        
    def vector_to_vec(self, inVec):
        return vec(inVec.to_points()[0], inVec.to_points()[1], inVec.to_points()[2])
