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

        self.layers = translate_joints(joints, layers)
        #for i in range(len(layers)):
        #    self.parse_vector(layers[i][0], layers[i][1])

    def parse_vector(self, in_vector, in_attribs):
        basePath = path_to_vpy(in_vector)
        extrude, rotate, rotateMax, position = attribs_to_vpy(in_attribs)

        if extrude is None:
            extrude_vec = vec(0,0,2)
        else:
            extrude_vec = vec(0,0,extrude)

        if rotate is None:
            rotate_vec = vec(0,0,0)
        else:
            rotate_vec = vec(rotate[0],rotate[1],rotate[2])

        if position is None:
            position = in_vector.bbox()
            position_vec = vec(position[2] - position[0], 0, 0)
        else:
            position_vec = vec(position[0],position[1],position[2])

        extr = extrusion(path=[vec(0,0,0), extrude_vec], color=color.cyan, shape=[ basePath ], pos=position_vec)
        extr.rotate(angle=rotateMax, axis=rotate_vec)

class Layer(object):
    def __init__(self, loadPath = False, loadAttrib = False):
        self.clear()
        self.interpolation_points = 10
        if loadPath is not False:
            self.load_path(loadPath)
        if loadAttrib is not False:
            self.load_attrib(loadAttrib)

    def clear(self):
        self.straight_pairs = []
        self.interpolated_pairs = []
        self.translate = (0,0,0)
        self.rotate = (0,0,0)

    def load_attrib(self, path):
        return 0

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

    def set_range(self):
        #Todo: range to enclose bounding box of all points
        self.scene.range = 100

    def rate(self, sceneRate):
        rate(sceneRate)


    def load_svg(self, filename):
        paths, attributes, svg_attributes = svg2paths2(filename)
        self.layers = Layers()
        self.layers.load_layers(paths, attributes)

if __name__ == '__main__':

    modified = file_modified(filename)


    run = True

    def runner(r):
        global run
        run = r.checked

    checkbox(bind=runner, text='Run', checked=True)

    #scene.waitfor('textures')

    t = 0
    dt = 0.01
    dtheta = 0.001
    while True:
        rate(100)
        if file_modified(filename) > modified:
            print('Modified! Reloading')

            modified = file_modified(filename)

            for a in scene.objects:
                a.visible = False
                del a

            try:
                parse_svg(filename)
            except:
                print('SVG Error')

        if run:
            #scene.camera.rotate(angle=dtheta, axis=vec(0,1,0))
            t += dt