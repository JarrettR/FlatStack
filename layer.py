# from vpython import canvas, color, curve, vec, extrusion, checkbox, rate, radians, arrow, radians
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path, paths2svg
from svgpathtools.path import rotate

from vectors import Point, Vector

class Layer(object):
    def __init__(self, loadPath = False, loadAttrib = False):
        self.clear()
        self.interpolation_points = 10
        if loadAttrib is not False:
            self.load_attributes(loadAttrib)
        if loadPath is not False:
            self.load_path(loadPath)

    def clear(self):
        self.straight_pairs = []
        self.interpolated_pairs = []
        self.translate = Vector(0,0,0)
        self._axis = Vector(1,0,0)
        self._angle = 0
        self.extrude = 2
        self.color = False
        self.fixed = False
        self.joint = False
        self.showAxis = False
        self.showPoints = False
        self.id = ''
        self.volume = []

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
            elif key == 'fsshowpoints':
                self.showPoints = bool(attributes[key])
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
        
        if(self._angle != 0):
            path = rotate(path, self._angle)
        
        #Bounding box to find path origin and translate to global origin
        xmin, xmax, ymin, ymax = paths2svg.big_bounding_box(path)
        origin = [(xmax + xmin) / 2, (ymax + ymin) / 2]
        
        for segment in path:
            if isinstance(segment, Line):
                straight_pairs.append([segment.start.real - origin[0], segment.start.imag - origin[1]])
                interpolated_pairs.append([segment.start.real - origin[0], segment.start.imag - origin[1]])
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
                    self._axis.x, self._axis.y, self._axis.z,
                    self.color, self.showAxis, self.fixed]

