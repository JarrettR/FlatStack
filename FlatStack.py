from vpython import canvas, color, curve, vec, extrusion, checkbox, rate, radians
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path
import os, platform


class Joint(object):
    def __init__(self):
        self.clear()
    
    def clear(self):
        self.paths = []
        self.enclosedPoints = []
        
    axis_with(self, otherJoint):
        return 0
        
class Path(object):
    def __init__(self):
        self.clear()
        self.interpolation_points = 10
    
    def clear(self):
        self.straight_pairs = []
        self.interpolated_pairs = []
        
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
        self.clear()
        
    def clear(self):
        self.paths = []
        
    def load_svg(self, filename):
        paths, attributes, svg_attributes = svg2paths2(filename)
        joints = {}
        layers = []
        for p, a in zip(paths, attributes):
            #print(a)
            if is_joint(a) == True:
                print('Joint at ', p.bbox())
                if a['fsjoint'] in joints:
                    joints[a['fsjoint']].append(p.bbox())
                else:
                    joints[a['fsjoint']] = [p.bbox()]
            else:
                layers.append((p,a,))

        layers = translate_joints(joints, layers)
        for i in range(len(layers)):
            parse_vector(layers[i][0], layers[i][1])
        print(joints)
        
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
        

def attribs_to_vpy(in_attribs):
    extrude = None
    rotate = None
    rotateMax = 0
    position = None

    if 'fsextrude' in in_attribs:
        extrude = int(in_attribs['fsextrude'])
    if 'fsrotate' in in_attribs:
        rotate = list(map(float, in_attribs['fsrotate'].split(',')))
        rotate = list(map(radians, rotate))

        rotateMax = rotate[0]
        if rotate[1] > rotateMax:
            rotateMax = rotate[1]
        if rotate[2] > rotateMax:
            rotateMax = rotate[2]
        if rotateMax > 0:
            rotate[0] = rotate[0] / rotateMax
            rotate[1] = rotate[1] / rotateMax
            rotate[2] = rotate[2] / rotateMax

    if 'fsposition' in in_attribs:
        position = list(map(float, in_attribs['fsposition'].split(',')))

    return extrude, rotate, rotateMax, position

def is_joint(attributes):
    if 'fsjoint' in attributes:
        return True
    return False

def parse_vector(in_vector, in_attribs):
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

def parse_svg(filename):
    paths, attributes, svg_attributes = svg2paths2(filename)
    joints = {}
    layers = []
    for p, a in zip(paths, attributes):
        #print(a)
        if is_joint(a) == True:
            print('Joint at ', p.bbox())
            if a['fsjoint'] in joints:
                joints[a['fsjoint']].append(p.bbox())
            else:
                joints[a['fsjoint']] = [p.bbox()]
        else:
            layers.append((p,a,))

    layers = translate_joints(joints, layers)
    for i in range(len(layers)):
        parse_vector(layers[i][0], layers[i][1])
    print(joints)

def translate_joints(joints, layers):
    return layers

def file_modified(filename):
    if platform.system() == 'Windows':
        return os.path.getmtime(filename)
    else:
        stat = os.stat(filename)
        return stat.st_mtime



if __name__ == '__main__':
    scene = canvas()
    scene.background = color.gray(0.8)
    scene.forward = vec(0,-0.2,-1)
    scene.fov = 0.2
    #Todo: range to enclose bounding box of all points
    scene.range = 100
    scene.caption = """Right button drag or Ctrl-drag to rotate "camera" to view scene.
    To zoom, drag with middle button or Alt/Option depressed, or use scroll wheel.
         On a two-button mouse, middle is left + right.
    Touch screen: pinch/extend to zoom, swipe or two-finger rotate.\n"""

    filename = 'drawing.svg'

    parse_svg(filename)
    #try:
    #    parse_svg(filename)
    #except:
    #    print('SVG Error')

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