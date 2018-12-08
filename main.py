from vpython import canvas, color, curve, vec, extrusion, checkbox, rate, radians
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path
import os, platform

def path_to_vpy(inPath):
    pairs = []
    for segment in inPath:
        if isinstance(segment, Line):
            pairs.append([segment.start.real, segment.start.imag])
        else:
            print('Not line!', segment)
    pairs.append(pairs[0])
    return pairs

def attribs_to_vpy(in_attribs):
    extrude = None
    rotate = None
    max = 0
    if 'fsextrude' in in_attribs:
        extrude = int(in_attribs['fsextrude'])
    if 'fsrotate' in in_attribs:
        rotate = list(map(float, in_attribs['fsrotate'].split(',')))
        rotate = list(map(radians, rotate))

        max = rotate[0]
        if rotate[1] > max:
            max = rotate[1]
        if rotate[2] > max:
            max = rotate[2]
        if max > 0:
            rotate[0] = rotate[0] / max
            rotate[1] = rotate[1] / max
            rotate[2] = rotate[2] / max
    return extrude, rotate, max

def is_joint(attributes):
    if 'fsjoint' in attributes:
        return True
    return False

def parse_vector(in_vector, in_attribs):
    type = 4
    basePath = path_to_vpy(in_vector)
    extrude, rotate, rotateMax = attribs_to_vpy(in_attribs)
    if extrude is None:
        extrude_vec = vec(0,0,2)
    else:
        extrude_vec = vec(0,0,extrude)
    if rotate is None:
        rotate_vec = vec(0,0,0)
    else:
        rotate_vec = vec(rotate[0],rotate[1],rotate[2])
    position = in_vector.bbox()
    vecPosition = vec(position[3] - position[0], 0, 0)
    extr = extrusion(path=[vec(0,0,0), extrude_vec], color=color.cyan, shape=[ basePath ], pos=vecPosition)
    extr.rotate(angle=rotateMax, axis=rotate_vec)

def parse_svg(filename):
    paths, attributes, svg_attributes = svg2paths2(filename)
    for p, a in zip(paths, attributes):
        #print(a)
        if is_joint(a) == True:
            print('Joint at ', p.bbox())
        else:
            parse_vector(p, a)


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
            parse_svg(filename)
        if run:
            #scene.camera.rotate(angle=dtheta, axis=vec(0,1,0))
            t += dt