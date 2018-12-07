from vpython import canvas, color, curve, vec, extrusion, checkbox, rate
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
    
def is_joint(attributes):
    if 'joint' in attributes:
        return True
    return False

def parse_vector(in_vector, in_attribs):
    type = 4
    basePath = path_to_vpy(in_vector)
    position = in_vector.bbox()
    vecPosition = vec(position[3] - position[0], 0, 0)
    extrusion(path=[vec(0,0,0), vec(0,0,-0.7)], color=color.cyan, shape=[ basePath ], pos=vecPosition)
    
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
    scene = canvas() # This is needed in Jupyter notebook and lab to make programs easily rerunnable
    scene.background = color.gray(0.8)
    scene.forward = vec(0,-0.2,-1)
    scene.fov = 0.2
    #Todo: range to enclose bounding box of all points
    scene.range = 200
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
                print(a)
                a.visible = False
                del a
            parse_svg(filename)
        if run:
            #scene.camera.rotate(angle=dtheta, axis=vec(0,1,0))
            t += dt