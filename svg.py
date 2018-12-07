# Coordinates are given as points in the complex plane
from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc, svg2paths2, parse_path
from vpython import vector


def pathToVPy(inPath):
    for segment in inPath:
        if isinstance(segment, Line):
            print('line!')
            print(segment.start)
        else:
            print('Not line!', segment)

def isJoint(attributes):
    if 'joint' in attributes:
        return True
    return False

def parse_svg(filename):
    paths, attributes, svg_attributes = svg2paths2(filename)
    for p, a in zip(paths, attributes):
        #print(a)
        if isJoint(a) == True:
            print('Joint at ', p.bbox())
        else:
            print('Not joint: ', p.bbox())
        #parse_vector(p, a)
    #print("Paths: ", len(paths))
    #print(paths)
    #print("attributes: ", len(attributes))
    #print(attributes)


if __name__ == '__main__':
    filename = 'drawing.svg'
    parse_svg(filename)