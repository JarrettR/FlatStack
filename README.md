# FlatStack

## SVG XML Commands

### fsjoint: id

All points of all paths within the area inside the path marked fsjoint will be labeled with that ID. Multiple paths can be marked with the same ID. All points with the same ID will be joined/mated/coincident with each other in the 3D assembly.

### fsextrude: depth

Depth in mm of the extrusion.

### fsrotate: x, y, z

3 axis of rotation for the final 3D model, in degrees

### fsshowaxis

true/false or 1/0 showing axis of rotation/translation. Default false. Arrows are RGB for XYZ axes.

### fsposition: x, y, z

3D coordinates for the final 3D model

### fscolor / fscolour: hexrgb

Todo

### fsfixed

true/false for fixing object in place. Default false. Causes rotation and translation to be ignored. Undefined behaviour if two fixed object are mated.

