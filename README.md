# FlatStack

Better documentation forthcoming.

![interface](interface.png)

FlatStack is a flatpack design system, intended to leverage the strength of existing vector software.

This project comes from frequently designing complex 3D assemblies to be laser cut. Using standard CAD packages like Fusion 360 or SolidWorks is annoying, because the sketch or drawing capabilities are not their primary purpose, and a little bit clunky as a result.

Proper vector software, like Inkscape or Illustrator, however, are fantastic.


This is an attempt at turning them into a CAD package of sorts, for flatpack designs.

Keeping the 3D view in one window off to the side, it will continuously update the 3D assembly as you draw the 2D profiles in your vector software of choice.



## SVG XML Commands

### fsjoint: id

All points of all paths within the area inside the path marked fsjoint will be labeled with that ID. Multiple paths can be marked with the same ID. All points with the same ID will be joined/mated/coincident with each other in the 3D assembly.

### fsextrude: depth

Depth in mm of the extrusion.

### fsrotate: x, y, z

3 axis of rotation for the final 3D model, in degrees

### fsposition: x, y, z

3D coordinates for the final 3D model

### fsshowaxis

true/false or 1/0 showing axis of rotation/translation. Default false. Arrows are RGB for XYZ axes.

### fscolor / fscolour: hexrgb

Todo

### fsfixed

true/false for fixing object in place. Default false. Causes rotation and translation to be ignored. Undefined behaviour if two fixed object are mated.

