# Custom-Physics in Godot
This is a quick overview for the "custom physics" module, which facilitates implementing
custom physics functionality. The module is simply an adapted version of the "Bullet" module
that ships with Godot.
This module was created in a software project internship at RWTH Aachen.

## Compilation
Make sure that you are able to compile Godot 3.1. from sources.
Create a `customphysics` folder in the `modules` folder of Godots 3.1 source distribution.
(Re)compile Godot and run the binary, in the settings a new option at `Physics > 3d > Physics Engine` should appear called "CustomPhysics".

##Features
This module only focuses on Rigid body dynamics:
- unconstrained motion
- constrained motion
  - collision/friction
  - pin joints
  - hinge joints
