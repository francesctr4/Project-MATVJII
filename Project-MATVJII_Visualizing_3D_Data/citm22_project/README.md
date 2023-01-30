# citm22_project

This repos implements a template for the 2022 MATHSII project.

The aim of the project is to develop a tool to control the orientation of 3D shapes by using a mouse as input. This feature is implemented in many 3D-design software to help users to visualize 3D shapes in several spatial orientations.

Warranting a proper and natural user experience using a 2D device as input for controlling the orientation of a 3D body that can rotate arbitrarily about 3
independent axis is a hard task. What the arcball tool does is to define rotations based on the user input, apply the rotation to the 3D model (rotate its frame) and finally redraw the object. When this process is ran at high rates, it is possible to create the effect of interaction between the user and the 3D model.

You will use python and tkinter in this project to design a user interface that allows to rotate a cube by means of the arcball technique. Moreover, you will work with different attitude parametrizations and the relations between them to provide to the user useful data about the body orientation.

![Alt text](imgs/reference_img.png?raw=true "Title")

## Dependencies and requirements
This prject requires python > 3.9
Dependencies and packages can be installed by executing on the cloned directory

```
python3 -m pip install -r reqs.txt
```
