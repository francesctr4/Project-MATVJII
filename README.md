# Project-MATVJII

## Introduction

The aim of this project is to develop a tool to control the orientation of 3D shapes by using a mouse as input. This feature is implemented in many 3D-design software to help users to visualize 3D shapes in several spatial orientations. Warranting a proper and natural user experience using a 2D device as input for controlling the orientation of a 3D body that can rotate arbitrarily about 3 independent axis is a hard task. What the arcball tool does is to define rotations based on the user input, apply the rotation to the 3D model (rotate its frame) and finally redraw the object, refreshing the image on the screen. When this process is ran at high rates, it is possible to create the effect of interaction between the user and the 3D model.

You will use python and tkinter in this project to design a user interface that allows to rotate a cube by means of the arcball technique. Moreover, you will work with different attitude parametrizations and the relations between them to provide to the user useful data about the body orientation.

## Arcball basics

The arcball method consists in the implementation of a virtual trackball. You could imagine a ball enclosing the 3D shape that is desired to be rotated. By clicking on the figure or its surroundings and moving the cursor, the arcball method will simulate the rotation of the sphere about its center and as a consequence, the rotation of the body. By selecting two points over the sphere it is possible to calculate the quaternion that rotates the initial point to the final point. Therefore, we can use this fact to continuously generate rotations, apply them over the points that define our figure and simulate a change in the body’s orientation.

## Project definition

### You will implement a UI, based on the template that you can find on the virtual campus with the next funcionalities:
- The cube that appears on the left has to respond to the user inputs following the presented method of the Holroyd’s arcball and rotate the cube.
- 5 panels have to be shown on the right. Those panels will contain information of the equivalent attitude representation parametrized as:

    - Quaternions
    - Euler principal Angle and Axis
    - Euler angles
    - Rotation vector
    - Rotation matrix

- The information must be updated on run time while dragging the cube.

- Every panel except for the rotation matrix has to be editable and must contain a push button. The push button must update the information in the other parametrizations accordingly and rotate the figure to represent the attitude entered by the user.
- A general reset push button has to be implemented. When pressed, the cube must transform to its original position and all the attitude parametrization must take the corresponding value of zero rotation as well.
