# Project-MATVJII

## Context of the project

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
