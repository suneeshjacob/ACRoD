# Automatic Computation for Robot Design (ACRoD)

## Description

This repository is dedicated to develop functions for automatic computations for designing robotic manipulators.

## Currently available functions

- Jacobian formulation for spatial manipulators around a given end-effector point.
- Jacobian formulation for planar manipulators around a given end-effector point.

## Usage

### Jacobian for planar manipulators

The topological information of a robot is to be specified by using its robot-topology matrix. For a planar 2R serial manipulator, the robot topology matrix is given by

$$\begin{bmatrix}
9 & 1 & 0 \\
1 & 9 & 1 \\
0 & 1 & 9
\end{bmatrix}$$

The corresponding Jacobian function can be formulated as follows.

Firstly, the required functions are imported as shown below.
```py
from jacobian_planar import jacobian
from numpy import matrix
from sympy import Matrix, lambdify
```

The robot-topology matrix for 3R planar serial manipulator is defined and jacobian information is processed via the imported jacobian function as follows.
```py
M = matrix('9 1 0;1 9 1;0 1 9')
jacobian_information = jacobian(M)
```

Symbolic Jacobian is extracted from `jacobian_information` as follows.
```py
symbolic_jacobian = jacobian_information[0]
symbolic_jacobian
```

In an ipynb file, the above code would produce the following output.

$$\left[\begin{matrix}- a\_{y} + r\_{(1,2)y} & - a\_{y} + r\_{(2,3)y} \\\\ a\_{x} - r\_{(1,2)x} & a\_{x} - r\_{(2,3)x} \\\\ 1 & 1\end{matrix}\right]$$

Active joint velocities, in the corresponding order, can be viewed by running the following lines.
```py
active_joint_velocities = Matrix(jacobian_information[4][0])
print(active_joint_velocities)
```

Robot dimensional parameters can be viewed by running the below line.
```py
robot_dimensional_parameters = Matrix(jacobian_information[7])
print(robot_dimensional_parameters)
```

Jacobian as a Python function, where the arguments are the dimensional parameters of the robot, can be generated as shown below.
```py
jacobian_function = sympy.lambdify([robot_dimensional_parameters],symbolic_jacobian)
```

For a given set of dimensional parameters of the robot, the numerical Jacobian can be computed as follows.
```py
end_effector_point = [1,2]
configuration_parameters = [3,4,5,6]
total_parameters = end_effector_point + configuration_parameters
jacobian_at_the_given_configuration = jacobian_function(total_parameters)
print(jacobian_at_the_given_configuration)
```

