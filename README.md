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

The appropriate Jacobian function can be formulated as follows.

```py
from jacobian_planar import jacobian
from numpy import matrix
from sympy import Matrix, lambdify
```

```py
M = matrix('9 1 0;1 9 1;0 1 9')
jacobian_information = jacobian(M)
```

```py
symbolic_jacobian = jacobian_information[0]
print(symbolic_jacobian)
```

```py
active_joint_velocities = Matrix(jacobian_information[4][0])
print(active_joint_velocities)
```

```py
robot_dimensional_parameters = Matrix(jacobian_information[7])
print(robot_dimensional_parameters)
```

```py
jacobian_function = sympy.lambdify([robot_dimensional_parameters],symbolic_jacobian)
```

```py
end_effector_point = [1,2]
configuration_parameters = [3,4,5,6]
total_parameters = end_effector_point + configuration_parameters
jacobian_at_the_given_configuration = jacobian_function(total_parameters)
print(jacobian_at_the_given_configuration)
```

