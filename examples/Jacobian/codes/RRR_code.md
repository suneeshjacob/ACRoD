# Formulation of Jacobian for RRR planar serial manipulator as an example

## Figure

<p align="center">
    <img src="./examples/Jacobian/images/RRR.png" alt="RRR planar serial manipulator" width="500px">
</p>

A figure of RRR planar serial manipulator is shown above. The corresponding adjacency matrix is given by

$$\bf{M} = \left[\begin{matrix}L_1 & R & O & O \\\\A & L_2 & R & O\\\\O & A & L_3 & R\\\\O & O & A & L_4\end{matrix}\right]$$

## Usage

### Jacobian for planar manipulators

The topological information of a robot is to be specified by using its robot-topology matrix, as defined [here](Robot_Topology_Matrix.md). For RRR planar serial manipulator shown above, the robot topology matrix is given by

$$\left[\begin{matrix}9 & 1 & 0 & 0 \\\\ 1 & 9 & 1 & 0 \\\\ 0 & 1 & 9 & 1 \\\\ 0 & 0 & 1 & 9\end{matrix}\right]$$

The corresponding Jacobian function can be formulated as follows.

Firstly, the required functions are imported as shown below.
```py
from acrod.functions import jacobian
from numpy import array
```


The robot-topology matrix for 3R planar serial manipulator is defined and jacobian information is processed via the imported jacobian class as follows.
```py
M = numpy.array(
    [[9, 1, 0, 0],
     [1, 9, 1, 0],
     [0, 1, 9, 1],
     [0, 0, 1, 9]]
)
jac = jacobian(M, robot_type='planar')
```


Jacobian function is generated as shown below.
```py
jacobian_function = jac.get_jacobian_function()
```


In the process of generating the above jacobian function, other attributes of the jacobian object also are updated. Symbolic Jacobian matrices can be extracted from the attributes. Since this is a serial robot, the matrix $J_a$ itself would be the Jacobian matrix of the manipulator. The matrix $J_a$ is extracted from `Ja` attribute of the jacobian object as follows.
```py
symbolic_jacobian = jac.Ja
symbolic_jacobian
```

In an ipynb file of JupyterLab, the above code would produce the following output.

$$\left[\begin{matrix}- a_{y} + r_{(1,2)y} & - a_{y} + r_{(2,3)y} & - a_{y} + r_{(3,4)y}\\\\a_{x} - r_{(1,2)x} & a_{x} - r_{(2,3)x} & a_{x} - r_{(3,4)x}\\\\1 & 1 & 1\end{matrix}\right]$$

The above Jacobian is based on the notations defined and described [here](Notation_and_Nomenclature.md).

Active joint velocities, in the corresponding order, can be viewed by running the following lines.
```py
active_joint_velocities = jac.active_joint_velocities_symbolic
active_joint_velocities
```

In an ipynb file of JupyterLab, the above code would produce the following output.

$$\left[\begin{matrix}\dot{\theta}\_{(1,2)}\\\\\dot{\theta}\_{(2,3)}\\\\\dot{\theta}\_{(3,4)}\end{matrix}\right]$$

Robot dimensional parameters can be viewed by running the below line.
```py
robot_dimensional_parameters = jac.parameters_symbolic
robot_dimensional_parameters
```

In an ipynb file of JupyterLab, the above code would produce the following output.

$$\left[\begin{matrix}r_{(1,2)x}\\\\r_{(1,2)y}\\\\r_{(2,3)x}\\\\r_{(2,3)y}\\\\r_{(3,4)x}\\\\r_{(3,4)y}\end{matrix}\right]$$


Robot end-effector parameters can be viewed by running the below line.
```py
robot_endeffector_parameters = jac.endeffector_variables_symbolic
robot_endeffector_parameters
```

In an ipynb file of JupyterLab, the above code would produce the following output.

$$\left[\begin{matrix}a_{x}\\\\a_{y}\end{matrix}\right]$$

#### Sample computation of Jacobian for the configuration corresponding to the parameters shown below:

- End-effector point: $\textbf{a}=\hat{i}+2\hat{j}$
- Locations of joints: $\textbf{r}\_{(1,2)}=3\hat{i}+4\hat{j}$, $\textbf{r}\_{(2,3)}=2\hat{i}+1\hat{j}$ and $\textbf{r}\_{(3,4)}=4\hat{i}+2\hat{j}$

For the given set of dimensional parameters of the robot, the numerical Jacobian can be computed as follows. Firstly, we need to gather the configuration parameters in Python list format, in a particular order. The robot dimensional parameters from `jac.parameters_symbolic` are found (as shown earlier) to be in the order of $r_{(1,2)x}$, $r_{(1,2)y}$, $r_{(2,3)x}$, $r_{(2,3)y}$, $r_{(3,4)x}$ and $r_{(3,4)y}$. Hence the configuration parameters are to be supplied in the same order, as a list. Thus, the computation can be performed as shown below.
```py
end_effector_point = [1,2]
configuration_parameters = [3, 4, 2, 1, 4, 2]
jacobian_at_the_given_configuration = jacobian_function(end_effector_point, configuration_parameters)
jacobian_at_the_given_configuration
```

The output produced by running the above code, is shown below.
```py
array([[ 2, -1,  0],
       [-2, -1, -3],
       [ 1,  1,  1]])
```


