# Formulation of Jacobian for RRR planar serial manipulator (with a superfluous DOF) as an example

## Figure

<p align="center">
    <img src="./examples/Jacobian/RRR.png" alt="RRR planar serial manipulator" width="500px">
</p>

A figure of RRR planar serial manipulator is shown above. The corresponding adjacency matrix is given by

$$\bf{M} = \left[\begin{matrix}L_1 & R & O & O \\\\A & L_2 & R & O\\\\O & A & L_3 & R\\\\O & O & A & L_4\end{matrix}\right]$$

## Usage

### Jacobian for planar manipulators

The topological information of a robot is to be specified by using its robot-topology matrix, as defined [here](Robot_Topology_Matrix.md). For RRRRPPPP planar serial-parallel hybrid manipulator shown above, the robot topology matrix is given by

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

$$\left[\begin{matrix}a_{x}\\\\a_{y}\\\\a_{z}\end{matrix}\right]$$

#### Sample computation of Jacobian for the configuration corresponding to the parameters shown below:

- End-effector point: $\textbf{a}=\hat{i}+2\hat{j}+3\hat{k}$
- Locations of joints: $\textbf{r}\_{(1,2)}=2\hat{i}+8\hat{j}+3\hat{k}$, $\textbf{r}\_{(1,3)}=1\hat{i}+2\hat{j}+5\hat{k}$, $\textbf{r}\_{(1,5)}=2\hat{i}+4\hat{j}+7\hat{k}$, $\textbf{r}\_{(2,6)}=3\hat{i}+1\hat{j}+2\hat{k}$, $\textbf{r}\_{(3,4)}=6\hat{i}+8\hat{j}+4\hat{k}$, $\textbf{r}\_{(4,6)}=8\hat{i}+1\hat{j}+3\hat{k}$ and $\textbf{r}\_{(5,6)}=5\hat{i}+7\hat{j}+3\hat{k}$
- Orientations of joints: $\beta\_{(1,2)}=\pi/6$, $\phi\_{(1,2)}=\pi/3$, $\beta\_{(1,3)}=5\pi/6$, $\phi\_{(1,3)}=2\pi/3$, $\beta\_{(1,5)}=\pi/12$ and $\phi\_{(1,5)}=\pi/2$.

For the given set of dimensional parameters of the robot, the numerical Jacobian can be computed as follows. Firstly, we need to gather the configuration parameters in Python list format, in a particular order. The robot dimensional parameters from `jac.parameters_symbolic` are found (as shown earlier) to be in the order of $\phi_{(1,2)}$, $\phi_{(1,3)}$, $\phi_{(2,4)}$, $\phi_{(3,4)}$, $r_{(4,5)x}$, $r_{(4,5)y}$, $r_{(4,6)x}$, $r_{(4,6)y}$, $r_{(5,7)x}$, $r_{(5,7)y}$, $r_{(6,7)x}$ and $r_{(6,7)y}$. Hence the configuration parameters are to be supplied in the same order, as a list. Thus, the computation can be performed as shown below.
```py
from numpy import pi

end_effector_point = [1,2,3]
configuration_parameters = [2, 8, 3, pi/6, pi/3, 1, 2, 5, 5*pi/6, 2*pi/3, 2, 4, 7, pi/12, pi/2, 3, 1, 2, 6, 8, 4, 8, 1, 3, 5, 7, 3]
jacobian_at_the_given_configuration = jacobian_function(end_effector_point, configuration_parameters)
jacobian_at_the_given_configuration
```

The output produced by running the above code, is shown below.
```py
array([[ 19.62196517,  10.77434097],
       [-16.1460825 ,  -8.61947278],
       [ 43.06469529,  30.16815472],
       [  6.57574977,   4.30973639],
       [ 19.33597911,  12.92920917],
       [  5.34317907,   2.15486819]])
```

#### Accessing each matrix individually:

Each of $J_a$, $J_p$, $A_a$ and $A_p$ functions can be accessed as shown below

```py
numerical_Ja = jac.Ja_func(end_effector_point, configuration_parameters)
numerical_Ja
```
Output:
```py
array([[ 5.19615242,  0.        ],
       [-0.8660254 ,  0.        ],
       [-1.0669873 ,  0.        ],
       [ 0.25      ,  0.        ],
       [ 0.4330127 ,  0.        ],
       [ 0.8660254 ,  0.        ]])
```

```py
numerical_Jp = jac.Jp_func(end_effector_point, configuration_parameters)
numerical_Jp
```
Output:
```py
array([[ 0,  0,  1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0],
       [ 0, -1,  0, -2,  0,  0,  0,  0,  0,  0,  0,  0,  0],
       [ 0,  1,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
       [ 0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
       [ 0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
       [ 0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0]])
```

```py
numerical_Aa = jac.Aa_func(end_effector_point, configuration_parameters)
numerical_Aa
```
Output:
```py
array([[-5.19615242,  0.        ],
       [ 0.8660254 ,  0.        ],
       [ 1.0669873 ,  0.        ],
       [-5.19615242, -0.8660254 ],
       [ 0.8660254 , -0.5       ],
       [ 1.0669873 , -0.        ],
       [-0.25      ,  0.        ],
       [-0.4330127 ,  0.        ],
       [-0.8660254 ,  0.        ],
       [-0.25      , -0.25      ],
       [-0.4330127 ,  0.4330127 ],
       [-0.8660254 , -0.8660254 ],
       [ 0.        ,  2.66506351]])
```

```py
numerical_Ap = jac.Ap_func(end_effector_point, configuration_parameters)
numerical_Ap.round()
```
Output:
```py
array([[ 1.,  0., -1.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  5.],
       [-1.,  1.,  0.,  2.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., -4.],
       [ 0., -1., -2.,  0.,  0.,  0.,  0.,  0.,  0.,  0., -5.,  4.,  0.],
       [ 0.,  0., -1.,  1.,  0., -1.,  6.,  0.,  0., -1.,  0.,  0.,  0.],
       [ 0.,  1.,  0.,  2.,  1.,  0., -5.,  0.,  0., -7.,  0.,  0.,  0.],
       [ 0., -1., -2.,  0., -6.,  5.,  0.,  1.,  7.,  0.,  0.,  0.,  0.],
       [ 0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.],
       [ 0.,  0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.],
       [ 1.,  0.,  0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  1.],
       [ 0., -1.,  0.,  0.,  1.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.],
       [ 0.,  0., -1.,  0.,  0.,  1.,  0.,  0.,  1.,  0.,  0.,  0.,  0.],
       [ 0.,  0.,  0., -1.,  0.,  0.,  1.,  0.,  0.,  1.,  0.,  0.,  0.],
       [ 0.,  0.,  0.,  0., -2.,  7.,  1.,  0.,  0.,  0.,  0.,  0.,  0.]])
```

And the computation $J_a-J_pA^{-1}_pA_a$ is given by
```py
import numpy
numerical_Ja-numpy.matmul(numpy.matmul(numerical_Jp,numpy.linalg.inv(numerical_Ap)),numerical_Aa)
```
which gives the same output as `jacobian_at_the_given_configuration`.


### Some other attributes

To get the computed list of all connecting paths from the base link to the end-effector link, the below script can be used:

```py
jac.P
```
Output:
```py
[[0, 1, 5], [0, 4, 5], [0, 2, 3, 5]]
```
which gives the list of all connecting paths (only link numbers are shown, indexed from 0).

#### Independent paths pertaining to linear and angular velocities:

Out of the above paths, the computed list of independent paths pertaining to linear velocities and angular velocities, can be accessed by the below scripts:

For independent paths pertaining to linear velocities:
```py
jac.P_tilde
```
Output:
```py
[[0, 1, 5], [0, 4, 5], [0, 2, 3, 5]]
```
For independent paths pertaining to angular velocities:
```py
jac.P_tilde_omega
```
Output:
```py
[[0, 1, 5], [0, 4, 5], [0, 2, 3, 5]]
```

#### Superfluous DOF information:

This manipulator has a superfluous DOF. The information of all the superfluous DOFs in the manipulator can be retrieved in a list by the script shown below:

```py
superfluous_dof_information = jac.superfluous_dof_information
len(superfluous_dof_information)
```
Output:
```py
1
```
The above output shows that the number of superfluous DOF is 1. In order to access the information of this superfluous DOF, the script shown below can be used:
```py
c_be, [(i,j),(k,l)] = superfluous_dof_information[0]
[(i,j),(k,l)]
```
Output:
```py
[(2, 3), (3, 5)]
```
The above output shows that the superfluous DOF exists between the spherical joints joined by links 3,4 and 4,6 (since the link numbers are indexed from zero). And `c_be` represents the list of link numbers of the part that contains base and end-effector links (as explained here). The complement list of `c_be` can be computed as shown below.

```py
c_be_complement = [i for i in range(len(M)) if i not in c_be]
c_be_complement
```
Output:
```py
[3]
```
This shows that it is the link 4 that exhibits a superfluous DOF, which is evident from the figure in the beginning of this document.

In the above results, the link numbers are indexed from 0 (not 1).
