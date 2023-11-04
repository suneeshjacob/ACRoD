# Formulation of Jacobian for RSSR-SSR spatial parallel manipulator (with a superfluous DOF) as an example

## Figure

<p align="center">
    <img src="./examples/Jacobian/RSSRSSR.png" alt="RSSR-SSR spatial parallel manipulator" width="500px">
</p>

A figure of RRRRPPPP planar serial-parallel hybrid manipulator is shown above. The corresponding adjacency matrix is given by

$$\bf{M} = \left[\begin{matrix}L_1 & R & R & O & R & O\\\\A & L_2 & O & O & O & S\\\\A & O & L_3 & S & O & O\\\\O & O & O & L_4 & O & S\\\\O & O & O & O & L_5 & S\\\\O & O & O & O & O & L_6\end{matrix}\right]$$

## Usage

### Jacobian for planar manipulators

The topological information of a robot is to be specified by using its robot-topology matrix, as defined [here](Robot_Topology_Matrix.md). For RRRRPPPP planar serial-parallel hybrid manipulator shown above, the robot topology matrix is given by

$$\left[\begin{matrix}9 & 1 & 1 & 0 & 1 & 0\\\\1 & 9 & 0 & 0 & 0 & 4\\\\1 & 0 & 9 & 4 & 0 & 0\\\\0 & 0 & 0 & 9 & 0 & 4\\\\0 & 0 & 0 & 0 & 9 & 4\\\\0 & 0 & 0 & 0 & 0 & 9\end{matrix}\right]$$

The corresponding Jacobian function can be formulated as follows.

Firstly, the required functions are imported as shown below.
```py
from acrod.functions import jacobian
from numpy import array
```


The robot-topology matrix for 3R planar serial manipulator is defined and jacobian information is processed via the imported jacobian class as follows.
```py
M = array(
        [[9, 1, 1, 0, 1, 0],
         [1, 9, 0, 0, 0, 4],
         [1, 0, 9, 4, 0, 0],
         [0, 0, 0, 9, 0, 4],
         [0, 0, 0, 0, 9, 4],
         [0, 0, 0, 0, 0, 9]]
    )
jac = jacobian(M)
```


Jacobian function is generated as shown below.
```py
jacobian_function = jac.get_jacobian_function()
```


In the process of generating the above jacobian function, other attributes of the jacobian object also are updated. Symbolic Jacobian matrices can be extracted from the attributes. Since this is a non-serial robot, there would be four matrices required to compute the Jacobian, which are $J_a$, $J_p$, $A_a$ and $A_p$. These can be extracted by the attributes `Ja`, `Jp`, `Aa` and `Ap`, respectively, as shown below.

```py
symbolic_Ja = jac.Ja
symbolic_Ja
```

Output in Jupyter notebook:

$$\begin{bmatrix}- \left(a_{y} - r_{(1,2)y}\right) \cos{\left(\beta_{(1,2)} \right)} + \left(a_{z} - r_{(1,2)z}\right) \sin{\left(\beta_{(1,2)} \right)} \sin{\left(\phi_{(1,2)} \right)} & 0 \\\\
\left(a_{x} - r_{(1,2)x}\right) \cos{\left(\beta_{(1,2)} \right)} - \left(a_{z} - r_{(1,2)z}\right) \sin{\left(\beta_{(1,2)} \right)} \cos{\left(\phi_{(1,2)} \right)} & 0 \\\\
\left(a_{x} - r_{(1,2)x}\right) \sin{\left(\beta_{(1,2)} \right)} \sin{\left(\phi_{(1,2)} \right)} + \left(a_{y} - r_{(1,2)y}\right) \sin{\left(\beta_{(1,2)} \right)} \cos{\left(\phi_{(1,2)} \right)} & 0 \\\\
\sin{\left(\beta_{(1,2)} \right)} \cos{\left(\phi_{(1,2)} \right)} & 0 \\\\
\sin{\left(\beta_{(1,2)} \right)} \sin{\left(\phi_{(1,2)} \right)} & 0 \\\\
\cos{\left(\beta_{(1,2)} \right)} & 0\end{bmatrix}$$

```py
symbolic_Jp = jac.Jp
symbolic_Jp
```
Output in Jupyter notebook:

$$\left[\begin{array}{ccccccccccccc}0 & 0 & a_{z} - r_{(2,6)z} & - a_{y} + r_{(2,6)y} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\\\0 & - a_{z} + r_{(2,6)z} & 0 & a_{x} - r_{(2,6)x} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\\\0 & a_{y} - r_{(2,6)y} & - a_{x} + r_{(2,6)x} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\\\0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\\\0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\\\0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\end{array}\right]$$

```py
symbolic_Aa = jac.Aa
symbolic_Aa
```
Output in Jupyter notebook:

$$\left[\begin{matrix}\left(a_{y} -r_{(1,2)y}\right) \cos{\left(\beta_{(1,2)} \right)} -\left(a_{z} -r_{(1,2)z}\right) \sin{\left(\beta_{(1,2)} \right)} \sin{\left(\phi_{(1,2)} \right)} & 0\\\\-\left(a_{x} -r_{(1,2)x}\right) \cos{\left(\beta_{(1,2)} \right)} + \left(a_{z} -r_{(1,2)z}\right) \sin{\left(\beta_{(1,2)} \right)} \cos{\left(\phi_{(1,2)} \right)} & 0\\\\\left(a_{x} -r_{(1,2)x}\right) \sin{\left(\beta_{(1,2)} \right)} \sin{\left(\phi_{(1,2)} \right)} -\left(a_{y} -r_{(1,2)y}\right) \sin{\left(\beta_{(1,2)} \right)} \cos{\left(\phi_{(1,2)} \right)} & 0\\\\\left(a_{y} -r_{(1,2)y}\right) \cos{\left(\beta_{(1,2)} \right)} -\left(a_{z} -r_{(1,2)z}\right) \sin{\left(\beta_{(1,2)} \right)} \sin{\left(\phi_{(1,2)} \right)} & -\left(a_{y} -r_{(1,3)y}\right) \cos{\left(\beta_{(1,3)} \right)} + \left(a_{z} -r_{(1,3)z}\right) \sin{\left(\beta_{(1,3)} \right)} \sin{\left(\phi_{(1,3)} \right)}\\\\-\left(a_{x} -r_{(1,2)x}\right) \cos{\left(\beta_{(1,2)} \right)} + \left(a_{z} -r_{(1,2)z}\right) \sin{\left(\beta_{(1,2)} \right)} \cos{\left(\phi_{(1,2)} \right)} & \left(a_{x} -r_{(1,3)x}\right) \cos{\left(\beta_{(1,3)} \right)} -\left(a_{z} -r_{(1,3)z}\right) \sin{\left(\beta_{(1,3)} \right)} \cos{\left(\phi_{(1,3)} \right)}\end{matrix}\right]$$
```py
symbolic_Ap = jac.Ap
symbolic_Ap
```
Output in Jupyter notebook:

$$\left[\begin{array}{ccccccccccccc}\- \left(a_{y} \- r_{(1,5)y}\right) \cos{\left(\beta_{(1,5)} \right)} + \left(a_{z} \- r_{(1,5)z}\right) \sin{\left(\beta_{(1,5)} \right)} \sin{\left(\phi_{(1,5)} \right)} & 0 & \- a_{z} + r_{(2,6)z} & a_{y} \- r_{(2,6)y} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & a_{z} \- r_{(5,6)z} & \- a_{y} + r_{(5,6)y}\\\\\left(a_{x} \- r_{(1,5)x}\right) \cos{\left(\beta_{(1,5)} \right)} \- \left(a_{z} \- r_{(1,5)z}\right) \sin{\left(\beta_{(1,5)} \right)} \cos{\left(\phi_{(1,5)} \right)} & a_{z} \- r_{(2,6)z} & 0 & \- a_{x} + r_{(2,6)x} & 0 & 0 & 0 & 0 & 0 & 0 & \- a_{z} + r_{(5,6)z} & 0 & a_{x} \- r_{(5,6)x}\\\\\- \left(a_{x} \- r_{(1,5)x}\right) \sin{\left(\beta_{(1,5)} \right)} \sin{\left(\phi_{(1,5)} \right)} + \left(a_{y} \- r_{(1,5)y}\right) \sin{\left(\beta_{(1,5)} \right)} \cos{\left(\phi_{(1,5)} \right)} & \- a_{y} + r_{(2,6)y} & a_{x} \- r_{(2,6)x} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & a_{y} \- r_{(5,6)y} & \- a_{x} + r_{(5,6)x} & 0\\\\0 & 0 & \- a_{z} + r_{(2,6)z} & a_{y} \- r_{(2,6)y} & 0 & a_{z} \- r_{(3,4)z} & \- a_{y} + r_{(3,4)y} & 0 & a_{z} \- r_{(4,6)z} & \- a_{y} + r_{(4,6)y} & 0 & 0 & 0\\\\0 & a_{z} \- r_{(2,6)z} & 0 & \- a_{x} + r_{(2,6)x} & \- a_{z} + r_{(3,4)z} & 0 & a_{x} \- r_{(3,4)x} & \- a_{z} + r_{(4,6)z} & 0 & a_{x} \- r_{(4,6)x} & 0 & 0 & 0\\\\0 & \- a_{y} + r_{(2,6)y} & a_{x} \- r_{(2,6)x} & 0 & a_{y} \- r_{(3,4)y} & \- a_{x} + r_{(3,4)x} & 0 & a_{y} \- r_{(4,6)y} & \- a_{x} + r_{(4,6)x} & 0 & 0 & 0 & 0\\\\\sin{\left(\beta_{(1,5)} \right)} \cos{\left(\phi_{(1,5)} \right)} & -1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0\\\\\sin{\left(\beta_{(1,5)} \right)} \sin{\left(\phi_{(1,5)} \right)} & 0 & -1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0\\\\\cos{\left(\beta_{(1,5)} \right)} & 0 & 0 & -1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1\\\\0 & -1 & 0 & 0 & 1 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0\\\\0 & 0 & -1 & 0 & 0 & 1 & 0 & 0 & 1 & 0 & 0 & 0 & 0\\\\0 & 0 & 0 & -1 & 0 & 0 & 1 & 0 & 0 & 1 & 0 & 0 & 0\\\\0 & 0 & 0 & 0 & r_{(3,4)x} \- r_{(4,6)x} & r_{(3,4)y} \- r_{(4,6)y} & r_{(3,4)z} \- r_{(4,6)z} & 0 & 0 & 0 & 0 & 0 & 0\end{array}\right]$$

The above Jacobian is based on the notations defined and described [here](Notation_and_Nomenclature.md).

Active joint velocities, in the corresponding order, can be viewed by running the following lines.
```py
active_joint_velocities = jac.active_joint_velocities_symbolic
active_joint_velocities
```

In an ipynb file of JupyterLab, the above code would produce the following output.

$$\left[\begin{matrix}\dot{d}\_{(1,2)} \\\\ \dot{d}\_{(1,3)} \\\\ \dot{\theta}\_{(4,5)}\end{matrix}\right]$$

Robot dimensional parameters can be viewed by running the below line.
```py
robot_dimensional_parameters = jac.parameters_symbolic
robot_dimensional_parameters
```

In an ipynb file of JupyterLab, the above code would produce the following output.

$$\left[\begin{matrix}\phi_{(1,2)}\\\\\phi_{(1,3)}\\\\\phi_{(2,4)}\\\\\phi_{(3,4)}\\\\r_{(4,5)x}\\\\r_{(4,5)y}\\\\r_{(4,6)x}\\\\r_{(4,6)y}\\\\r_{(5,7)x}\\\\r_{(5,7)y}\\\\r_{(6,7)x}\\\\r_{(6,7)y}\end{matrix}\right]$$


Robot end-effector parameters can be viewed by running the below line.
```py
robot_endeffector_parameters = jac.endeffector_variables_symbolic
robot_endeffector_parameters
```

In an ipynb file of JupyterLab, the above code would produce the following output.

$$\left[\begin{matrix}a_{x} \\\\ a_{y}\end{matrix}\right]$$

#### Sample computation of Jacobian for the configuration corresponding to the parameters shown below:

- End-effector point: $\textbf{a}=\hat{i}+2\hat{j}$
- Locations of joints: $\textbf{r}\_{(4,5)}=2\hat{i}+8\hat{j}$, $\textbf{r}\_{(4,6)}=6\hat{i}+8\hat{j}$, $\textbf{r}\_{(5,7)}=3\hat{i}+10\hat{j}$ and $\textbf{r}\_{(6,7)}=5\hat{i}+10\hat{j}$
- Orientations of joints: $\phi\_{(1,2)}=2\pi/3$, $\phi\_{(1,3)}=\pi/3$, $\phi\_{(2,4)}=\pi/3$ and $\phi\_{(3,4)}=2\pi/3$

For the given set of dimensional parameters of the robot, the numerical Jacobian can be computed as follows. Firstly, we need to gather the configuration parameters in Python list format, in a particular order. The robot dimensional parameters from `jac.parameters_symbolic` are found (as shown earlier) to be in the order of $\phi_{(1,2)}$, $\phi_{(1,3)}$, $\phi_{(2,4)}$, $\phi_{(3,4)}$, $r_{(4,5)x}$, $r_{(4,5)y}$, $r_{(4,6)x}$, $r_{(4,6)y}$, $r_{(5,7)x}$, $r_{(5,7)y}$, $r_{(6,7)x}$ and $r_{(6,7)y}$. Hence the configuration parameters are to be supplied in the same order, as a list. Thus, the computation can be performed as shown below.
```py
from numpy import pi

end_effector_point = [1,2]
configuration_parameters = [2*pi/3,pi/3,pi/3,2*pi/3,2,8,6,8,3,10,5,10]
jacobian_at_the_given_configuration = jacobian_function(end_effector_point, configuration_parameters)
jacobian_at_the_given_configuration
```

The output produced by running the above code, is shown below.
```py
array([[ -0.5      ,   0.5      , -10.       ],
       [  0.8660254,   0.8660254,   3.       ],
       [  0.       ,   0.       ,  -1.       ]])
```

#### Accessing each matrix individually:

Each of $J_a$, $J_p$, $A_a$ and $A_p$ functions can be accessed as shown below

```py
numerical_Ja = jac.Ja_func(end_effector_point, configuration_parameters)
numerical_Ja
```
Output:
```py
array([[-0.5      ,  0.       ,  6.       ],
       [ 0.8660254,  0.       , -1.       ],
       [ 0.       ,  0.       ,  1.       ]])
```

```py
numerical_Jp = jac.Jp_func(end_effector_point, configuration_parameters)
numerical_Jp
```
Output:
```py
array([[ 0.5      ,  0.       ,  0.       ,  8.       ,  0.       ],
       [ 0.8660254,  0.       ,  0.       , -2.       ,  0.       ],
       [ 0.       ,  0.       ,  0.       ,  1.       ,  0.       ]])
```

```py
numerical_Aa = jac.Aa_func(end_effector_point, configuration_parameters)
numerical_Aa
```
Output:
```py
array([[ 0.       ,  0.       , -6.       ],
       [ 0.       ,  0.       ,  1.       ],
       [ 0.5      ,  0.5      ,  0.       ],
       [-0.8660254,  0.8660254,  0.       ],
       [ 0.       ,  0.       , -1.       ]])
```

```py
numerical_Ap = jac.Ap_func(end_effector_point, configuration_parameters)
numerical_Ap
```
Output:
```py
array([[ 0.       ,  0.       ,  6.       , -8.       ,  8.       ],
       [ 0.       ,  0.       , -5.       ,  2.       , -4.       ],
       [-0.5      , -0.5      ,  0.       ,  0.       ,  0.       ],
       [-0.8660254,  0.8660254,  0.       ,  0.       ,  0.       ],
       [ 0.       ,  0.       ,  1.       , -1.       ,  1.       ]])
```

And the computation $J_a-J_pA^{-1}_pA_a$ is given by
```py
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
[[0, 1, 3, 4, 6], [0, 1, 3, 5, 6], [0, 2, 3, 4, 6], [0, 2, 3, 5, 6]]
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
[[0, 1, 3, 4, 6], [0, 1, 3, 5, 6], [0, 2, 3, 4, 6]]
```
For independent paths pertaining to angular velocities:
```py
jac.P_tilde_omega
```
Output:
```py
[[0, 1, 3, 4, 6], [0, 1, 3, 5, 6]]
```
The above scripts give the lists of all the independent connecting paths pertaining to linear and angular velocities (only link numbers are shown, indexed from 0).
