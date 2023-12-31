3R Planar Serial Robot
======================

Figure
------

.. image:: ../examples/Jacobian/images/RRR.png
   :alt: Alternative Text
   :width: 300
   :align: center

A figure of RRR planar serial manipulator is shown above. The
corresponding adjacency matrix is given by

.. math:: \bf{M} = \left[\begin{matrix}L_1 & R & O & O \\A & L_2 & R & O\\O & A & L_3 & R\\O & O & A & L_4\end{matrix}\right]

Usage
-----

Jacobian for planar manipulators
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The topological information of a robot is to be specified by using its
robot-topology matrix, as defined
`here <../../../misc/Robot_Topology_Matrix.md>`__. For RRR planar serial
manipulator shown above, the robot topology matrix is given by

.. math:: \left[\begin{matrix}9 & 1 & 0 & 0 \\ 1 & 9 & 1 & 0 \\ 0 & 1 & 9 & 1 \\ 0 & 0 & 1 & 9\end{matrix}\right]

The corresponding Jacobian function can be formulated as follows.

Firstly, the required functions are imported as shown below.

.. code:: py

   from acrod.jacobian import Jacobian
   from numpy import array

The robot-topology matrix for 3R planar serial manipulator is defined
and jacobian information is processed via the imported jacobian class as
follows.

.. code:: py

   M = array(
       [[9, 1, 0, 0],
        [1, 9, 1, 0],
        [0, 1, 9, 1],
        [0, 0, 1, 9]]
   )
   jac = Jacobian(M, robot_type='planar')

Jacobian function is generated as shown below.

.. code:: py

   jacobian_function = jac.get_jacobian_function()

In the process of generating the above jacobian function, other
attributes of the jacobian object also are updated. Symbolic Jacobian
matrices can be extracted from the attributes. Since this is a serial
robot, the matrix :math:`J_a` itself would be the Jacobian matrix of the
manipulator. The matrix :math:`J_a` is extracted from ``Ja`` attribute
of the jacobian object as follows.

.. code:: py

   symbolic_jacobian = jac.Ja
   symbolic_jacobian

In an ipynb file of JupyterLab, the above code would produce the
following output.

.. math:: \left[\begin{matrix}- a_{y} + r_{(1,2)y} & - a_{y} + r_{(2,3)y} & - a_{y} + r_{(3,4)y}\\a_{x} - r_{(1,2)x} & a_{x} - r_{(2,3)x} & a_{x} - r_{(3,4)x}\\1 & 1 & 1\end{matrix}\right]

The above Jacobian is based on the notations defined and described
`here <../../../misc/Notation_and_Nomenclature.md>`__.

Active joint velocities, in the corresponding order, can be viewed by
running the following lines.

.. code:: py

   active_joint_velocities = jac.active_joint_velocities_symbolic
   active_joint_velocities

In an ipynb file of JupyterLab, the above code would produce the
following output.

.. math:: \left[\begin{matrix}\dot{\theta}_{(1,2)}\\\dot{\theta}_{(2,3)}\\\dot{\theta}_{(3,4)}\end{matrix}\right]

Robot dimensional parameters can be viewed by running the below line.

.. code:: py

   robot_dimensional_parameters = jac.parameters_symbolic
   robot_dimensional_parameters

In an ipynb file of JupyterLab, the above code would produce the
following output.

.. math:: \left[\begin{matrix}r_{(1,2)x}\\r_{(1,2)y}\\r_{(2,3)x}\\r_{(2,3)y}\\r_{(3,4)x}\\r_{(3,4)y}\end{matrix}\right]

Robot end-effector parameters can be viewed by running the below line.

.. code:: py

   robot_endeffector_parameters = jac.endeffector_variables_symbolic
   robot_endeffector_parameters

In an ipynb file of JupyterLab, the above code would produce the
following output.

.. math:: \left[\begin{matrix}a_{x}\\a_{y}\end{matrix}\right]

Sample computation of Jacobian for the configuration corresponding to the parameters shown below:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

-  End-effector point: :math:`\textbf{a}=\hat{i}+2\hat{j}`
-  Locations of joints: :math:`\textbf{r}_{(1,2)}=3\hat{i}+4\hat{j}`,
   :math:`\textbf{r}_{(2,3)}=2\hat{i}+1\hat{j}` and
   :math:`\textbf{r}_{(3,4)}=4\hat{i}+2\hat{j}`

For the given set of dimensional parameters of the robot, the numerical
Jacobian can be computed as follows. Firstly, we need to gather the
configuration parameters in Python list format, in a particular order.
The robot dimensional parameters from ``jac.parameters_symbolic`` are
found (as shown earlier) to be in the order of :math:`r_{(1,2)x}`,
:math:`r_{(1,2)y}`, :math:`r_{(2,3)x}`, :math:`r_{(2,3)y}`,
:math:`r_{(3,4)x}` and :math:`r_{(3,4)y}`. Hence the configuration
parameters are to be supplied in the same order, as a list. Thus, the
computation can be performed as shown below.

.. code:: py

   end_effector_point = [1,2]
   configuration_parameters = [3, 4, 2, 1, 4, 2]
   jacobian_at_the_given_configuration = jacobian_function(end_effector_point, configuration_parameters)
   jacobian_at_the_given_configuration

The output produced by running the above code, is shown below.

.. code:: py

   array([[ 2, -1,  0],
          [-2, -1, -3],
          [ 1,  1,  1]])

