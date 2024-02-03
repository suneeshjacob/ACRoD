URDF support
============

URDF to Robot-topology Matrix
-----------------------------

Support for getting the robot's topology from a URDF file is provided in this section.

URDF files are often used to describe serial robots. These files could describe not only the topological connectivity of links with joints but also the positions and orientations of joints and the joint limits (if applicable), etc. In order to extract the topological information from a URDF file, the below script can be used.

.. code:: py

   from acrod.urdf_to_robottopologymatrix import from_urdf_to_matrix

   urdf_filepath = 'path/to/urdf/file'
   M = from_urdf_to_matrix(urdf_filepath)

This would automatically extract the robot-topology matrix from the URDF file located at the specified file path in the local disc.

Limitations
-----------

There are some limitations for URDF support at the moment. Firstly, ACRoD in its current form considers only those robots that have a single base link and a single end-effector link. Hence, `from_urdf_to_matrix` assumes that the supplied URDF file refers to a robot with a single base link and a single end-effector link.