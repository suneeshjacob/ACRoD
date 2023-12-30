.. ACRoD documentation master file, created by
   sphinx-quickstart on Sat Dec 30 17:00:29 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to ACRoD's documentation!
=================================

Ever felt tedious of computations required for designing robots? ACRoD is dedicated to develop functions for automatic computations for designing robotic manipulators.

Currently available features
----------------------------

- Jacobian formulation for planar and spatial manipulators around a given end-effector point. (This is useful in performing optimisation of Jacobian-based performance parameters of any non-redundant robot directly from its robot-topology matrix)
   - Statement of need: Jacobian formulation is highly used in dimensional synthesis of robotic manipulators, which deals with optimal design of robot's dimensional parameters (link-lengths and joint-orientations). For a given topological structure, formulating Jacobian around a given end-effector point for designing optimal dimensional parameters would require only the topological information, as every other step can be automated. Formulating Jacobian for serial manipulators is easy but for parallel manipulators and serial-parallel hybrid manipulators it is more complicated and often tedious. ACRoD automates this task of formulating Jacobian for a given end-effector point by running all the required steps in the background. The targetted audience includes researchers and engineers working on performing dimensional synthesis of manipulators (especially for multiple manipultors in bulk for comparison) to compute optimal dimensions for operation around a single end-effector point, and those needing to verify the DOF of a topological structure (especially for those special cases of kinematic mechanisms where Chebychev–Grübler–Kutzbach criterion fails to accurately determine the DOF) by analysing the Jacobian.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   installation_and_usage
   notation_and_nomenclature
   robot_topology_matrix
   mathematics_behind_jacobian_formulation
   community_guidelines
   modules

.. toctree::
   :maxdepth: 2
   :caption: Examples:

   examples

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


