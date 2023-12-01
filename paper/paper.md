---
title: 'Automatic Computation for Robot Design (ACRoD): A Python package for numerical calculation of Jacobian of a robot at a given configuration around a specified end-effector point'
tags:
  - Python
  - Robotics
  - Jacobian
  - Kinematics
authors:
  - name: Akkarapakam Suneesh Jacob
    orcid: 0000-0002-9855-9368
    equal-contrib: true
    affiliation: 1 # (Multiple affiliations must be quoted)
    corresponding: true
  - name: Rituparna Datta
    orcid: 0000-0003-3816-2438
    equal-contrib: true # (This is how you can denote equal contributions between multiple authors)
    affiliation: 2
affiliations:
 - name: Indian Institute of Technology Kanpur, Kanpur, India.
   index: 1
 - name: Capgemini Technological Services India Limited, Bengaluru, India.
   index: 2
date: 7 April 2023
bibliography: paper.bib

---

# Summary

Jacobian of a robot refers to the matrix that linearly maps the velocity components of the end-effector and the velocities at the actuated joints. Jacobian is extensively used in dimensional synthesis for Jacobian-based optimal performances of robotic manipulators, in which the optimal dimensional parameters of robots are computed. Determination of accurate mobility [@YANG20081175] of planar and spatial mechanisms can also be performed by using Jacobian in cases where Chebychev–Grübler–Kutzbach criterion cannot [@GOGU2005427] accurately determine the mobility. As a result, Jacobian is a significant part for both kinematic analysis, dimensional synthesis and mobility determination of a mechanism. Hence, the formulation of Jacobian has its key importance in the literature and in the application of performance optimisation along with mobility computation. Formulation of Jacobian for serial manipulators can be computed easily, however, it is increasingly complicated to formulate Jacobian for parallel manipulators due to the existence of passive joint velocities and the nature in which these are related to active joint velocities. Several studies [@606737;@846382;@1220722;@ALTUZARRA20061407] exist for formulation of parallel manipulators but all these studies have dependency on human inspection at some level. Several open source softwares are also available for Jacobian formulation [@nadeau2019pybotics;@Owan2018;@Baumgärtner2022;@Lee2018], but either their application is limited to serial manipulators or they require human intervention or they are part of computationally expensive simulations. For example, TriP [@Baumgärtner2022] does facilitate Jacobian computation with closed-loop chains in the manipulator structure, however, as obvious from the triped robot example in the documentary, human inspection is apparently required to appropriately join the legs of the robot. To alleviate the drawback of the requirement of human inspection, the present research aims to formulate Jacobian that is required for dimensional synthesis for optimal performance around a single point that can be used for any non-redundant manipulator without any dependency on human inspection. Jacob and Dasgupta [@synthesis] used a systematic method as a tool to formulate Jacobian matrices for several manipulators in bulk for performance optimisation around a given task point. However, several steps in that algorithm are not totally computerised but rather human-intervention-dependent. Moreover, it can be applicable only with four types of joints. The current paper extends their method to present a fully computerisable Jacobian formulation algorithm that is applicable for general non-redundant planar and spatial manipulators of any topological structure of seven types of joints, namely revolute, prismatic, cylindrical, spherical, universal, helical and plane joints. Based on this extended method, the Python package `Automatic Computation for Robot Design (ACRoD)` is developed by the authors. `ACRoD` provides a Python-based package for generating functions required to compute Jacobian at a given configuration for a given end-effector point, merely from the simple topological information of the robot in a fully automated manner. This can be directly used in optimisation process to derive optimal dimensions of the robot for optimal performance around a given end-effector point, thereby avoiding many tedious steps in manual formulation, especially when a comparison study is performed on multiple manipulators in bulk [@synthesis].

# Statement of need

For a manipulator of a given topology, designing the dimensions based on optimising Jacobian-based performance parameters (such as manipulability index and condition number [@patel2015manipulator]) around a given end-effector point would require only the topological information for the formulation of Jacobian, as every other step can be automated. Formulation of Jacobian for parallel manipulators and serial-parallel hybrid manipulators are non-trivial, although all the steps of Jacobian formulation even in those cases would have to stem from the mere information of topology of the robot. ACRoD automates the non-trivial formulation of Jacobian systematically. It uses a matrix-based representation of the topology of the robotic manipulator (referred to here as the robot-topology matrix, of which more information is provided [here](https://github.com/suneeshjacob/ACRoD/blob/main/Robot_Topology_Matrix.md)) which is a modified version of the graph adjacency matrix representation [@enumeration] of robotic manipulators. This Jacobian formulation can be used to generate numerical Jacobian matrices with a few random configurations, from which the singular values can be calculated which can confirm the Degree of Freedom (DoF). In other words, the DoF of a given robot topology for a given base link and a given end-effector link can be verified by using this Jacobian function even in cases where Chebychev–Grübler–Kutzbach criterion fails to verify. This can be useful in mechanism synthesis to accurately verify the mobility of a given manipulator directly from its robot-topology matrix by using the method shown in Yang et al.'s paper [@YANG20081175].

`ACRoD` uses NumPy [@numpy] and SymPy [@sympy] packages to generate the functions for Jacobian, which can be directly used in optimisation process to find the optimal dimensional parameters of the robot. The topology of a valid robot (with a single base-link and a single end-effector link and without non-contributing chains) is to be specified using robot-topology matrix in NumPy matrix format. The jacobian class object takes this robot-topology matrix as input argument and generates functions that are required to compute Jacobian. As byproducts, the Jacobian function generation produces the symbolic matrices, the set of independent paths, etc., the sets of active joint velocities and passive joint velocities, etc., which can be accessed from the attributes of the jacobian class object. More technical details on formulation of Jacobian (along with appropriate algorithms) can be found [here](https://github.com/suneeshjacob/ACRoD/blob/main/Mathematics%20behind%20Jacobian%20formulation.md), and the notations and the nomenclature are explained [here](https://github.com/suneeshjacob/ACRoD/blob/main/Notation_and_Nomenclature.md) in detail. The robot-topology matrix representation is explained [here](https://github.com/suneeshjacob/ACRoD/blob/main/Robot_Topology_Matrix.md) in detail. Jacobian formulation for three robot examples, namely the [3R planar serial robot](https://github.com/suneeshjacob/ACRoD/blob/main/examples/Jacobian/maths/3R_serial_robot.md), [a 4R-4P planar serial-parallel hybrid robot](https://github.com/suneeshjacob/ACRoD/blob/main/examples/Jacobian/maths/4R4P_parallel_robot.md) and [an RSSR-SSR spatial parallel robot](https://github.com/suneeshjacob/ACRoD/blob/main/examples/Jacobian/maths/RSSR-SSR_serial-parallel_hybrid_robot.md), are explained in detail in the corresponding hyperlinks.

## Comparisonn with other Jacobian-computation softwares
| Software        | Base Language   | Closed-loop linkages in documentation | Automation level from mere topology        | Primary purpose       |
|-----------------|:---------------:|:-------------------------------------:|--------------------------------------------|-----------------------|
| TriP (2022)     | Python          | yes                                   | Human intervention needed to build robot   | Kinematics            |
| Pybotics (2019) | Python          | no                                    | Automatically buildable from DH parameters |                       |
| DART (2018)     | C++             | X                                     | Human intervention needed to build robot   | 1972                  |
| CoreRobotics    | C++             | X                                     | Human intervention needed to build robot   | 1972                  |
| pinocchio       | C++             | X                                     | Human intervention needed to build robot^* | Jacobian computation  |
| ACRoD (2023)    | Python          | yes                                   | yes                                        | Dimensional Synthesis |

- X = possible but neither documentation is provided nor an example is provided
- ^* = Accepts URDF models (but URDF models are limited to serial manipulators)
- H = Human intervention required

# Acknowledgements

This paper is an extended version of the Jacobian formulation presented in previous work [@synthesis] co-authored by Dr. Bhaskar Dasgupta, whom the authors would like to acknowledge for whatever credits applicable. However, the authors would like to clarify that the technical extension of theory and the whole software are developed solely by the authors of the current paper.

# References
