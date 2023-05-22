---
title: 'Automatic Computation for Robot Design (ACRoD): A Python package for numerically calculating the Jacobian of a robot around a single end-effector point for optimisation of performance around the end-effector point.'
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

Jacobian is extensively used in dimensional synthesis for Jacobian-based optimal performances of robotic manipulators, in which the optimal dimensional parameters of robots are computed. Determination of accurate mobility [@YANG20081175] of planar and spatial mechanisms can also be performed by using Jacobian in cases where Chebychev–Grübler–Kutzbach criterion cannot [@GOGU2005427] accurately determine the mobility. As a result, Jacobian is a significant part for both kinematic analysis, dimensional synthesis and mobility determination of a mechanism. Hence, the formulation of Jacobian has its key importance in the literature and in the application of performance optimisation along with mobility computation. Formulation of Jacobian for serial manipulators can be computed easily, however, it is increasingly complicated to formulate Jacobian for parallel manipulators due to the existence of passive joint velocities and the nature in which they are related to active joint velocities. Several studies [@606737;@846382;@1220722;@ALTUZARRA20061407] exist for formulation of parallel manipulators but they happen to require human inspection at some level. Several open source softwares are available for Jacobian formulation [@nadeau2019pybotics;@Owan2018;@Baumgärtner2022;@Lee2018], but they are either limited to serial manipulators or require human intervention or part of computationally expensive simulations. To alleviate the drawback of human inspection, the present research aims to formulate Jacobian that is required for dimensional synthesis for optimal performance around a single point that can be used for any non-redundant manipulator without any dependency on human inspection. Jacob et al. [@synthesis] used a systematic method as a tool to formulate Jacobian matrices for several manipulators in bulk for performance optimisation around a given task point. However, several steps in that algorithm are not totally computerised but rather human-intervention-dependent. Moreover, it can be applicable only with four types of joints. The current paper extends their method to present a fully computerisable Jacobian formulation algorithm that is applicable for general non-redundant planar and spatial manipulators of any topological structure of seven types of joints.

# Statement of need

`ACRoD` provides a Python-based package for generating functions required to compute Jacobian at a given configuration for a given end-effector point. For a manipulator of a given topology, designing the dimensions based on optimal dexterous performance around a given end-effector point would require only the topological information for the formulation of Jacobian, and every other step can be automated. Formulation of Jacobian for parallel manipulators and serial-parallel hybrid manipulators are non-trivial, although all the steps of Jacobian formulation even in those cases would have to stem from the mere information of topology of the robot. This package automates the non-trivial formulation of Jacobian systematically. It uses a matrix-based representation of the topology of the robotic manipulator (referred to here as the robot-topology matrix, of which more information is provided [here](https://github.com/suneeshjacob/ACRoD/blob/main/Robot_Topology_Matrix.md)) which is a modified version of the graph adjacency matrix representation [@enumeration] of robotic manipulators.

`ACRoD` uses NumPy [@numpy] and SymPy [@sympy] packages to generate the functions for Jacobian, which can be directly used in optimisation process to find the optimal dimensional parameters of the robot. The topology of a valid robot (with a single base-link and a single end-effector link and without non-contributing chains) is to be specified using robot-topology matrix in NumPy matrix format.

# Mathematics

For a serial manipulator, the contributions of the velocity of the end-effector from each joint (along the serial path from the base link to the end-effector link) can be summed up to get the velocity of the end-effector. Hence, Jacobian formulation for serial manipulators is very straight-forward, as there would be just one connecting path (of links through joints) from the base-link to the end-effector link and each joint velocity would be an active joint velocity (as serial manipulators do not have passive joint velocities). The approach is that is typically used to formulate Jacobian of a serial manipulator is to simply calculate the individual contributions of each of the joints to the end-effector along the path that connects the base-link and the end-effector link. However, for non-serial manipulators (including parallel manipulators and serial-parallel hybrid manipulators), more than one connecting path would exist from the base link to the end-effector link, and passive joint velocities come into picture. Hence, for parallel manipulators, the same concept can be used [@synthesis] for all the connecting paths from the base-link to the end- effector link. But not all joint velocities through these paths are actually independent but instead the passive joint velocities are supposed to be dependent on the active joint velocities. Additionally, it is desirable for the paths to be independent in order to avoid the case of an over-determined system. Hence, the procedure would be to take the robot-topology matrix and identify all the independent connecting paths in it for both linear and angular velocities independently, and to formulate linear and angular velocities of the end-effector through these paths and to form constraint equations by which the passive joint velocities can be expressed in terms of active joint velocities, and to finally calculate the end-effector's velocity in terms of active joint velocities alone, from which the Jacobian matrix can be extracted. Since the relationship between active and passive joint velocities is always linear, the passive joint velocities can be expressed in terms of active joint velocities by using constraint equations that can be derived from velocities, as shown in \autoref{eq:constraintequations}.

\begin{equation}\label{eq:velocity}\bm{v} = \mathbf{J}\mathbf{\Omega} = \mathbf{J_a}\mathbf{\Omega_a}+\mathbf{J_p}\mathbf{\Omega_p}\end{equation}

\begin{equation}\label{eq:constraintequations}\mathbf{A}\mathbf{\\Omega} = \mathbf{A_a}\mathbf{\\Omega_a}+\mathbf{A_p}\mathbf{\\Omega_p}=0\end{equation}

The above steps of the algorithm (to formulate Jacobian of a typical non-serial manipulator) are concisely shown below.

1. Identification of all connecting paths from the base link to the end-effector link.
1. Reduction of the list of paths to a list of independent paths for linear velocities.
1. Further reduction of paths to non-trivial ones in the context of angular velocities.
1. Calculation of the contributions of linear velocities through each of the independent paths and the contributions of angular velocities through each of the independent non-trivial paths, and calculation of their sums for each path.
1. Formulation of the velocity of the end-effector as shown in \autoref{eq:velocity} by using one of the sums (for both linear and angular velocities) formulated in the previous step.
1. Formulation of constraint equations as shown in \autoref{eq:constraintequations} by using the rest of the velocities (sums).
1. Checking for superfluous DOF and formulating supplementary equations (if applicable).
1. Formulation of matrices $\mathbf{J_a}$, $\mathbf{J_p}$, $\mathbf{A_a}$ and $\mathbf{A_p}$.

The above steps can be used to find the four matrices $\mathbf{J_a}$, $\mathbf{J_p}$, $\mathbf{A_a}$ and $\mathbf{A_p}$, from which the Jacobian can be formulated by using \autoref{eq:nonserial}. For serial manipulators, since passive joints do not come into picture, the Jacobian would simply be as shown in \autoref{eq:serial}.

\begin{equation}\label{eq:nonserial}\mathbf{\widetilde{J}} = \mathbf{J_a}-\mathbf{J_p}\mathbf{A^{-1}_p}\mathbf{A_a}\end{equation}

\begin{equation}\label{eq:serial}\mathbf{\widetilde{J}} = \mathbf{J_a}\end{equation}

More details on formulation of Jacobian (along with appropriate algorithms) can be found [here](https://github.com/suneeshjacob/ACRoD/blob/main/Mathematics%20behind%20Jacobian%20formulation.md), and the notations and the nomenclature are explained [here](https://github.com/suneeshjacob/ACRoD/blob/main/Notation_and_Nomenclature.md) in detail. The robot-topology matrix representation is explained [here](https://github.com/suneeshjacob/ACRoD/blob/main/Robot_Topology_Matrix.md) in detail.

# Acknowledgements

This paper is an extended version of the Jacobian formulation presented in previous work [@synthesis] co-authored by Dr. Bhaskar Dasgupta, whom the authors would like to acknowledge for whatever credits applicable. However, the authors would like to clarify that the technical extension of theory and the whole software are developed solely by the authors of the current paper.

# References
