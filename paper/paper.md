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

Jacobian is extensively used in dimensional synthesis for Jacobian-based optimal performances of robotic manipulators, in which the optimal dimensional parameters of robots are computed. Determination of accurate mobility [@YANG20081175] of planar and spatial mechanisms can also be performed by using Jacobian in cases where Chebychev–Grübler–Kutzbach criterion cannot [@GOGU2005427] accurately determine the mobility. As a result, Jacobian is a significant part for both kinematic analysis, dimensional synthesis and mobility determination of a mechanism. Hence, the formulation of Jacobian has its key importance in the literature and in the application of performance optimisation along with mobility computation. Formulation of Jacobian for serial manipulators can be computed easily, however, it is increasingly complicated to formulate Jacobian for parallel manipulators due to the existence of passive joint velocities and the nature in which they are related to active joint velocities. Several studies [@606737;@846382;@1220722;@ALTUZARRA20061407;@Baumgärtner2022] exist for formulation of parallel manipulators but they happen to require human inspection at some level. To alleviate the drawback of human inspection, the present research aims to formulate Jacobian that is required for dimensional synthesis for optimal performance around a single point that can be used for any non-redundant manipulator without any dependency on human inspection. Jacob et al. [@synthesis] used a systematic method as a tool to formulate Jacobian matrices for several manipulators in bulk for performance optimisation around a given task point. However, several steps in that algorithm are not totally computerised but rather human-intervention-dependent. Moreover, it can be applicable only with four types of joints. The current study extends their method to present a fully computerisable Jacobian formulation algorithm that is applicable for general non-redundant planar and spatial manipulators of any topological structure of seven types of joints.

# Statement of need

ACRoD provides a Python-based package for generating functions of Jacobian

`Gala` is an Astropy-affiliated Python package for galactic dynamics. Python
enables wrapping low-level languages (e.g., C) for speed without losing
flexibility or ease-of-use in the user-interface. The API for `Gala` was
designed to provide a class-based and user-friendly interface to fast (C or
Cython-optimized) implementations of common operations such as gravitational
potential and force evaluation, orbit integration, dynamical transformations,
and chaos indicators for nonlinear dynamics. `Gala` also relies heavily on and
interfaces well with the implementations of physical units and astronomical
coordinate systems in the `Astropy` package [@astropy] (`astropy.units` and
`astropy.coordinates`).

`Gala` was designed to be used by both astronomical researchers and by
students in courses on gravitational dynamics or astronomy. It has already been
used in a number of scientific publications [@Pearson:2017] and has also been
used in graduate courses on Galactic dynamics to, e.g., provide interactive
visualizations of textbook material [@Binney:2008]. The combination of speed,
design, and support for Astropy functionality in `Gala` will enable exciting
scientific explorations of forthcoming data releases from the *Gaia* mission
[@gaia] by students and experts alike.

# Mathematics

\begin{equation}\label{eq:nonserial}\mathbf{\widetilde{J}} = \mathbf{J_a}-\mathbf{J_p}\mathbf{A^{-1}_p}\mathbf{A_a}\end{equation}

\begin{equation}\label{eq:serial}\mathbf{\widetilde{J}} = \mathbf{J_a}\end{equation}


The above steps of the algorithm are concisely shown in the pseudocode of algorithm \ref{alg:jacobian}.


1. Assert: Type(_iterator_) is Object.
1. Assert: _completion_ is a Completion Record.
1. Let _hasReturn_ be HasProperty(_iterator_, `"return"`).
1. ReturnIfAbrupt(_hasReturn_).
  1. If _hasReturn_ is *true*, then
    1. Let _innerResult_ be Invoke(_iterator_, `"return"`, ( )).
    1. If _completion_.[[type]] is not ~throw~ and _innerResult_.[[type]] is ~throw~, then
      1. Return _innerResult_.
1. Return _completion_.


This algorithm can be used to find the four matrices $\mathbf{J_a}$, $\mathbf{J_p}$, $\mathbf{A_a}$ and $\mathbf{A_p}$, from which the Jacobian can be formulated as $\mathbf{\widetilde{J}}=\mathbf{J_a}-\mathbf{J_p}\mathbf{A^{-1}_p}\mathbf{A_a}$. For serial manipulators, since passive joints do not come into picture, the Jacobian would simply be $\mathbf{\widetilde{J}}=\mathbf{J_a}$.

# Mathematics

Single dollars ($) are required for inline mathematics e.g. $f(x) = e^{\pi/x}$

Double dollars make self-standing equations:

$$\Theta(x) = \left\{\begin{array}{l}
0\textrm{ if } x < 0\cr
1\textrm{ else}
\end{array}\right.$$

You can also use plain \LaTeX for equations
\begin{equation}\label{eq:fourier}
\hat f(\omega) = \int_{-\infty}^{\infty} f(x) e^{i\omega x} dx
\end{equation}
and refer to \autoref{eq:fourier} from text.

# Citations

Citations to entries in paper.bib should be in
[rMarkdown](http://rmarkdown.rstudio.com/authoring_bibliographies_and_citations.html)
format.

If you want to cite a software repository URL (e.g. something on GitHub without a preferred
citation) then you can do it with the example BibTeX entry below for @fidgit.

For a quick reference, the following citation commands can be used:
- `@author:2001`  ->  "Author et al. (2001)"
- `[@author:2001]` -> "(Author et al., 2001)"
- `[@author1:2001; @author2:2001]` -> "(Author1 et al., 2001; Author2 et al., 2002)"

# Figures

Figures can be included like this:
![Caption for example figure.\label{fig:example}](figure.png)
and referenced from text using \autoref{fig:example}.

Figure sizes can be customized by adding an optional second parameter:
![Caption for example figure.](figure.png){ width=20% }

# Acknowledgements

This paper is an extended version of the Jacobian formulation presented in [@synthesis] and co-authored by Dr. Bhaskar Dasgupta, whom the authors would like to acknowledge for whatever credits applicable. However, the authors would like to clarify that the technical extension of theory and the whole software are developed solely by the authors of the current paper.

# References
