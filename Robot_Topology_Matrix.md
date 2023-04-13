# Robot-topology Matrix
## Matrix-based representation of robot topology
From graph theory, an adjacency matrix of a mechanism is normally a symmetric matrix in which each diagonal element represents each link and each off-diagonal element represents the connection between the corresponding links. The adjacency matrix is normally a symmetric matrix. However, to accommodate the distinction between active and passive joints, the upper off-diagonal region is dedicated to represent the connectivity of links whilst the lower off-diagonal region is dedicated to represent whether each corresponding joint is an active joint or a passive. The diagonal elements are normally taken as zeroes but in this study they are filled with '9's in order to distinguish the links from the absence of joints which could be helpful from the programming point of view. For linkages of single type of joints, each off-diagonal element is normally taken to be 1 when the two links are directly connected by a joint, and 0 when the two links are not directly connected by a joint. Since the robots in this study could have more than one type of joint, the off-diagonal elements are considered as shown in the table below.

| S. No. | Joint type | Off-diagonal element |
| :----:  | ---     | :---: |
|   1     |  Revolute ($R$)     | 1 |
|   2     |  Prismatic ($P$)    | 2 |
|   3     |  Cylindrical ($C$)    | 3 |
|   4     |  Spherical ($S$)    | 4 |
|   5     |  Universal ($U$)    | 5 |
|   6     |  Helical ($H$)    | 6 |
|   7     |  Plane ($F$)    | 7 |
|   8     |  No joint ($O$)    | 0 |


Unlike mere mechanisms, robots have a link dedicated to base link and a link dedicated to end-effector link. Hence, the first diagonal element in the matrix is dedicated to the base link and the last diagonal element is dedicated to the end-effector link.

In a serial manipulator, typically all joints are active. However, in parallel manipulators (including closed-loop manipulators), not all joints are active. There could be multiple ways of choosing active and passive sets of joints for parallel ones. The adjacency matrix notation provided in our earlier study[[1]](#1) does not consist of information on the distinction of active and passive sets of joints but rather considers both the active and the passive joints alike. In this study, this issue is resolved by dedicating the lower off-diagonal elements of the matrix to represent whether they are active joints or not. For every upper off-diagonal element that represents a joint, the corresponding lower off-diagonal element (the corresponding element of its transpose) is assigned the number 1 if the joint is active and 0 if the joint is passive. And every upper off-diagonal element that does not represent a joint, would have the number 0 assigned to its corresponding off-diagonal element.

## References
<a id="1">[1]</a> 
Jacob, Akkarapakam Suneesh, Bhaskar Dasgupta, and Rituparna Datta. "Enumeration of spatial manipulators by using the concept of Adjacency Matrix." arXiv preprint arXiv:2210.03327 (2022).
