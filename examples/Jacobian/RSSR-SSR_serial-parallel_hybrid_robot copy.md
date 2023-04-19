# RSSR-SSR Serial-Parallel Hybrid Robot

## Mathematics involved

<p align="center">
    <img src="./RSSRSSR.png" alt="RSSR-SSR spatial parallel manipulator" width="500px">
</p>

The RSSR-SSR Serial-Parallel Hybrid Robot is taken from [[1]](#1), and a picture [[2]](#2) of the robot is shown in the above figure. The corresponding adjacency matrix is given by


\subsection{Example 3: RSSR-SSR spatial parallel manipulator}

\begin{figure}[hbt!]
  \centering
  \includegraphics[width=\linewidth]{RSSRSSR.png}
  \caption{RSSR-SSR spatial parallel manipulator}
  \label{fig:RSSRSSR}
\end{figure}

A schematic diagram of RSSR-SSR spatial parallel manipulator is shown in figure \ref{fig:RSSRSSR}. The corresponding adjacency matrix is given by

$$M = \left[\begin{matrix}L_1 & R & R & O & R & O\\A & L_2 & O & O & O & S\\A & O & L_3 & S & O & O\\O & O & O & L_4 & O & S\\O & O & O & O & L_5 & S\\O & O & O & O & O & L_6\end{matrix}\right]$$

All the steps that are followed in the previous example would follow here as well, except in step 7, the superfluous DOF comes into picture. If step 7 is not done, then the system of equations shown in \eqref{eq:velocities_v2toN} would be insufficient to represent the passive joint velocities in terms of active joint velocities. This is due to the fact that the fourth link has rotation along its longitudinal axis not controllable with the actuators yet does not affect the end-effector's velocity. Performing pseudo-inverse can fix this issue but pseudo-inverse could be a discontinuous operation near singular values. Also to enable the calculation of mobility of the robotic mechanism, the additional equation for each such superfluous DOF is calculated as per the method shown in algorithm 3 of the main document.

In step 7, since the manipulator has more than two spherical joints, the list of all possible combinations C of dividing the manipulator into two parts is considered. Since there are six links and since 6 is an odd number, ${}^{6}C_{1}+{}^{6}C_{2}+{}^{6}C_{3}=41$ distinct combinations exist, out of which the combination $[\{4\}, \{1,2,3,5,6\}]$ is discussed in detail in this sub-section. This combination has $c_1 = \{4\}$ and $c_2 = \{1,2,3,5,6\}$. And by grouping the links of each part together, the topology-matrix can be rewritten as shown in equation \eqref{eq:adjmat_RSSRSSR_superfluous}, from which the coupling matrix can be extracted to be as shown in \eqref{eq:couplingmatrix}.

$$\widetilde{M} = \begin{bmatrix}
    L_4 & O & O & S & O & S \\\\
    O & L_1 & R & R & R & O \\\\
    O & A & L_2 & O & O & S \\\\
    O & A & O & L_3 & O & O \\\\
    O & O & O & O & L_5 & S \\\\
    O & O & O & O & O & L_6
    \end{bmatrix}$$

$$\widetilde{C} = \left[\begin{matrix}O & O & S & O & S\end{matrix}\right]$$

It can be seen that the coupling matrix has only two spherical joints and no other joint. This shows that the two parts $c_1$ and $c_2$ are connected by two spherical joints alone and no other joint. And it can also be seen that both the base link (first link) and the end-effector link (last link), i.e., both the links $1$ and $6$, lie in one part of the combination, i.e., in $c_2$. Hence, $c_{be}=c_2$. The corresponding link numbers for each of the two spherical joints are $3,4$ and $4,6$. Since $3,6\in c_{be}$, the sequences $(i,j)$ and $(k,l)$ are considered to be $(3,4)$ and $(4,6)$, respectively. Since $j$ and $k$ are the same link (link 4), the superfluous link $s$ would be link $4$. Only one of the connecting paths $P$ happens to contain the link $4$ for this particular case, and that is $(1,3,4,6)$. If the path is truncated at the superfluous link, it would become $(1,3,4)$. Hence, absolute angular velocity of link 4, formulated through this truncated path, is given by equation \eqref{eq:angvelsupflulink_forexample3}.


$$\bm{\omega}\_{s} = \bm{\Omega}\_{(1,3)} + \bm{\Omega}\_{(3,4)} \Rightarrow \bm{\omega}\_{s} = \dot{\theta}\_{(1,3)}\bm{\hat{n}}\_{(1,3)} + \bm{\omega}\_{(3,4)}$$

Therefore, the additional equation corresponding to this superfluous DOF can be formulated as shown in equation \eqref{eq:supfludofeqn_forexample3}. This needs to be added to the system of equations shown in equation \eqref{eq:velocities_v2toN}, in order to make $\bm{A_p}$ a square matrix, with which the passive joint velocities can be written in terms of active joint velocities.

$$\bm{\omega}\_{s}\cdot \left(\bm{r}\_{(3,4)}-\bm{r}\_{(4,6)}\right) = 0$$

## References
<a id="1">[1]</a> 
Muralidharan V, Bandyopadhyay S (2019) "A two-degree-of-freedom rssr-ssr manipulator for sun-tracking." In: Badodkar DN, Dwarakanath TA (eds) Machines, Mechanism and Robotics. Springer Singapore, Singapore, pp 135–147

<a id="2">[2]</a> 
Jacob, Akkarapakam Suneesh, and Rituparna Datta. "A Generalised Method for Multi-objective Optimisation of Performance Parameters for Dimensional Synthesis of Robotic Manipulators Around a Specified End-effector Point" International Journal of Intelligent Robotics and Applications ***(submitted)**** (2022).
