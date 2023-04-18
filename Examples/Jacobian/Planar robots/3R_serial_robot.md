# 3R Serial Robot

## Mathematics involved

\subsection{Example 1: RRR planar serial manipulator}
\label{subsec:RRR}

\begin{figure}[hbt!]
  \centering
  \includegraphics[width=\linewidth]{RRR.png}
  \caption{RRR planar serial manipulator}
  \label{fig:RRR}
\end{figure}

An RRR planar serial manipulator is considered as shown in the figure \ref{fig:RRR}. The corresponding adjacency matrix is given by

$$\bm{M} = \left[\begin{matrix}L_1 & R & O & O \\A & L_2 & R & O\\O & A & L_3 & R\\O & O & A & L_4\end{matrix}\right]$$

\emph{Connecting paths:}

$$
\begin{matrix}
    \text{Path 1:} \;\;\; L_1-L_2-L_3-L_4
\end{matrix}
$$

Since this has only one connecting path, if the manipulator represented by the matrix is valid then it must be a serial manipulator. Hence, there would be only one independent set of formulation of linear and angular velocities, and formulation of $\begin{bmatrix}\bm{C}_{V}\end{bmatrix}$ and $\begin{bmatrix}\bm{C}_{\Omega}\end{bmatrix}$ are not required.

The following are the linear and angular velocity contributions to the end-effector from each joint of the path, which are calculated by using the formulation shown in table \ref{velocities} and by using the convention that all the revolute joints of a planar manipulator would have their axes on the xy-plane, thereby reducing the unit vector along each axis to $\bm{\hat{n}}_{(i,j)}=\bm{\hat{k}}$, as mentioned in equation 20 of the main document.

$$\begin{matrix}
  \bm{V_{12}}=\dot{\theta}_{(1,2)} \bm{\hat{n}_{(1,2)}} \times \left( \bm{a} - \bm{r}_{(1,2)} \right) = \dot{\theta}_{(1,2)} \bm{\hat{k}} \times \left( \bm{a} - \bm{r}_{(1,2)} \right) \\
  \bm{V_{23}}=\dot{\theta}_{(2,3)} \bm{\hat{n}_{(2,3)}} \times \left( \bm{a} - \bm{r}_{(2,3)} \right) = \dot{\theta}_{(2,3)} \bm{\hat{k}} \times \left( \bm{a} - \bm{r}_{(2,3)} \right) \\
  \bm{V_{34}}=\dot{\theta}_{(3,4)} \bm{\hat{n}_{(3,4)}} \times \left( \bm{a} - \bm{r}_{(3,4)} \right) = \dot{\theta}_{(3,4)} \bm{\hat{k}} \times \left( \bm{a} - \bm{r}_{(3,4)} \right)
\end{matrix}
$$

$$\begin{matrix}
  \bm{\Omega_{12}}=\dot{\theta}_{(1,2)} \bm{\hat{n}_{(1,2)}} = \dot{\theta}_{(1,2)} \bm{\hat{k}} \\
  \bm{\Omega_{23}}=\dot{\theta}_{(2,3)} \bm{\hat{n}_{(2,3)}} = \dot{\theta}_{(2,3)} \bm{\hat{k}} \\
  \bm{\Omega_{34}}=\dot{\theta}_{(3,4)} \bm{\hat{n}_{(3,4)}} = \dot{\theta}_{(3,4)} \bm{\hat{k}}
\end{matrix}
$$

Therefore, the linear and angular velocities are given by \eqref{eq:RRR_linvel} and \eqref{eq:RRR_angvel}, respectively.

\begin{equation}
    \label{eq:RRR_linvel}
    \begin{array}{cc}
        \bm{v}^{(1)}=\dot{\theta}_{(1,2)} \bm{\hat{k}} \times \left( \bm{a} - \bm{r}_{(1,2)} \right) \\
        + \dot{\theta}_{(2,3)} \bm{\hat{k}} \times \left( \bm{a} - \bm{r}_{(2,3)} \right) \\
        + \dot{\theta}_{(3,4)} \bm{\hat{k}} \times \left( \bm{a} - \bm{r}_{(3,4)} \right)
    \end{array}
\end{equation}

\begin{equation}
        \label{eq:RRR_angvel}
    \begin{array}{cc}
        \bm{\omega}^{(1)}=\dot{\theta}_{(1,2)} \bm{\hat{k}} + \dot{\theta}_{(2,3)} \bm{\hat{k}} + \dot{\theta}_{(3,4)} \bm{\hat{k}}
    \end{array}
\end{equation}

Since this is a planar manipulator, the case of superfluous DOF does not come into picture.

If the actuating joint velocities vector is considered to be $\bm{\Omega_a} = \begin{Bmatrix}\dot{\theta}_{(1,2)} & \dot{\theta}_{(2,3)} & \dot{\theta}_{(3,4)}\end{Bmatrix}^T$, the velocity of the end-effector is given by



% $$
% \begin{Bmatrix}\bm{v} \\ \bm{\omega}\end{Bmatrix} = \begin{Bmatrix}\bm{v}^{(1)} \\ \bm{\omega}^{(1)}\end{Bmatrix} = \left[\begin{matrix}- a_{y} + r_{(1,2)y} & - a_{y} + r_{(2,3)y} \\a_{x} - r_{(1,2)x} & a_{x} - r_{(2,3)x} \\1 & 1 \end{matrix}\right.
% $$
% \\
% $$
% \qquad\left.\begin{matrix} - a_{y} + r_{(3,4)y}\\ a_{x} - r_{(3,4)x}\\ 1\end{matrix}\right]\begin{Bmatrix}\dot{\theta}_{(1,2)}\\\dot{\theta}_{(2,3)}\\\dot{\theta}_{(3,4)}\end{Bmatrix}
% $$

$$\resizebox{\columnwidth}{!}{$%
\begin{Bmatrix}\bm{v} \\ \bm{\omega}\end{Bmatrix} = \begin{Bmatrix}\bm{v}^{(1)} \\ \bm{\omega}^{(1)}\end{Bmatrix} = \left[\begin{matrix}- a_{y} + r_{(1,2)y} & - a_{y} + r_{(2,3)y} & - a_{y} + r_{(3,4)y} \\a_{x} - r_{(1,2)x} & a_{x} - r_{(2,3)x} & a_{x} - r_{(3,4)x}\\1 & 1 & 1\end{matrix}\right]\begin{Bmatrix}\dot{\theta}_{(1,2)}\\\dot{\theta}_{(2,3)}\\\dot{\theta}_{(3,4)}\end{Bmatrix}
$}$$

$$
\Rightarrow \begin{Bmatrix}\bm{v} \\ \bm{\omega}\end{Bmatrix} = \bm{J_a} \bm{\Omega_a}
$$

Therefore, the Jacobian of the manipulator is

$$
\bm{\widetilde{J}} = \bm{J_a} = \left[\begin{matrix}- a_{y} + r_{(1,2)y} & - a_{y} + r_{(2,3)y} & - a_{y} + r_{(3,4)y}\\a_{x} - r_{(1,2)x} & a_{x} - r_{(2,3)x} & a_{x} - r_{(3,4)x}\\1 & 1 & 1\end{matrix}\right]
$$

Since it is a serial manipulator, the matrices $\bm{J_p}$, $\bm{A_a}$ and $\bm{A_p}$ do not come into picture.
