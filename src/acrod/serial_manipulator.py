from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import numpy as np

ArrayLike = Sequence[float]


@dataclass
class Material:
    name: str = "custom"
    density: float = 1.0  # kg/m^3


@dataclass
class LinkSpec:
    """
    One revolute link described by standard DH parameters and inertial data.

    alpha, a, d, theta_offset are standard DH parameters.
    mass is in kg.
    com is expressed in the link frame after the corresponding DH transform.
    inertia_local is the 3x3 inertia tensor about the COM, expressed in the
    same link frame.
    """
    alpha: float
    a: float
    d: float
    theta_offset: float = 0.0
    mass: float = 0.0
    com: ArrayLike = (0.0, 0.0, 0.0)
    inertia_local: ArrayLike = ((0.0, 0.0, 0.0),
                                (0.0, 0.0, 0.0),
                                (0.0, 0.0, 0.0))
    name: str = "link"

    def __post_init__(self) -> None:
        self.com = np.asarray(self.com, dtype=float).reshape(3)
        self.inertia_local = np.asarray(self.inertia_local, dtype=float).reshape(3, 3)
        self.inertia_local = 0.5 * (self.inertia_local + self.inertia_local.T)

    @classmethod
    def from_box(
        cls,
        *,
        alpha: float,
        a: float,
        d: float,
        size: Tuple[float, float, float],
        material: Material,
        theta_offset: float = 0.0,
        com: Optional[ArrayLike] = None,
        name: str = "link",
    ) -> "LinkSpec":
        """
        Approximate the link as a box with dimensions (lx, ly, lz).
        """
        lx, ly, lz = map(float, size)
        volume = lx * ly * lz
        mass = material.density * volume
        if com is None:
            com = np.array([0.5 * lx, 0.0, 0.0], dtype=float)
        Ixx = (mass / 12.0) * (ly ** 2 + lz ** 2)
        Iyy = (mass / 12.0) * (lx ** 2 + lz ** 2)
        Izz = (mass / 12.0) * (lx ** 2 + ly ** 2)
        inertia_local = np.diag([Ixx, Iyy, Izz])
        return cls(
            alpha=alpha,
            a=a,
            d=d,
            theta_offset=theta_offset,
            mass=mass,
            com=com,
            inertia_local=inertia_local,
            name=name,
        )

    @classmethod
    def from_cylinder_z(
        cls,
        *,
        alpha: float,
        a: float,
        d: float,
        radius: float,
        length: float,
        material: Material,
        theta_offset: float = 0.0,
        com: Optional[ArrayLike] = None,
        name: str = "link",
    ) -> "LinkSpec":
        """
        Approximate the link as a solid cylinder aligned with local z.
        """
        radius = float(radius)
        length = float(length)
        volume = np.pi * radius ** 2 * length
        mass = material.density * volume
        if com is None:
            com = np.array([0.0, 0.0, 0.5 * length], dtype=float)
        Ixx = (mass / 12.0) * (3.0 * radius ** 2 + length ** 2)
        Iyy = Ixx
        Izz = 0.5 * mass * radius ** 2
        inertia_local = np.diag([Ixx, Iyy, Izz])
        return cls(
            alpha=alpha,
            a=a,
            d=d,
            theta_offset=theta_offset,
            mass=mass,
            com=com,
            inertia_local=inertia_local,
            name=name,
        )


class SerialManipulator:
    """
    Reusable class for revolute serial manipulators using standard DH parameters.

    Features:
    - Forward kinematics
    - Geometric Jacobian
    - Jacobian derivative (numerical)
    - Inverse kinematics (Newton-Raphson and gradient descent)
    - Mass matrix, Coriolis matrix, gravity vector
    - Inverse dynamics
    - Forward dynamics
    - RK4 integration
    """

    def __init__(self, links: Sequence[LinkSpec], gravity: ArrayLike = (0.0, 0.0, -9.81)) -> None:
        if len(links) == 0:
            raise ValueError("At least one link is required.")
        self.links: List[LinkSpec] = list(links)
        self.n = len(self.links)
        self.gravity = np.asarray(gravity, dtype=float).reshape(3)

    # ------------------------------------------------------------------
    # Constructors / helpers
    # ------------------------------------------------------------------
    @classmethod
    def from_dh_table(
        cls,
        dh_table: Sequence[Sequence[float]],
        *,
        masses: Optional[Sequence[float]] = None,
        coms: Optional[Sequence[ArrayLike]] = None,
        inertias: Optional[Sequence[ArrayLike]] = None,
        gravity: ArrayLike = (0.0, 0.0, -9.81),
        names: Optional[Sequence[str]] = None,
    ) -> "SerialManipulator":
        """
        Build the manipulator directly from a DH table.

        Each DH row must be [alpha, a, d, theta_offset].
        The joint variables themselves are passed later in q.

        Example row:
            [np.pi/2, 0.0, L1, 0.0]
        meaning theta_used = q[i] + 0.0

        Another example row:
            [0.0, L2, 0.0, np.pi/2]
        meaning theta_used = q[i] + np.pi/2
        """
        n = len(dh_table)
        if masses is None:
            masses = [0.0] * n
        if coms is None:
            coms = [(0.0, 0.0, 0.0)] * n
        if inertias is None:
            inertias = [np.zeros((3, 3), dtype=float) for _ in range(n)]
        if names is None:
            names = [f"link_{i+1}" for i in range(n)]

        if not (len(masses) == len(coms) == len(inertias) == len(names) == n):
            raise ValueError("DH table, masses, coms, inertias, and names must have the same length.")

        links = []
        for i, row in enumerate(dh_table):
            if len(row) != 4:
                raise ValueError(f"DH row {i} must have exactly 4 values: [alpha, a, d, theta_offset].")
            alpha, a, d, theta_offset = row
            links.append(
                LinkSpec(
                    alpha=float(alpha),
                    a=float(a),
                    d=float(d),
                    theta_offset=float(theta_offset),
                    mass=float(masses[i]),
                    com=coms[i],
                    inertia_local=inertias[i],
                    name=str(names[i]),
                )
            )
        return cls(links, gravity=gravity)

    @staticmethod
    def box_inertia(mass: float, lx: float, ly: float, lz: float) -> np.ndarray:
        """Inertia of a solid box about its center, aligned with the local frame."""
        Ixx = (mass / 12.0) * (ly ** 2 + lz ** 2)
        Iyy = (mass / 12.0) * (lx ** 2 + lz ** 2)
        Izz = (mass / 12.0) * (lx ** 2 + ly ** 2)
        return np.diag([Ixx, Iyy, Izz])

    @staticmethod
    def cylinder_inertia_z(mass: float, radius: float, length: float) -> np.ndarray:
        """Inertia of a solid cylinder aligned with local z, about its center."""
        Ixx = (mass / 12.0) * (3.0 * radius ** 2 + length ** 2)
        Iyy = Ixx
        Izz = 0.5 * mass * radius ** 2
        return np.diag([Ixx, Iyy, Izz])

    @staticmethod
    def box_mass_from_density(lx: float, ly: float, lz: float, density: float) -> float:
        return float(lx * ly * lz * density)

    @staticmethod
    def cylinder_mass_from_density(radius: float, length: float, density: float) -> float:
        return float(np.pi * radius ** 2 * length * density)

    # ------------------------------------------------------------------
    # Basic utilities
    # ------------------------------------------------------------------
    @staticmethod
    def _dh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0, sa,       ca,      d],
            [0.0, 0.0,      0.0,     1.0],
        ], dtype=float)

    @staticmethod
    def _rotation_error(R_current: np.ndarray, R_target: np.ndarray) -> np.ndarray:
        return 0.5 * (
            np.cross(R_current[:, 0], R_target[:, 0]) +
            np.cross(R_current[:, 1], R_target[:, 1]) +
            np.cross(R_current[:, 2], R_target[:, 2])
        )

    @staticmethod
    def _wrap_angles(q: np.ndarray) -> np.ndarray:
        return (q + np.pi) % (2.0 * np.pi) - np.pi

    def _kinematic_cache(self, q: ArrayLike) -> dict:
        q = np.asarray(q, dtype=float).reshape(self.n)

        frames: List[np.ndarray] = [np.eye(4)]
        for i, link in enumerate(self.links):
            theta = q[i] + link.theta_offset
            A_i = self._dh_transform(link.alpha, link.a, link.d, theta)
            frames.append(frames[-1] @ A_i)

        joint_origins = [frames[i][:3, 3].copy() for i in range(self.n)]
        joint_axes = [frames[i][:3, 2].copy() for i in range(self.n)]

        com_positions = []
        com_rotations = []
        for i, link in enumerate(self.links):
            T_i = frames[i + 1]
            p_com = T_i[:3, 3] + T_i[:3, :3] @ link.com
            com_positions.append(p_com)
            com_rotations.append(T_i[:3, :3])

        return {
            "q": q,
            "frames": frames,
            "joint_origins": joint_origins,
            "joint_axes": joint_axes,
            "com_positions": com_positions,
            "com_rotations": com_rotations,
        }

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------
    def forward_kinematics(self, q: ArrayLike, return_all_frames: bool = False):
        cache = self._kinematic_cache(q)
        if return_all_frames:
            return cache["frames"]
        return cache["frames"][-1]

    def forward_kinematics_to_link(self, q: ArrayLike, link_index: int) -> np.ndarray:
        frames = self.forward_kinematics(q, return_all_frames=True)
        return frames[link_index + 1]

    def end_effector_position(self, q: ArrayLike) -> np.ndarray:
        return self.forward_kinematics(q)[:3, 3]

    def end_effector_rotation(self, q: ArrayLike) -> np.ndarray:
        return self.forward_kinematics(q)[:3, :3]

    def all_joint_positions(self, q: ArrayLike) -> np.ndarray:
        frames = self.forward_kinematics(q, return_all_frames=True)
        return np.array([T[:3, 3] for T in frames], dtype=float)

    def jacobian(self, q: ArrayLike) -> np.ndarray:
        cache = self._kinematic_cache(q)
        frames = cache["frames"]
        joint_origins = cache["joint_origins"]
        joint_axes = cache["joint_axes"]

        p_ee = frames[-1][:3, 3]
        J = np.zeros((6, self.n), dtype=float)

        for j in range(self.n):
            z = joint_axes[j]
            p = joint_origins[j]
            J[:3, j] = np.cross(z, p_ee - p)
            J[3:, j] = z

        return J

    def jacobian_derivative(self, q: ArrayLike, q_dot: ArrayLike, eps: float = 1e-6) -> np.ndarray:
        """
        Numerical approximation of Jdot = dJ/dt using a central difference
        along the motion direction q_dot.
        """
        q = np.asarray(q, dtype=float).reshape(self.n)
        q_dot = np.asarray(q_dot, dtype=float).reshape(self.n)
        J_plus = self.jacobian(q + eps * q_dot)
        J_minus = self.jacobian(q - eps * q_dot)
        return (J_plus - J_minus) / (2.0 * eps)

    def link_com_jacobian(self, q: ArrayLike, link_index: int) -> np.ndarray:
        cache = self._kinematic_cache(q)
        joint_origins = cache["joint_origins"]
        joint_axes = cache["joint_axes"]
        p_com = cache["com_positions"][link_index]

        J = np.zeros((6, self.n), dtype=float)
        for j in range(link_index + 1):
            z = joint_axes[j]
            p = joint_origins[j]
            J[:3, j] = np.cross(z, p_com - p)
            J[3:, j] = z
        return J

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------
    def pose_error(self, q: ArrayLike, T_target: np.ndarray, position_only: bool = False) -> np.ndarray:
        T_current = self.forward_kinematics(q)
        pos_err = T_target[:3, 3] - T_current[:3, 3]
        if position_only:
            return pos_err
        rot_err = self._rotation_error(T_current[:3, :3], T_target[:3, :3])
        return np.hstack((pos_err, rot_err))

    def inverse_kinematics_newton(
        self,
        T_target: np.ndarray,
        q0: ArrayLike,
        max_iter: int = 100,
        tol: float = 1e-6,
        damping: float = 1e-6,
        position_only: bool = False,
    ) -> Tuple[np.ndarray, bool, int]:
        q = np.asarray(q0, dtype=float).reshape(self.n).copy()

        for k in range(max_iter):
            err = self.pose_error(q, T_target, position_only=position_only)
            if np.linalg.norm(err) < tol:
                return self._wrap_angles(q), True, k

            J = self.jacobian(q)
            if position_only:
                J = J[:3, :]

            A = J.T @ J + damping * np.eye(self.n)
            dq = np.linalg.solve(A, J.T @ err)
            q = self._wrap_angles(q + dq)

        return q, False, max_iter

    def inverse_kinematics_gradient(
        self,
        T_target: np.ndarray,
        q0: ArrayLike,
        step_size: float = 0.05,
        max_iter: int = 2000,
        tol: float = 1e-6,
        position_only: bool = False,
    ) -> Tuple[np.ndarray, bool, int]:
        q = np.asarray(q0, dtype=float).reshape(self.n).copy()

        for k in range(max_iter):
            err = self.pose_error(q, T_target, position_only=position_only)
            if np.linalg.norm(err) < tol:
                return self._wrap_angles(q), True, k

            J = self.jacobian(q)
            if position_only:
                J = J[:3, :]

            q = self._wrap_angles(q + step_size * J.T @ err)

        return q, False, max_iter

    # ------------------------------------------------------------------
    # Dynamics
    # ------------------------------------------------------------------
    def mass_matrix(self, q: ArrayLike) -> np.ndarray:
        cache = self._kinematic_cache(q)
        M = np.zeros((self.n, self.n), dtype=float)

        for i, link in enumerate(self.links):
            J_i = self.link_com_jacobian(cache["q"], i)
            Jv = J_i[:3, :]
            Jw = J_i[3:, :]
            R_i = cache["com_rotations"][i]
            I_base = R_i @ link.inertia_local @ R_i.T
            M += link.mass * (Jv.T @ Jv) + Jw.T @ I_base @ Jw

        return 0.5 * (M + M.T)

    def gravity_vector(self, q: ArrayLike) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(self.n)
        G = np.zeros(self.n, dtype=float)

        for i, link in enumerate(self.links):
            J_i = self.link_com_jacobian(q, i)
            Jv = J_i[:3, :]
            G += -link.mass * (Jv.T @ self.gravity)

        return G

    def coriolis_matrix(self, q: ArrayLike, q_dot: ArrayLike, eps: float = 1e-6) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(self.n)
        q_dot = np.asarray(q_dot, dtype=float).reshape(self.n)

        dM_dq = np.zeros((self.n, self.n, self.n), dtype=float)
        for k in range(self.n):
            dq = np.zeros(self.n, dtype=float)
            dq[k] = eps
            M_plus = self.mass_matrix(q + dq)
            M_minus = self.mass_matrix(q - dq)
            dM_dq[k] = (M_plus - M_minus) / (2.0 * eps)

        C = np.zeros((self.n, self.n), dtype=float)
        for i in range(self.n):
            for j in range(self.n):
                cij = 0.0
                for k in range(self.n):
                    c_ijk = 0.5 * (
                        dM_dq[k, i, j] +
                        dM_dq[j, i, k] -
                        dM_dq[i, j, k]
                    )
                    cij += c_ijk * q_dot[k]
                C[i, j] = cij
        return C

    def inverse_dynamics(self, q: ArrayLike, q_dot: ArrayLike, q_ddot: ArrayLike) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(self.n)
        q_dot = np.asarray(q_dot, dtype=float).reshape(self.n)
        q_ddot = np.asarray(q_ddot, dtype=float).reshape(self.n)

        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, q_dot)
        G = self.gravity_vector(q)
        return M @ q_ddot + C @ q_dot + G

    def forward_dynamics(self, q: ArrayLike, q_dot: ArrayLike, tau: ArrayLike) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(self.n)
        q_dot = np.asarray(q_dot, dtype=float).reshape(self.n)
        tau = np.asarray(tau, dtype=float).reshape(self.n)

        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, q_dot)
        G = self.gravity_vector(q)
        rhs = tau - C @ q_dot - G
        return np.linalg.solve(M, rhs)

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------
    def state_derivative(self, state: ArrayLike, tau: ArrayLike) -> np.ndarray:
        state = np.asarray(state, dtype=float).reshape(2 * self.n)
        q = state[:self.n]
        q_dot = state[self.n:]
        q_ddot = self.forward_dynamics(q, q_dot, tau)
        return np.hstack((q_dot, q_ddot))

    def step_rk4(self, q: ArrayLike, q_dot: ArrayLike, tau: ArrayLike, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        q = np.asarray(q, dtype=float).reshape(self.n)
        q_dot = np.asarray(q_dot, dtype=float).reshape(self.n)
        tau = np.asarray(tau, dtype=float).reshape(self.n)
        state = np.hstack((q, q_dot))

        k1 = self.state_derivative(state, tau)
        k2 = self.state_derivative(state + 0.5 * dt * k1, tau)
        k3 = self.state_derivative(state + 0.5 * dt * k2, tau)
        k4 = self.state_derivative(state + dt * k3, tau)

        next_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        q_next = self._wrap_angles(next_state[:self.n])
        q_dot_next = next_state[self.n:]
        return q_next, q_dot_next


if __name__ == "__main__":
    # --------------------------------------------------------------
    # Example for the exact DH style from your notebook.
    # --------------------------------------------------------------
    pi = np.pi
    ninety_degrees = np.pi / 2

    L1 = 0.40
    L2 = 0.30
    L3 = 0.20
    L4 = 0.10
    L5 = 0.10
    L6 = 0.08

    # Convert your notebook-style table into constant offsets.
    # The actual joint variables will be supplied separately in q.
    DH_table = [
        [ninety_degrees, 0.0, L1, 0.0],
        [0.0,            L2,  0.0, ninety_degrees],
        [ninety_degrees, 0.0, 0.0, 0.0],
        [ninety_degrees, 0.0, L3 + L4, np.pi],
        [ninety_degrees, 0.0, 0.0, ninety_degrees],
        [0.0,            0.0, L5 + L6, 0.0],
    ]

    masses = [5.0, 4.0, 3.0, 2.5, 1.5, 1.0]
    coms = [
        (0.0, 0.0, L1 / 2),
        (L2 / 2, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, (L3 + L4) / 2),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, (L5 + L6) / 2),
    ]
    inertias = [
        np.diag([0.02, 0.02, 0.01]),
        np.diag([0.01, 0.03, 0.03]),
        np.diag([0.01, 0.01, 0.01]),
        np.diag([0.02, 0.02, 0.01]),
        np.diag([0.005, 0.005, 0.003]),
        np.diag([0.004, 0.004, 0.002]),
    ]

    robot = SerialManipulator.from_dh_table(
        DH_table,
        masses=masses,
        coms=coms,
        inertias=inertias,
        gravity=(0.0, 0.0, -9.81),
    )

    q = np.array([0.2, -0.3, 0.1, 0.4, -0.2, 0.3], dtype=float)
    q_dot = np.array([0.1, 0.0, -0.05, 0.02, 0.01, -0.03], dtype=float)
    q_ddot = np.array([0.5, -0.2, 0.1, 0.3, -0.1, 0.2], dtype=float)

    T_ee = robot.forward_kinematics(q)
    J = robot.jacobian(q)
    J_dot = robot.jacobian_derivative(q, q_dot)
    tau = robot.inverse_dynamics(q, q_dot, q_ddot)
    q_ddot_fd = robot.forward_dynamics(q, q_dot, tau)

    print("End-effector transform:\n", T_ee)
    print("\nJacobian shape:", J.shape)
    print("Jdot shape:", J_dot.shape)
    print("\nInverse dynamics tau:\n", tau)
    print("\nForward dynamics q_ddot:\n", q_ddot_fd)

    # IK example (position only)
    T_target = T_ee.copy()
    q0 = np.zeros(robot.n)
    q_sol_nr, ok_nr, it_nr = robot.inverse_kinematics_newton(T_target, q0, position_only=False)
    q_sol_gd, ok_gd, it_gd = robot.inverse_kinematics_gradient(T_target, q0, position_only=False)

    print("\nNewton IK converged:", ok_nr, "iterations:", it_nr)
    print("Newton IK solution:\n", q_sol_nr)
    print("\nGradient IK converged:", ok_gd, "iterations:", it_gd)
    print("Gradient IK solution:\n", q_sol_gd)
