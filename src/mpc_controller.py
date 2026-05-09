from dataclasses import dataclass, field
from typing import Tuple

import casadi as ca
import numpy as np


@dataclass
class MPCConfig:
    """Configuration for the nonlinear MPC controller."""

    dt: float = 0.05
    horizon: int = 50
    v_max: float = 0.5
    omega_max: float = np.pi / 2.0
    x_bound: Tuple[float, float] = (-2.0, 2.0)
    y_bound: Tuple[float, float] = (-2.0, 2.0)
    q: np.ndarray = field(default_factory=lambda: np.diag([20.0, 20.0, 0.1]))
    r: np.ndarray = field(default_factory=lambda: np.diag([0.5, 0.05]))
    obstacles: Tuple[Tuple[float, float, float], ...] = ()
    obstacle_safety_margin: float = 0.12
    obstacle_slack_weight: float = 1.0e4
    obstacle_repulsion_weight: float = 0.0


class MPCController:
    """Nonlinear MPC controller for a differential-wheeled robot.

    The solver graph is built once during initialization. Each control step only
    updates the current state, reference trajectory, and warm-start values.
    """

    def __init__(self, config: MPCConfig | None = None):
        self.config = config or MPCConfig()
        self._build_solver()
        self._state_guess = np.zeros((self.config.horizon + 1, 3))
        self._control_guess = np.zeros((self.config.horizon, 2))
        self._obstacle_slack_guess = None
        if self.config.obstacles:
            self._obstacle_slack_guess = np.full(
                (self.config.horizon, len(self.config.obstacles)),
                (self.config.obstacle_safety_margin + 0.2) ** 2,
            )

    @staticmethod
    def dynamics_np(state: np.ndarray, control: np.ndarray) -> np.ndarray:
        v, omega = control
        theta = state[2]
        return np.array([v * np.cos(theta), v * np.sin(theta), omega], dtype=float)

    def step_dynamics(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        return state + self.config.dt * self.dynamics_np(state, control)

    def _build_solver(self) -> None:
        cfg = self.config
        opti = ca.Opti()

        controls = opti.variable(cfg.horizon, 2)
        states = opti.variable(cfg.horizon + 1, 3)
        obstacle_slacks = None
        if cfg.obstacles:
            obstacle_slacks = opti.variable(cfg.horizon, len(cfg.obstacles))

        v = controls[:, 0]
        omega = controls[:, 1]
        x = states[:, 0]
        y = states[:, 1]

        x0 = opti.parameter(3)
        x_ref = opti.parameter(cfg.horizon + 1, 3)

        def dynamics_ca(state, control):
            return ca.vertcat(
                control[0] * ca.cos(state[2]),
                control[0] * ca.sin(state[2]),
                control[1],
            )

        opti.subject_to(states[0, :] == x0.T)
        for i in range(cfg.horizon):
            next_state = states[i, :] + dynamics_ca(states[i, :], controls[i, :]).T * cfg.dt
            opti.subject_to(states[i + 1, :] == next_state)

        objective = 0
        for i in range(cfg.horizon):
            state_error = states[i + 1, :] - x_ref[i + 1, :]
            objective += ca.mtimes([state_error, cfg.q, state_error.T])
            objective += ca.mtimes([controls[i, :], cfg.r, controls[i, :].T])

        opti.subject_to(opti.bounded(cfg.x_bound[0], x, cfg.x_bound[1]))
        opti.subject_to(opti.bounded(cfg.y_bound[0], y, cfg.y_bound[1]))
        opti.subject_to(opti.bounded(-cfg.v_max, v, cfg.v_max))
        opti.subject_to(opti.bounded(-cfg.omega_max, omega, cfg.omega_max))

        if cfg.obstacles and obstacle_slacks is not None:
            # CasADi treats matrix inequalities specially. Use element-wise scalar
            # constraints instead of `obstacle_slacks >= 0.0`.
            for slack_idx in range(cfg.horizon):
                for obs_idx in range(len(cfg.obstacles)):
                    opti.subject_to(obstacle_slacks[slack_idx, obs_idx] >= 0.0)

            for obs_idx, (obs_x, obs_y, obs_radius) in enumerate(cfg.obstacles):
                safe_radius = obs_radius + cfg.obstacle_safety_margin
                for i in range(1, cfg.horizon + 1):
                    distance_sq = (states[i, 0] - obs_x) ** 2 + (states[i, 1] - obs_y) ** 2
                    slack = obstacle_slacks[i - 1, obs_idx]
                    # Soft safety constraint: slack keeps the nonlinear program feasible,
                    # while the large penalty still strongly discourages safety violation.
                    opti.subject_to(distance_sq + slack >= safe_radius**2)
                    objective += cfg.obstacle_slack_weight * slack**2
                    if cfg.obstacle_repulsion_weight > 0.0:
                        objective += cfg.obstacle_repulsion_weight / (distance_sq + 1.0e-3)

        opti.minimize(objective)

        solver_options = {
            "ipopt.max_iter": 800,
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.acceptable_tol": 1e-6,
            "ipopt.acceptable_obj_change_tol": 1e-5,
            "ipopt.mu_strategy": "adaptive",
        }
        opti.solver("ipopt", solver_options)

        self.opti = opti
        self.controls = controls
        self.states = states
        self.obstacle_slacks = obstacle_slacks
        self.x0 = x0
        self.x_ref = x_ref

    def solve_step(self, current_state: np.ndarray, reference_window: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Solve one MPC step.

        Args:
            current_state: Current robot state, shaped `(3,)`.
            reference_window: Reference states, shaped `(horizon + 1, 3)`.

        Returns:
            first_control: The first optimized control input `[v, omega]`.
            next_state: State after applying the first control for one time step.
            predicted_states: Full predicted state trajectory from the solver.
        """
        cfg = self.config
        if reference_window.shape[0] < cfg.horizon + 1:
            raise ValueError(f"reference_window must contain at least {cfg.horizon + 1} states")

        self.opti.set_value(self.x0, current_state)
        self.opti.set_value(self.x_ref, reference_window[: cfg.horizon + 1])
        self.opti.set_initial(self.controls, self._control_guess)
        # A reference-window state warm start is much more reliable for obstacle
        # scenes than an all-zero state guess.
        self.opti.set_initial(self.states, reference_window[: cfg.horizon + 1])
        if self.obstacle_slacks is not None and self._obstacle_slack_guess is not None:
            self.opti.set_initial(self.obstacle_slacks, self._obstacle_slack_guess)

        solution = self.opti.solve()
        optimized_controls = solution.value(self.controls)
        predicted_states = solution.value(self.states)

        first_control = optimized_controls[0]
        next_state = self.step_dynamics(current_state, first_control)

        self._control_guess = np.vstack([optimized_controls[1:], optimized_controls[-1:]])
        self._state_guess = np.vstack([predicted_states[1:], predicted_states[-1:]])
        if self.obstacle_slacks is not None and self._obstacle_slack_guess is not None:
            optimized_slacks = solution.value(self.obstacle_slacks)
            self._obstacle_slack_guess = np.vstack([optimized_slacks[1:], optimized_slacks[-1:]])

        return first_control, next_state, predicted_states
