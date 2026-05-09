import argparse
import copy
import os

import numpy as np

from cubic_spline_planner import CubicSpline2D
from mpc_controller import MPCConfig, MPCController
from visualization import (
    plot_tracking_result,
    save_control_profile,
    save_demo_log,
    save_error_profile,
    save_tracking_gif,
)

DEFAULT_OUTPUT_GIF = os.path.join("assets", "images", "test.gif")
DEFAULT_CONTROL_PLOT = os.path.join("assets", "images", "control_profile.png")
DEFAULT_ERROR_PLOT = os.path.join("assets", "images", "tracking_error.png")
DEFAULT_LOG_CSV = os.path.join("assets", "logs", "demo_tracking.csv")


def generate_reference_path(waypoints: np.ndarray, horizon: int, ds: float = 0.02) -> np.ndarray:
    spline = CubicSpline2D(waypoints[:, 0], waypoints[:, 1])
    s_values = np.arange(0, spline.s[-1], ds)

    reference_path = []
    for s in s_values:
        x, y = spline.calc_position(s)
        yaw = spline.calc_yaw(s)
        reference_path.append([x, y, yaw])

    while len(reference_path) <= horizon + 1:
        reference_path.append(reference_path[-1])

    return np.asarray(reference_path)


def run_demo(
    output_path: str = DEFAULT_OUTPUT_GIF,
    control_plot_path: str = DEFAULT_CONTROL_PLOT,
    error_plot_path: str = DEFAULT_ERROR_PLOT,
    log_path: str = DEFAULT_LOG_CSV,
    show_plot: bool = False,
    save_metrics: bool = True,
) -> None:
    config = MPCConfig(dt=0.05, horizon=50)
    controller = MPCController(config)

    current_state = np.array([-1.0, -1.0, 0.0])
    goal_state = np.array([0.5, 0.0, 0.0])
    waypoints = np.array(
        [
            [-1.0, -1.0, 0.0],
            [0.0, -0.5, 0.0],
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
        ]
    )

    reference_path = generate_reference_path(waypoints, horizon=config.horizon)
    moving_reference = copy.deepcopy(reference_path)
    actual_path = [current_state]
    controls = []
    predicted_paths = []
    errors = [np.linalg.norm(current_state[:2] - goal_state[:2])]

    for _ in range(1000):
        if np.linalg.norm(current_state[:2] - goal_state[:2]) <= 3e-2:
            break

        moving_reference = np.concatenate((moving_reference[1:], moving_reference[-1:]))
        moving_reference[0] = current_state

        control, current_state, predicted_states = controller.solve_step(current_state, moving_reference)
        controls.append(control)
        predicted_paths.append(predicted_states)
        actual_path.append(current_state)
        current_error = np.linalg.norm(current_state[:2] - goal_state[:2])
        errors.append(current_error)
        print(current_state, current_error)

    actual_path = np.asarray(actual_path)
    controls = np.asarray(controls)
    errors = np.asarray(errors)

    # Align the prediction list with animation frames. Frame 0 has no previous solve,
    # so use the first prediction to make the GIF easier to read.
    if predicted_paths:
        predicted_paths_for_gif = [predicted_paths[0]] + predicted_paths
    else:
        predicted_paths_for_gif = None

    save_tracking_gif(
        reference_path,
        actual_path,
        output_path,
        predicted_paths=predicted_paths_for_gif,
        controls=controls,
        errors=errors,
    )
    print(f"Saved tracking GIF to {output_path}")

    if save_metrics and len(controls) > 0:
        save_control_profile(controls, config.dt, control_plot_path)
        save_error_profile(errors, config.dt, error_plot_path)
        save_demo_log(actual_path, controls, errors, log_path)
        print(f"Saved control profile to {control_plot_path}")
        print(f"Saved tracking error plot to {error_plot_path}")
        print(f"Saved demo log to {log_path}")

    if show_plot:
        plot_tracking_result(reference_path, actual_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the differential-wheeled robot NMPC tracking demo.")
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_GIF,
        help="Path for the generated tracking GIF.",
    )
    parser.add_argument(
        "--control-plot",
        default=DEFAULT_CONTROL_PLOT,
        help="Path for the generated control profile figure.",
    )
    parser.add_argument(
        "--error-plot",
        default=DEFAULT_ERROR_PLOT,
        help="Path for the generated tracking error figure.",
    )
    parser.add_argument(
        "--log",
        default=DEFAULT_LOG_CSV,
        help="Path for the generated CSV run log.",
    )
    parser.add_argument(
        "--no-metrics",
        action="store_true",
        help="Only generate the GIF and skip PNG/CSV metric outputs.",
    )
    parser.add_argument(
        "--show-plot",
        action="store_true",
        help="Show a static tracking result plot after saving the GIF.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_demo(
        output_path=args.output,
        control_plot_path=args.control_plot,
        error_plot_path=args.error_plot,
        log_path=args.log,
        show_plot=args.show_plot,
        save_metrics=not args.no_metrics,
    )
