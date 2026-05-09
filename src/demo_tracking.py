import argparse
import copy
import os

import numpy as np

from cubic_spline_planner import CubicSpline2D
from mpc_controller import MPCConfig, MPCController
from visualization import plot_tracking_result, save_tracking_gif

DEFAULT_OUTPUT_GIF = os.path.join("assets", "images", "test.gif")


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


def run_demo(output_path: str = DEFAULT_OUTPUT_GIF, show_plot: bool = False) -> None:
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

    for _ in range(1000):
        if np.linalg.norm(current_state[:2] - goal_state[:2]) <= 3e-2:
            break

        moving_reference = np.concatenate((moving_reference[1:], moving_reference[-1:]))
        moving_reference[0] = current_state

        _, current_state, _ = controller.solve_step(current_state, moving_reference)
        actual_path.append(current_state)
        print(current_state, np.linalg.norm(current_state[:2] - goal_state[:2]))

    actual_path = np.asarray(actual_path)
    save_tracking_gif(reference_path, actual_path, output_path)
    print(f"Saved tracking GIF to {output_path}")

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
        "--show-plot",
        action="store_true",
        help="Show a static tracking result plot after saving the GIF.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_demo(output_path=args.output, show_plot=args.show_plot)
