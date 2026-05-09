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
    save_multi_scenario_gif,
    save_tracking_gif,
)

DEFAULT_OUTPUT_GIF = os.path.join("assets", "images", "test.gif")
DEFAULT_MULTI_SCENARIO_GIF = os.path.join("assets", "images", "multi_scenario_demo.gif")
DEFAULT_CONTROL_PLOT = os.path.join("assets", "images", "control_profile.png")
DEFAULT_ERROR_PLOT = os.path.join("assets", "images", "tracking_error.png")
DEFAULT_LOG_CSV = os.path.join("assets", "logs", "demo_tracking.csv")
DEFAULT_SCENARIOS = ["curve", "s_curve", "circle"]


def get_scenario_waypoints(name: str) -> np.ndarray:
    scenarios = {
        "line": np.array(
            [
                [-1.0, -0.8, 0.0],
                [-0.4, -0.8, 0.0],
                [0.2, -0.8, 0.0],
                [0.8, -0.8, 0.0],
            ]
        ),
        "curve": np.array(
            [
                [-1.0, -1.0, 0.0],
                [0.0, -0.5, 0.0],
                [0.0, 0.0, 0.0],
                [0.5, 0.0, 0.0],
            ]
        ),
        "s_curve": np.array(
            [
                [-1.2, -0.8, 0.0],
                [-0.6, 0.5, 0.0],
                [0.2, -0.5, 0.0],
                [0.9, 0.6, 0.0],
            ]
        ),
        # A closed circle cannot be tracked as one segment with the current
        # point-goal termination rule, because start and goal coincide.
        # Use circle_upper + circle_lower internally and combine them as "circle".
        "circle_upper": np.array(
            [
                [0.6, 0.0, 0.0],
                [0.35, 0.55, 0.0],
                [0.0, 0.7, 0.0],
                [-0.35, 0.55, 0.0],
                [-0.6, 0.0, 0.0],
            ]
        ),
        "circle_lower": np.array(
            [
                [-0.6, 0.0, 0.0],
                [-0.35, -0.55, 0.0],
                [0.0, -0.7, 0.0],
                [0.35, -0.55, 0.0],
                [0.6, 0.0, 0.0],
            ]
        ),
        "zigzag": np.array(
            [
                [-1.1, -0.7, 0.0],
                [-0.6, 0.6, 0.0],
                [0.0, -0.6, 0.0],
                [0.6, 0.6, 0.0],
                [1.1, -0.2, 0.0],
            ]
        ),
    }
    if name not in scenarios:
        valid = ", ".join(sorted([*scenarios.keys(), "circle"]))
        raise ValueError(f"Unknown scenario '{name}'. Available scenarios: {valid}")
    return scenarios[name]


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


def simulate_tracking_segment(
    scenario_name: str,
    config: MPCConfig,
    start_state: np.ndarray | None = None,
    max_steps: int = 1000,
) -> dict:
    controller = MPCController(config)
    waypoints = get_scenario_waypoints(scenario_name)
    reference_path = generate_reference_path(waypoints, horizon=config.horizon)

    current_state = reference_path[0].copy() if start_state is None else start_state.copy()
    goal_state = reference_path[-1].copy()
    moving_reference = copy.deepcopy(reference_path)
    moving_reference[0] = current_state

    actual_path = [current_state.copy()]
    controls = []
    predicted_paths = []
    errors = [np.linalg.norm(current_state[:2] - goal_state[:2])]

    for _ in range(max_steps):
        if np.linalg.norm(current_state[:2] - goal_state[:2]) <= 3e-2:
            break

        moving_reference = np.concatenate((moving_reference[1:], moving_reference[-1:]))
        moving_reference[0] = current_state

        control, current_state, predicted_states = controller.solve_step(current_state, moving_reference)
        controls.append(control)
        predicted_paths.append(predicted_states)
        actual_path.append(current_state.copy())
        current_error = np.linalg.norm(current_state[:2] - goal_state[:2])
        errors.append(current_error)
        print(f"[{scenario_name}]", current_state, current_error)

    actual_path = np.asarray(actual_path)
    controls = np.asarray(controls)
    errors = np.asarray(errors)
    predicted_paths_for_gif = [predicted_paths[0]] + predicted_paths if predicted_paths else None

    return {
        "name": scenario_name,
        "reference_path": reference_path,
        "actual_path": actual_path,
        "controls": controls,
        "errors": errors,
        "predicted_paths": predicted_paths,
        "predicted_paths_for_gif": predicted_paths_for_gif,
    }


def _merge_segment_results(name: str, segments: list[dict]) -> dict:
    reference_parts = []
    actual_parts = []
    controls_parts = []
    errors_parts = []
    predicted_paths = []

    for idx, segment in enumerate(segments):
        start = 0 if idx == 0 else 1
        reference_parts.append(segment["reference_path"][start:])
        actual_parts.append(segment["actual_path"][start:])
        if len(segment["controls"]) > 0:
            controls_parts.append(segment["controls"])
        errors_parts.append(segment["errors"][start:])
        predicted_paths.extend(segment["predicted_paths"])

    reference_path = np.vstack(reference_parts)
    actual_path = np.vstack(actual_parts)
    controls = np.vstack(controls_parts) if controls_parts else np.empty((0, 2))
    errors = np.concatenate(errors_parts)
    predicted_paths_for_gif = [predicted_paths[0]] + predicted_paths if predicted_paths else None

    return {
        "name": name,
        "reference_path": reference_path,
        "actual_path": actual_path,
        "controls": controls,
        "errors": errors,
        "predicted_paths": predicted_paths,
        "predicted_paths_for_gif": predicted_paths_for_gif,
    }


def simulate_tracking(scenario_name: str, config: MPCConfig, max_steps: int = 1000) -> dict:
    if scenario_name == "circle":
        upper = simulate_tracking_segment("circle_upper", config, max_steps=max_steps)
        lower = simulate_tracking_segment(
            "circle_lower",
            config,
            start_state=upper["actual_path"][-1],
            max_steps=max_steps,
        )
        return _merge_segment_results("circle", [upper, lower])
    return simulate_tracking_segment(scenario_name, config, max_steps=max_steps)


def run_demo(
    output_path: str = DEFAULT_OUTPUT_GIF,
    control_plot_path: str = DEFAULT_CONTROL_PLOT,
    error_plot_path: str = DEFAULT_ERROR_PLOT,
    log_path: str = DEFAULT_LOG_CSV,
    scenario: str = "curve",
    show_plot: bool = False,
    save_metrics: bool = True,
) -> None:
    config = MPCConfig(dt=0.05, horizon=50)
    result = simulate_tracking(scenario, config)

    save_tracking_gif(
        result["reference_path"],
        result["actual_path"],
        output_path,
        predicted_paths=result["predicted_paths_for_gif"],
        controls=result["controls"],
        errors=result["errors"],
    )
    print(f"Saved tracking GIF to {output_path}")

    if save_metrics and len(result["controls"]) > 0:
        save_control_profile(result["controls"], config.dt, control_plot_path)
        save_error_profile(result["errors"], config.dt, error_plot_path)
        save_demo_log(result["actual_path"], result["controls"], result["errors"], log_path)
        print(f"Saved control profile to {control_plot_path}")
        print(f"Saved tracking error plot to {error_plot_path}")
        print(f"Saved demo log to {log_path}")

    if show_plot:
        plot_tracking_result(result["reference_path"], result["actual_path"])


def run_multi_scenario_demo(
    scenarios: list[str],
    output_path: str = DEFAULT_MULTI_SCENARIO_GIF,
    max_steps: int = 1000,
) -> None:
    config = MPCConfig(dt=0.05, horizon=50)
    scenario_results = [simulate_tracking(name, config, max_steps=max_steps) for name in scenarios]
    save_multi_scenario_gif(scenario_results, output_path)
    print(f"Saved multi-scenario GIF to {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the differential-wheeled robot NMPC tracking demo.")
    scenario_choices = ["line", "curve", "s_curve", "circle", "circle_upper", "circle_lower", "zigzag"]
    parser.add_argument(
        "--output",
        default=DEFAULT_MULTI_SCENARIO_GIF,
        help="Path for the generated tracking GIF.",
    )
    parser.add_argument(
        "--scenario",
        default="curve",
        choices=scenario_choices,
        help="Scenario used when --multi-scenario is disabled.",
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=DEFAULT_SCENARIOS,
        choices=scenario_choices,
        help="Scenario list rendered together in one multi-panel GIF.",
    )
    parser.add_argument(
        "--single-scenario",
        action="store_true",
        help="Generate only one scenario GIF instead of the combined multi-scenario GIF.",
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
    if args.single_scenario:
        run_demo(
            output_path=args.output,
            control_plot_path=args.control_plot,
            error_plot_path=args.error_plot,
            log_path=args.log,
            scenario=args.scenario,
            show_plot=args.show_plot,
            save_metrics=not args.no_metrics,
        )
    else:
        run_multi_scenario_demo(
            scenarios=args.scenarios,
            output_path=args.output,
        )
