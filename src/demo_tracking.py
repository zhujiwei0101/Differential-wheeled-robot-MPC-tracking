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
        valid = ", ".join(sorted([*scenarios.keys(), "circle", "circle_upper", "circle_lower"]))
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

    reference_path = np.asarray(reference_path)
    # Yaw is periodic. Without unwrap, yaw may jump from +pi to -pi,
    # which makes the MPC believe there is a huge heading error.
    reference_path[:, 2] = np.unwrap(reference_path[:, 2])
    return reference_path


def generate_circle_reference_path(horizon: int, radius: float = 0.7, ds: float = 0.02) -> np.ndarray:
    """Generate one geometrically continuous circular reference path.

    The previous implementation generated the upper and lower half-circles as two
    independent cubic splines. Even if the endpoints coincide, the spline tangent
    and curvature are not guaranteed to match at the split point, so the plotted
    circle can look sharp at the boundary. This parametric circle keeps position,
    tangent, yaw, and curvature continuous. The tracking code can still execute it
    as two open half-circle segments to avoid the closed-loop start==goal issue.
    """
    arc_length = 2.0 * np.pi * radius
    n_points = max(int(np.ceil(arc_length / ds)) + 1, horizon + 2)
    angles = np.linspace(0.0, 2.0 * np.pi, n_points)

    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    yaw = angles + np.pi / 2.0
    reference_path = np.column_stack([x, y, yaw])

    while len(reference_path) <= horizon + 1:
        reference_path = np.vstack([reference_path, reference_path[-1]])

    reference_path[:, 2] = np.unwrap(reference_path[:, 2])
    return reference_path


def get_circle_reference_segment(name: str, horizon: int) -> np.ndarray:
    full_reference = generate_circle_reference_path(horizon=horizon)
    split_index = (len(full_reference) - 1) // 2
    if name == "circle_upper":
        return full_reference[: split_index + 1]
    if name == "circle_lower":
        return full_reference[split_index:]
    raise ValueError(f"Unknown circle segment '{name}'")


def align_state_yaw_to_reference(state: np.ndarray, reference_yaw: float) -> np.ndarray:
    """Shift state yaw by multiples of 2pi so it is close to the reference yaw."""
    aligned = state.copy()
    aligned[2] = reference_yaw + np.arctan2(np.sin(aligned[2] - reference_yaw), np.cos(aligned[2] - reference_yaw))
    return aligned


def get_reference_window(
    reference_path: np.ndarray,
    current_state: np.ndarray,
    horizon: int,
    progress_index: int,
    search_back: int = 5,
) -> tuple[np.ndarray, int]:
    """Find a progress-based local reference window."""
    search_start = max(progress_index - search_back, 0)
    search_path = reference_path[search_start:, :2]
    distances = np.linalg.norm(search_path - current_state[:2], axis=1)
    nearest_index = search_start + int(np.argmin(distances))
    progress_index = max(progress_index, nearest_index)

    end_index = progress_index + horizon + 1
    window = reference_path[progress_index:end_index]
    if len(window) < horizon + 1:
        padding = np.repeat(reference_path[-1][None, :], horizon + 1 - len(window), axis=0)
        window = np.vstack([window, padding])

    window = window.copy()
    window[0] = align_state_yaw_to_reference(current_state, window[0, 2])
    return window, progress_index


def simulate_tracking_reference(
    scenario_name: str,
    reference_path: np.ndarray,
    config: MPCConfig,
    start_state: np.ndarray | None = None,
    max_steps: int = 1000,
) -> dict:
    controller = MPCController(config)
    current_state = reference_path[0].copy() if start_state is None else align_state_yaw_to_reference(start_state, reference_path[0, 2])
    goal_state = reference_path[-1].copy()
    progress_index = 0
    actual_path = [current_state.copy()]
    controls = []
    predicted_paths = []
    errors = [np.linalg.norm(current_state[:2] - goal_state[:2])]

    for _ in range(max_steps):
        near_path_end = progress_index >= len(reference_path) - config.horizon - 2
        close_to_goal = np.linalg.norm(current_state[:2] - goal_state[:2]) <= 3e-2
        if near_path_end and close_to_goal:
            break

        reference_window, progress_index = get_reference_window(
            reference_path,
            current_state,
            config.horizon,
            progress_index,
        )

        control, current_state, predicted_states = controller.solve_step(current_state, reference_window)
        current_state = align_state_yaw_to_reference(current_state, reference_window[1, 2])
        controls.append(control)
        predicted_paths.append(predicted_states)
        actual_path.append(current_state.copy())
        current_error = np.linalg.norm(current_state[:2] - goal_state[:2])
        errors.append(current_error)
        print(f"[{scenario_name}]", current_state, current_error, f"progress={progress_index}")

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


def simulate_tracking_segment(
    scenario_name: str,
    config: MPCConfig,
    start_state: np.ndarray | None = None,
    max_steps: int = 1000,
) -> dict:
    if scenario_name in {"circle_upper", "circle_lower"}:
        reference_path = get_circle_reference_segment(scenario_name, horizon=config.horizon)
    else:
        waypoints = get_scenario_waypoints(scenario_name)
        reference_path = generate_reference_path(waypoints, horizon=config.horizon)
    return simulate_tracking_reference(scenario_name, reference_path, config, start_state=start_state, max_steps=max_steps)


def _merge_segment_results(name: str, segments: list[dict], reference_path: np.ndarray | None = None) -> dict:
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

    merged_reference_path = reference_path if reference_path is not None else np.vstack(reference_parts)
    actual_path = np.vstack(actual_parts)
    controls = np.vstack(controls_parts) if controls_parts else np.empty((0, 2))
    errors = np.concatenate(errors_parts)
    predicted_paths_for_gif = [predicted_paths[0]] + predicted_paths if predicted_paths else None

    return {
        "name": name,
        "reference_path": merged_reference_path,
        "actual_path": actual_path,
        "controls": controls,
        "errors": errors,
        "predicted_paths": predicted_paths,
        "predicted_paths_for_gif": predicted_paths_for_gif,
    }


def simulate_tracking(scenario_name: str, config: MPCConfig, max_steps: int = 1000) -> dict:
    if scenario_name == "circle":
        full_reference = generate_circle_reference_path(horizon=config.horizon)
        split_index = (len(full_reference) - 1) // 2
        upper_reference = full_reference[: split_index + 1]
        lower_reference = full_reference[split_index:]
        upper = simulate_tracking_reference("circle_upper", upper_reference, config, max_steps=max_steps)
        lower = simulate_tracking_reference(
            "circle_lower",
            lower_reference,
            config,
            start_state=upper["actual_path"][-1],
            max_steps=max_steps,
        )
        return _merge_segment_results("circle", [upper, lower], reference_path=full_reference)
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
