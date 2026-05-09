import csv
import os
from typing import Sequence

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np


def ensure_parent_dir(path: str) -> None:
    output_dir = os.path.dirname(path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)


def _transform_points(local_points: np.ndarray, state: np.ndarray) -> np.ndarray:
    x, y, theta = state
    rot = np.array(
        [
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ]
    )
    return local_points @ rot.T + np.array([x, y])


def robot_body_points(state: np.ndarray, length: float = 0.22, width: float = 0.14) -> np.ndarray:
    local_body = np.array(
        [
            [length / 2.0, 0.0],
            [length * 0.15, width / 2.0],
            [-length / 2.0, width / 2.0],
            [-length / 2.0, -width / 2.0],
            [length * 0.15, -width / 2.0],
        ]
    )
    return _transform_points(local_body, state)


def robot_wheel_segments(state: np.ndarray, length: float = 0.22, width: float = 0.14) -> tuple[np.ndarray, np.ndarray]:
    wheel_length = length * 0.55
    left_wheel = np.array(
        [
            [-wheel_length / 2.0, width / 2.0 + 0.025],
            [wheel_length / 2.0, width / 2.0 + 0.025],
        ]
    )
    right_wheel = np.array(
        [
            [-wheel_length / 2.0, -width / 2.0 - 0.025],
            [wheel_length / 2.0, -width / 2.0 - 0.025],
        ]
    )
    return _transform_points(left_wheel, state), _transform_points(right_wheel, state)


def plot_tracking_result(reference_path: np.ndarray, actual_path: np.ndarray) -> None:
    plt.plot(reference_path[:, 0], reference_path[:, 1], "-r", label="reference")
    plt.plot(actual_path[:, 0], actual_path[:, 1], "-b", label="actual")
    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="lower right")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()


def _set_robot_artists(body_patch: Polygon, left_wheel, right_wheel, state: np.ndarray) -> None:
    body_patch.set_xy(robot_body_points(state))
    left_segment, right_segment = robot_wheel_segments(state)
    left_wheel.set_data(left_segment[:, 0], left_segment[:, 1])
    right_wheel.set_data(right_segment[:, 0], right_segment[:, 1])


def save_tracking_gif(
    reference_path: np.ndarray,
    actual_path: np.ndarray,
    output_path: str,
    predicted_paths: Sequence[np.ndarray] | None = None,
    controls: np.ndarray | None = None,
    errors: np.ndarray | None = None,
    robot_arrow_length: float = 0.12,
) -> None:
    """Save an enhanced NMPC tracking animation."""
    import matplotlib.animation as animation

    ensure_parent_dir(output_path)

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(reference_path[:, 0], reference_path[:, 1], "-r", label="reference")
    ax.plot(reference_path[0, 0], reference_path[0, 1], "go", markersize=7, label="start")
    ax.plot(reference_path[-1, 0], reference_path[-1, 1], "r*", markersize=10, label="goal")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    margin = 0.25
    all_xy = np.vstack([reference_path[:, :2], actual_path[:, :2]])
    ax.set_xlim(all_xy[:, 0].min() - margin, all_xy[:, 0].max() + margin)
    ax.set_ylim(all_xy[:, 1].min() - margin, all_xy[:, 1].max() + margin)

    actual_line, = ax.plot([], [], "-b", label="actual")
    actual_points, = ax.plot([], [], "ob", markersize=3)
    predicted_line, = ax.plot([], [], "--g", linewidth=1.5, label="MPC prediction")
    heading_arrow = ax.quiver([], [], [], [], angles="xy", scale_units="xy", scale=1.0, color="k")
    body_patch = Polygon(robot_body_points(actual_path[0]), closed=True, fill=False, linewidth=1.8)
    left_wheel, = ax.plot([], [], "k", linewidth=2.0)
    right_wheel, = ax.plot([], [], "k", linewidth=2.0)
    ax.add_patch(body_patch)
    info_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")
    ax.legend(loc="lower right")

    def update(frame_idx: int):
        actual_line.set_data(actual_path[: frame_idx + 1, 0], actual_path[: frame_idx + 1, 1])
        actual_points.set_data(actual_path[: frame_idx + 1, 0], actual_path[: frame_idx + 1, 1])

        if predicted_paths is not None and frame_idx < len(predicted_paths):
            pred = predicted_paths[frame_idx]
            predicted_line.set_data(pred[:, 0], pred[:, 1])
        else:
            predicted_line.set_data([], [])

        state = actual_path[frame_idx]
        dx = robot_arrow_length * np.cos(state[2])
        dy = robot_arrow_length * np.sin(state[2])
        heading_arrow.set_offsets([[state[0], state[1]]])
        heading_arrow.set_UVC([dx], [dy])
        _set_robot_artists(body_patch, left_wheel, right_wheel, state)

        text_lines = [f"step: {frame_idx}"]
        if errors is not None and frame_idx < len(errors):
            text_lines.append(f"position error: {errors[frame_idx]:.4f} m")
        if controls is not None and frame_idx > 0 and frame_idx - 1 < len(controls):
            v, omega = controls[frame_idx - 1]
            text_lines.append(f"v: {v:.3f} m/s")
            text_lines.append(f"omega: {omega:.3f} rad/s")
        info_text.set_text("\n".join(text_lines))

        return actual_line, actual_points, predicted_line, heading_arrow, body_patch, left_wheel, right_wheel, info_text

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(actual_path),
        interval=40,
        blit=False,
        repeat_delay=500,
    )
    ani.save(output_path, writer="pillow")
    plt.close(fig)


def save_multi_scenario_gif(scenario_results: Sequence[dict], output_path: str) -> None:
    """Save one GIF that plays multiple tracking scenarios sequentially."""
    import matplotlib.animation as animation

    ensure_parent_dir(output_path)
    if not scenario_results:
        raise ValueError("scenario_results must not be empty")

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    all_xy = []
    for result in scenario_results:
        all_xy.append(result["reference_path"][:, :2])
        all_xy.append(result["actual_path"][:, :2])
    all_xy = np.vstack(all_xy)
    margin = 0.3
    ax.set_xlim(all_xy[:, 0].min() - margin, all_xy[:, 0].max() + margin)
    ax.set_ylim(all_xy[:, 1].min() - margin, all_xy[:, 1].max() + margin)

    reference_line, = ax.plot([], [], "-r", label="reference")
    start_marker, = ax.plot([], [], "go", markersize=7, label="start")
    goal_marker, = ax.plot([], [], "r*", markersize=10, label="goal")
    actual_line, = ax.plot([], [], "-b", label="actual")
    actual_points, = ax.plot([], [], "ob", markersize=3)
    predicted_line, = ax.plot([], [], "--g", linewidth=1.5, label="MPC prediction")
    heading_arrow = ax.quiver([], [], [], [], angles="xy", scale_units="xy", scale=1.0, color="k")
    body_patch = Polygon(robot_body_points(scenario_results[0]["actual_path"][0]), closed=True, fill=False, linewidth=1.8)
    left_wheel, = ax.plot([], [], "k", linewidth=2.0)
    right_wheel, = ax.plot([], [], "k", linewidth=2.0)
    ax.add_patch(body_patch)
    title_text = ax.text(0.5, 1.02, "", transform=ax.transAxes, ha="center", va="bottom", fontsize=12)
    info_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")
    ax.legend(loc="lower right")

    frame_map = []
    for scenario_idx, result in enumerate(scenario_results):
        for local_idx in range(len(result["actual_path"])):
            frame_map.append((scenario_idx, local_idx))
        # Hold the last frame briefly before switching to the next scenario.
        for _ in range(12):
            frame_map.append((scenario_idx, len(result["actual_path"]) - 1))

    def update(global_frame_idx: int):
        scenario_idx, frame_idx = frame_map[global_frame_idx]
        result = scenario_results[scenario_idx]
        reference_path = result["reference_path"]
        actual_path = result["actual_path"]
        predicted_paths = result.get("predicted_paths_for_gif")
        controls = result.get("controls")
        errors = result.get("errors")

        reference_line.set_data(reference_path[:, 0], reference_path[:, 1])
        start_marker.set_data([reference_path[0, 0]], [reference_path[0, 1]])
        goal_marker.set_data([reference_path[-1, 0]], [reference_path[-1, 1]])
        actual_line.set_data(actual_path[: frame_idx + 1, 0], actual_path[: frame_idx + 1, 1])
        actual_points.set_data(actual_path[: frame_idx + 1, 0], actual_path[: frame_idx + 1, 1])

        if predicted_paths is not None and frame_idx < len(predicted_paths):
            pred = predicted_paths[frame_idx]
            predicted_line.set_data(pred[:, 0], pred[:, 1])
        else:
            predicted_line.set_data([], [])

        state = actual_path[frame_idx]
        heading_arrow.set_offsets([[state[0], state[1]]])
        heading_arrow.set_UVC([0.12 * np.cos(state[2])], [0.12 * np.sin(state[2])])
        _set_robot_artists(body_patch, left_wheel, right_wheel, state)

        title_text.set_text(f"Scenario {scenario_idx + 1}/{len(scenario_results)}: {result['name']}")
        text_lines = [f"step: {frame_idx}"]
        if errors is not None and frame_idx < len(errors):
            text_lines.append(f"position error: {errors[frame_idx]:.4f} m")
        if controls is not None and frame_idx > 0 and frame_idx - 1 < len(controls):
            v, omega = controls[frame_idx - 1]
            text_lines.append(f"v: {v:.3f} m/s")
            text_lines.append(f"omega: {omega:.3f} rad/s")
        info_text.set_text("\n".join(text_lines))

        return (
            reference_line,
            start_marker,
            goal_marker,
            actual_line,
            actual_points,
            predicted_line,
            heading_arrow,
            body_patch,
            left_wheel,
            right_wheel,
            title_text,
            info_text,
        )

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(frame_map),
        interval=40,
        blit=False,
        repeat_delay=500,
    )
    ani.save(output_path, writer="pillow")
    plt.close(fig)


def save_control_profile(controls: np.ndarray, dt: float, output_path: str) -> None:
    ensure_parent_dir(output_path)
    time_axis = np.arange(len(controls)) * dt

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(time_axis, controls[:, 0], label="v [m/s]")
    ax.plot(time_axis, controls[:, 1], label="omega [rad/s]")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("control")
    ax.set_title("MPC Control Profile")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def save_error_profile(errors: np.ndarray, dt: float, output_path: str) -> None:
    ensure_parent_dir(output_path)
    time_axis = np.arange(len(errors)) * dt

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(time_axis, errors, label="position error [m]")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error [m]")
    ax.set_title("Tracking Error")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def save_demo_log(states: np.ndarray, controls: np.ndarray, errors: np.ndarray, output_path: str) -> None:
    ensure_parent_dir(output_path)
    with open(output_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["step", "x", "y", "theta", "v", "omega", "position_error"])
        for step, state in enumerate(states):
            if step == 0:
                v, omega = 0.0, 0.0
            else:
                v, omega = controls[step - 1]
            writer.writerow([step, state[0], state[1], state[2], v, omega, errors[step]])
