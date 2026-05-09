import csv
import math
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


def _set_robot_artists(body_patch: Polygon, left_wheel, right_wheel, state: np.ndarray) -> None:
    body_patch.set_xy(robot_body_points(state))
    left_segment, right_segment = robot_wheel_segments(state)
    left_wheel.set_data(left_segment[:, 0], left_segment[:, 1])
    right_wheel.set_data(right_segment[:, 0], right_segment[:, 1])


def plot_tracking_result(reference_path: np.ndarray, actual_path: np.ndarray) -> None:
    plt.plot(reference_path[:, 0], reference_path[:, 1], "-r", label="reference")
    plt.plot(actual_path[:, 0], actual_path[:, 1], "-b", label="actual")
    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="lower right")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()


def _configure_axis(ax, reference_path: np.ndarray, actual_path: np.ndarray, title: str | None = None) -> None:
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if title:
        ax.set_title(title)

    margin = 0.25
    all_xy = np.vstack([reference_path[:, :2], actual_path[:, :2]])
    ax.set_xlim(all_xy[:, 0].min() - margin, all_xy[:, 0].max() + margin)
    ax.set_ylim(all_xy[:, 1].min() - margin, all_xy[:, 1].max() + margin)


def _create_tracking_artists(ax, reference_path: np.ndarray, actual_path: np.ndarray):
    ax.plot(reference_path[:, 0], reference_path[:, 1], "-r", label="reference")
    ax.plot(reference_path[0, 0], reference_path[0, 1], "go", markersize=6, label="start")
    ax.plot(reference_path[-1, 0], reference_path[-1, 1], "r*", markersize=9, label="goal")
    actual_line, = ax.plot([], [], "-b", label="actual")
    actual_points, = ax.plot([], [], "ob", markersize=2.5)
    predicted_line, = ax.plot([], [], "--g", linewidth=1.2, label="MPC prediction")
    heading_arrow = ax.quiver([], [], [], [], angles="xy", scale_units="xy", scale=1.0, color="k")
    body_patch = Polygon(robot_body_points(actual_path[0]), closed=True, fill=False, linewidth=1.5)
    left_wheel, = ax.plot([], [], "k", linewidth=1.8)
    right_wheel, = ax.plot([], [], "k", linewidth=1.8)
    ax.add_patch(body_patch)
    info_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=8)
    return {
        "actual_line": actual_line,
        "actual_points": actual_points,
        "predicted_line": predicted_line,
        "heading_arrow": heading_arrow,
        "body_patch": body_patch,
        "left_wheel": left_wheel,
        "right_wheel": right_wheel,
        "info_text": info_text,
    }


def _update_tracking_artists(artists: dict, result: dict, frame_idx: int):
    actual_path = result["actual_path"]
    predicted_paths = result.get("predicted_paths_for_gif")
    controls = result.get("controls")
    errors = result.get("errors")

    frame_idx = min(frame_idx, len(actual_path) - 1)
    artists["actual_line"].set_data(actual_path[: frame_idx + 1, 0], actual_path[: frame_idx + 1, 1])
    artists["actual_points"].set_data(actual_path[: frame_idx + 1, 0], actual_path[: frame_idx + 1, 1])

    if predicted_paths is not None and frame_idx < len(predicted_paths):
        pred = predicted_paths[frame_idx]
        artists["predicted_line"].set_data(pred[:, 0], pred[:, 1])
    else:
        artists["predicted_line"].set_data([], [])

    state = actual_path[frame_idx]
    artists["heading_arrow"].set_offsets([[state[0], state[1]]])
    artists["heading_arrow"].set_UVC([0.12 * np.cos(state[2])], [0.12 * np.sin(state[2])])
    _set_robot_artists(artists["body_patch"], artists["left_wheel"], artists["right_wheel"], state)

    text_lines = [f"step: {frame_idx}"]
    if errors is not None and frame_idx < len(errors):
        text_lines.append(f"error: {errors[frame_idx]:.3f} m")
    if controls is not None and frame_idx > 0 and frame_idx - 1 < len(controls):
        v, omega = controls[frame_idx - 1]
        text_lines.append(f"v: {v:.2f} m/s")
        text_lines.append(f"w: {omega:.2f} rad/s")
    artists["info_text"].set_text("\n".join(text_lines))
    return tuple(artists.values())


def save_tracking_gif(
    reference_path: np.ndarray,
    actual_path: np.ndarray,
    output_path: str,
    predicted_paths: Sequence[np.ndarray] | None = None,
    controls: np.ndarray | None = None,
    errors: np.ndarray | None = None,
    robot_arrow_length: float = 0.12,
) -> None:
    """Save an enhanced single-scenario NMPC tracking animation."""
    import matplotlib.animation as animation

    ensure_parent_dir(output_path)
    fig, ax = plt.subplots(figsize=(7, 5))
    _configure_axis(ax, reference_path, actual_path)
    result = {
        "actual_path": actual_path,
        "predicted_paths_for_gif": predicted_paths,
        "controls": controls,
        "errors": errors,
    }
    artists = _create_tracking_artists(ax, reference_path, actual_path)
    ax.legend(loc="lower right")

    def update(frame_idx: int):
        return _update_tracking_artists(artists, result, frame_idx)

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
    """Save one GIF showing multiple scenarios at the same time."""
    import matplotlib.animation as animation

    ensure_parent_dir(output_path)
    if not scenario_results:
        raise ValueError("scenario_results must not be empty")

    n_scenarios = len(scenario_results)
    n_cols = min(3, n_scenarios)
    n_rows = math.ceil(n_scenarios / n_cols)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5.2 * n_cols, 4.2 * n_rows), squeeze=False)
    axes_flat = axes.ravel()

    all_artists = []
    for idx, result in enumerate(scenario_results):
        ax = axes_flat[idx]
        _configure_axis(ax, result["reference_path"], result["actual_path"], title=result["name"])
        artists = _create_tracking_artists(ax, result["reference_path"], result["actual_path"])
        all_artists.append(artists)

    for idx in range(n_scenarios, len(axes_flat)):
        axes_flat[idx].axis("off")

    axes_flat[0].legend(loc="lower right", fontsize=8)
    fig.suptitle("NMPC Multi-scenario Tracking Demo", fontsize=14)
    fig.tight_layout()

    max_frames = max(len(result["actual_path"]) for result in scenario_results)

    def update(global_frame_idx: int):
        updated = []
        for result, artists in zip(scenario_results, all_artists):
            updated.extend(_update_tracking_artists(artists, result, global_frame_idx))
        return updated

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=max_frames,
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
