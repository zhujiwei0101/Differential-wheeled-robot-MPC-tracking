import csv
import os
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np


def ensure_parent_dir(path: str) -> None:
    output_dir = os.path.dirname(path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)


def plot_tracking_result(reference_path: np.ndarray, actual_path: np.ndarray) -> None:
    plt.plot(reference_path[:, 0], reference_path[:, 1], "-r", label="reference")
    plt.plot(actual_path[:, 0], actual_path[:, 1], "-b", label="actual")
    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="lower right")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()


def save_tracking_gif(
    reference_path: np.ndarray,
    actual_path: np.ndarray,
    output_path: str,
    predicted_paths: Sequence[np.ndarray] | None = None,
    controls: np.ndarray | None = None,
    errors: np.ndarray | None = None,
    robot_arrow_length: float = 0.12,
) -> None:
    """Save an enhanced NMPC tracking animation.

    The animation shows:
    - red line: reference trajectory
    - blue line/points: executed trajectory
    - green dashed line: current MPC predicted horizon
    - black arrow: robot heading
    - text: step, error, and current control
    """
    import matplotlib.animation as animation

    ensure_parent_dir(output_path)

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(reference_path[:, 0], reference_path[:, 1], "-r", label="reference")
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

        text_lines = [f"step: {frame_idx}"]
        if errors is not None and frame_idx < len(errors):
            text_lines.append(f"position error: {errors[frame_idx]:.4f} m")
        if controls is not None and frame_idx > 0 and frame_idx - 1 < len(controls):
            v, omega = controls[frame_idx - 1]
            text_lines.append(f"v: {v:.3f} m/s")
            text_lines.append(f"omega: {omega:.3f} rad/s")
        info_text.set_text("\n".join(text_lines))

        return actual_line, actual_points, predicted_line, heading_arrow, info_text

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
