import os

import matplotlib.pyplot as plt
import numpy as np


def plot_tracking_result(reference_path: np.ndarray, actual_path: np.ndarray) -> None:
    plt.plot(reference_path[:, 0], reference_path[:, 1], "-r", label="ref")
    plt.plot(actual_path[:, 0], actual_path[:, 1], "-b", label="mpc")
    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="lower right")
    plt.show()


def save_tracking_gif(reference_path: np.ndarray, actual_path: np.ndarray, output_path: str) -> None:
    import matplotlib.animation as animation

    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    fig = plt.figure()
    plt.plot(reference_path[:, 0], reference_path[:, 1], "-r", label="ref")
    plt.axis("equal")
    plt.grid(True)

    frames = []
    for i in range(len(actual_path)):
        frame = plt.plot(actual_path[: i + 1, 0], actual_path[: i + 1, 1], "ob", label="mpc")
        frames.append(frame)

    ani = animation.ArtistAnimation(fig, frames, interval=10, repeat_delay=1)
    ani.save(output_path, writer="pillow")
    plt.close(fig)
