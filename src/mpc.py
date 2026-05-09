"""Backward-compatible entry point for the NMPC tracking demo.

The implementation has been split into:
- mpc_controller.py: reusable MPC controller
- demo_tracking.py: runnable demo
- visualization.py: plotting and GIF utilities
"""

from demo_tracking import main


if __name__ == "__main__":
    main()
