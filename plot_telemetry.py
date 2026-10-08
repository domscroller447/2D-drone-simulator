"""Offline visualization of drone flight-simulator telemetry.

Reads the CSV log produced by the C++ simulator (see Drone.cpp's
kLogFilePath / kLogHeader) and renders a three-panel figure comparing raw,
noisy IMU measurements against the exponential-moving-average (EMA)
filtered state and, where available, ground truth: altitude, horizontal
velocity, and the resulting pitch/thrust actuator response.
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

# NOTE: must match the path Drone.cpp writes to (kLogFilePath in Drone.cpp).
DEFAULT_LOG_PATH = Path(r"C:/Users/ADMIN/Desktop/drone_sim/flight_log.csv")

FIGURE_SIZE = (11.0, 8.5)
WINDOW_TITLE = "IMU Noise & State Filtering Analysis"

PREFERRED_STYLE = "seaborn-v0_8-darkgrid"
FALLBACK_STYLE = "default"

# Shared style dicts: each is applied at several call sites below, so
# defining them once here means a style tweak never has to be repeated
# (and can't accidentally drift between subplots).
AXIS_LABEL_STYLE = {"fontsize": 11, "fontweight": "bold"}
TITLE_STYLE = {"fontsize": 13, "fontweight": "bold", "pad": 10}
GRID_STYLE = {"linestyle": "--", "alpha": 0.6}
LEGEND_LOCATION = "upper right"


def _apply_plot_style() -> None:
    """Selects the nicer seaborn grid style if installed, else the default."""
    style = PREFERRED_STYLE if PREFERRED_STYLE in plt.style.available else FALLBACK_STYLE
    plt.style.use(style)


def _load_telemetry(file_path: Path) -> pd.DataFrame:
    """Loads the telemetry CSV and rebases Time to start at zero.

    Args:
        file_path: Path to the CSV log written by the simulator.

    Returns:
        The telemetry as a DataFrame, with "Time" shifted so its first
        sample is 0.

    Raises:
        FileNotFoundError: If file_path does not exist.
    """
    if not file_path.exists():
        raise FileNotFoundError(f"Telemetry log not found at '{file_path}'.")

    df = pd.read_csv(file_path)
    df["Time"] = df["Time"] - df["Time"].iloc[0]
    return df


def _plot_altitude(ax: plt.Axes, df: pd.DataFrame) -> None:
    """Renders the altitude subplot: raw IMU vs. EMA filter vs. ground truth."""
    if "RawHeight" in df.columns:
        ax.plot(df["Time"], df["RawHeight"], label="Raw IMU Height (Noisy)",
                 color="#ff7f0e", alpha=0.4, linewidth=1)
        ax.plot(df["Time"], df["FilteredHeight"], label="EMA Filtered Height",
                 color="#007acc", linewidth=2)
        ax.plot(df["Time"], df["TrueHeight"], label="True Height (Ground Truth)",
                 color="#2ca02c", linestyle="--", linewidth=1.5)
    else:
        ax.plot(df["Time"], df["Height"], label="Height (m)", color="#007acc")

    ax.set_ylabel("Height (m)", **AXIS_LABEL_STYLE)
    ax.set_title(
        "IMU Sensor Noise vs. Exponential Moving Average (EMA) State Filtering",
        **TITLE_STYLE,
    )
    ax.legend(loc=LEGEND_LOCATION)
    ax.grid(True, **GRID_STYLE)


def _plot_velocity(ax: plt.Axes, df: pd.DataFrame) -> None:
    """Renders the horizontal-velocity subplot: raw vs. filtered vs. true Vx."""
    if "RawVx" in df.columns:
        ax.plot(df["Time"], df["RawVx"], label="Raw Vx Noise",
                 color="#d9534f", alpha=0.35, linewidth=1)
        ax.plot(df["Time"], df["FilteredVx"], label="Filtered Vx",
                 color="#8a2be2", linewidth=2)
        ax.plot(df["Time"], df["Vx"], label="True Vx",
                 color="#333333", linestyle=":", linewidth=1.5)
    else:
        ax.plot(df["Time"], df["Vx"], label="Vx Velocity (m/s)", color="#8a2be2")

    ax.set_ylabel("Velocity Vx (m/s)", **AXIS_LABEL_STYLE)
    ax.legend(loc=LEGEND_LOCATION)
    ax.grid(True, **GRID_STYLE)


def _plot_actuator_response(ax: plt.Axes, df: pd.DataFrame) -> None:
    """Renders pitch (left axis) and thrust (right, twin axis) vs. time."""
    ax.plot(df["Time"], df["Pitch"], label="Pitch Angle (deg)",
             color="#28a745", linewidth=1.8)

    thrust_ax = ax.twinx()
    thrust_ax.plot(df["Time"], df["Thrust"], label="Thrust Output (N)",
                    color="#ff8c00", linestyle="--", linewidth=1.5)

    ax.set_xlabel("Flight Time (seconds)", **AXIS_LABEL_STYLE)
    ax.set_ylabel("Pitch (deg)", color="#28a745", **AXIS_LABEL_STYLE)
    thrust_ax.set_ylabel("Thrust (N)", color="#ff8c00", **AXIS_LABEL_STYLE)
    ax.tick_params(axis="y", labelcolor="#28a745")
    thrust_ax.tick_params(axis="y", labelcolor="#ff8c00")

    pitch_lines, pitch_labels = ax.get_legend_handles_labels()
    thrust_lines, thrust_labels = thrust_ax.get_legend_handles_labels()
    ax.legend(pitch_lines + thrust_lines, pitch_labels + thrust_labels, loc=LEGEND_LOCATION)
    ax.grid(True, **GRID_STYLE)


def plot_telemetry_analysis(file_path: Path) -> None:
    """Loads a telemetry CSV and displays the three-panel analysis figure.

    Args:
        file_path: Path to the CSV log written by the simulator.

    Raises:
        FileNotFoundError: If file_path does not exist.
    """
    df = _load_telemetry(file_path)

    _apply_plot_style()
    fig, (altitude_ax, velocity_ax, actuator_ax) = plt.subplots(
        3, 1, figsize=FIGURE_SIZE, sharex=True
    )
    fig.canvas.manager.set_window_title(WINDOW_TITLE)

    _plot_altitude(altitude_ax, df)
    _plot_velocity(velocity_ax, df)
    _plot_actuator_response(actuator_ax, df)

    plt.tight_layout()
    plt.show()


def _parse_args() -> argparse.Namespace:
    """Parses CLI arguments, defaulting to DEFAULT_LOG_PATH when omitted."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log_path",
        type=Path,
        nargs="?",
        default=DEFAULT_LOG_PATH,
        help=f"Path to the telemetry CSV (default: {DEFAULT_LOG_PATH})",
    )
    return parser.parse_args()


def main() -> int:
    """CLI entry point. Returns a process exit code."""
    args = _parse_args()
    try:
        plot_telemetry_analysis(args.log_path)
    except FileNotFoundError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())