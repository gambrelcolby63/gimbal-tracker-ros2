#!/usr/bin/env python3
"""Closed-loop flyby: line of sight, lead command, and gimbal response."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def wrap_pi(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    settled = data["t"] >= 1.0
    pan_error = wrap_pi(data["cmd_az"] - data["gimbal_az"])
    tilt_error = data["cmd_el"] - data["gimbal_el"]
    metrics = {
        "duration_s": float(data["t"][-1]),
        "pan_rms_rad_after_1s": rms(pan_error[settled]),
        "tilt_rms_rad_after_1s": rms(tilt_error[settled]),
        "mean_super_elevation_rad": float(np.mean(data["cmd_el"][settled] - data["los_el"][settled])),
        "mean_azimuth_lead_rad": float(np.mean(wrap_pi(data["cmd_az"][settled] - data["los_az"][settled]))),
        "final_position_error_m": float(data["pos_err"][-1]),
        "mean_position_error_m_after_1s": float(np.mean(data["pos_err"][settled])),
        "lead_valid_fraction": float(np.mean(data["valid"])),
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "tracking_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")

    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.grid": True,
            "grid.alpha": 0.35,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.size": 11,
        }
    )
    fig, axes = plt.subplots(3, 1, figsize=(8.2, 7.4), sharex=True)
    axes[0].plot(data["t"], data["los_az"], color="#64748b", lw=1.2, label="Line of sight")
    axes[0].plot(data["t"], data["cmd_az"], color="#1f4e79", lw=1.5, label="Lead command")
    axes[0].plot(data["t"], data["gimbal_az"], color="#d97706", lw=1.4, label="Gimbal")
    axes[0].set_ylabel("Pan (rad)")
    axes[0].legend(frameon=False, ncol=3, loc="lower right")
    axes[0].set_title("Closed-loop flyby")
    axes[1].plot(data["t"], data["los_el"], color="#64748b", lw=1.2, label="Line of sight")
    axes[1].plot(data["t"], data["cmd_el"], color="#1f4e79", lw=1.5, label="Lead command")
    axes[1].plot(data["t"], data["gimbal_el"], color="#0f766e", lw=1.4, label="Gimbal")
    axes[1].set_ylabel("Tilt (rad)")
    axes[1].legend(frameon=False, ncol=3, loc="upper right")
    axes[2].plot(data["t"], data["pos_err"], color="#9a3412", lw=1.6)
    axes[2].set_ylabel("Position error (m)")
    axes[2].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(args.out_dir / "tracking.png", dpi=140)
    plt.close(fig)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
