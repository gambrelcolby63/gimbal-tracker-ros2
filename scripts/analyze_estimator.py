#!/usr/bin/env python3
"""NEES / NIS consistency bands and RMSE curves for the CV Monte Carlo run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2


def band(samples: int, dof_per_sample: int, alpha: float = 0.05):
    total = samples * dof_per_sample
    return (
        float(chi2.ppf(alpha / 2.0, total) / samples),
        float(chi2.ppf(1.0 - alpha / 2.0, total) / samples),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    summary = json.loads(args.summary.read_text())
    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    runs = int(summary["runs"])
    state_dim = int(summary["state_dim"])
    meas_dim = int(summary["measurement_dim"])
    nees_lo, nees_hi = band(runs, state_dim)
    nis_lo, nis_hi = band(runs, meas_dim)

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

    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.0), sharex=True)
    axes[0].plot(data["t"], data["mean_nees"], color="#1f4e79", lw=1.6, label="Mean NEES")
    axes[0].axhline(state_dim, color="#0f766e", ls="--", lw=1.1, label=f"Expected ({state_dim})")
    axes[0].fill_between(data["t"], nees_lo, nees_hi, color="#99f6e4", alpha=0.55, label="95% band")
    axes[0].set_ylabel("NEES")
    axes[0].set_title(f"CV Kalman consistency, {runs} runs")
    axes[0].legend(frameon=False, loc="upper right")
    axes[1].plot(data["t"], data["mean_nis"], color="#9a3412", lw=1.6, label="Mean NIS")
    axes[1].axhline(meas_dim, color="#0f766e", ls="--", lw=1.1, label=f"Expected ({meas_dim})")
    axes[1].fill_between(data["t"], nis_lo, nis_hi, color="#fed7aa", alpha=0.55, label="95% band")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("NIS")
    axes[1].legend(frameon=False, loc="upper right")
    fig.tight_layout()
    fig.savefig(args.out_dir / "nees.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.2, 4.4))
    ax.plot(data["t"], data["pos_rmse"], color="#1f4e79", lw=1.8, label="Position RMSE")
    ax.plot(data["t"], data["vel_rmse"], color="#c2410c", lw=1.8, label="Velocity RMSE")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("RMSE")
    ax.set_title("Matched-noise CV filter error")
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(args.out_dir / "rmse.png", dpi=140)
    plt.close(fig)

    metrics = {
        "runs": runs,
        "state_dim": state_dim,
        "measurement_dim": meas_dim,
        "average_nees": float(np.mean(data["mean_nees"])),
        "average_nis": float(np.mean(data["mean_nis"])),
        "nees_95_band": [nees_lo, nees_hi],
        "nis_95_band": [nis_lo, nis_hi],
        "final_pos_rmse_m": float(data["pos_rmse"][-1]),
        "final_vel_rmse_m_s": float(data["vel_rmse"][-1]),
    }
    (args.out_dir / "estimator_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
