#!/usr/bin/env python3
"""RMSE, NEES, and model-probability plots for the CV versus IMM study."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2

KIND_COLOR = {
    "straight": "#e2e8f0",
    "coordinated_turn": "#dbeafe",
    "speed_change": "#ffedd5",
    "weave": "#ede9fe",
}
KIND_LABEL = {
    "straight": "Straight",
    "coordinated_turn": "Turn",
    "speed_change": "Speed",
    "weave": "Weave",
}
CV = "#be123c"
IMM = "#0f766e"
INK = "#1e293b"


def band(samples: int, dof_per_sample: int, alpha: float = 0.05) -> tuple[float, float]:
    total = samples * dof_per_sample
    return (
        float(chi2.ppf(alpha / 2.0, total) / samples),
        float(chi2.ppf(1.0 - alpha / 2.0, total) / samples),
    )


def spans(time: np.ndarray, kind: np.ndarray) -> list[tuple[float, float, str]]:
    dt = float(time[1] - time[0]) if len(time) > 1 else 0.0
    out: list[tuple[float, float, str]] = []
    start = 0
    for index in range(1, len(kind) + 1):
        if index == len(kind) or kind[index] != kind[start]:
            end = float(time[index]) if index < len(time) else float(time[-1] + dt)
            out.append((float(time[start]), end, str(kind[start])))
            start = index
    return out


def shade(ax, regions: list[tuple[float, float, str]]) -> None:
    for start, end, kind in regions:
        ax.axvspan(start, end, color=KIND_COLOR.get(kind, "#f1f5f9"), alpha=0.85, lw=0, zorder=0)


def style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.grid": True,
            "grid.alpha": 0.35,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.size": 11,
            "axes.labelcolor": INK,
            "xtick.color": INK,
            "ytick.color": INK,
            "text.color": INK,
        }
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    summary = json.loads(args.summary.read_text())
    data = np.genfromtxt(args.csv, delimiter=",", names=True, dtype=None, encoding="utf-8")
    time = np.asarray(data["t"], dtype=float)
    regions = spans(time, data["kind"])
    runs = int(summary["runs"])
    state_dim = int(summary["state_dim"])
    nees_lo, nees_hi = band(runs, state_dim)
    turn_cv = float(summary["turn_pos_rmse_cv"])
    turn_imm = float(summary["turn_pos_rmse_imm"])
    if not np.isfinite(turn_imm) or not np.isfinite(turn_cv) or turn_imm >= turn_cv:
        raise SystemExit(
            f"IMM turn position RMSE ({turn_imm:.3f} m) is not below CV ({turn_cv:.3f} m)."
        )

    style()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(2, 1, figsize=(8.4, 6.4), sharex=True)
    for ax, cv_key, imm_key, ylabel, title in (
        (axes[0], "cv_pos_rmse", "imm_pos_rmse", "Position RMSE (m)", "Position"),
        (axes[1], "cv_vel_rmse", "imm_vel_rmse", "Velocity RMSE (m/s)", "Velocity"),
    ):
        shade(ax, regions)
        ax.plot(time, data[cv_key], color=CV, lw=1.8, label="Constant velocity", zorder=2)
        ax.plot(time, data[imm_key], color=IMM, lw=1.8, label="IMM", zorder=2)
        ax.set_ylabel(ylabel)
        ax.set_title(title, loc="left", color=INK)
    axes[0].legend(frameon=False, loc="upper left")
    axes[1].set_xlabel("Time (s)")
    fig.suptitle(f"Maneuvering target, {runs} Monte Carlo runs", fontsize=13, color=INK)
    fig.tight_layout()
    fig.savefig(args.out_dir / "imm_rmse.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.4, 6.4), sharex=True)
    shade(axes[0], regions)
    shade(axes[1], regions)
    axes[0].plot(time, data["imm_nees"], color=IMM, lw=1.6, label="IMM mean NEES", zorder=2)
    axes[0].axhline(state_dim, color="#1d4ed8", ls="--", lw=1.1, label=f"Expected ({state_dim})", zorder=2)
    axes[0].fill_between(time, nees_lo, nees_hi, color="#99f6e4", alpha=0.45, label="95% band", zorder=1)
    axes[0].set_ylabel("NEES")
    axes[0].set_title("IMM, kinematic state", loc="left", color=INK)
    axes[0].text(
        0.98,
        0.92,
        f"Mean {float(summary['mean_nees_imm']):.0f}",
        transform=axes[0].transAxes,
        ha="right",
        va="top",
        color=INK,
        fontsize=10,
    )
    axes[0].legend(frameon=False, loc="upper left")
    axes[1].plot(time, data["cv_nees"], color=CV, lw=1.6, label="CV mean NEES", zorder=2)
    axes[1].axhline(state_dim, color="#1d4ed8", ls="--", lw=1.1, label=f"Expected ({state_dim})", zorder=2)
    axes[1].set_ylabel("NEES")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Constant velocity, same measurements", loc="left", color=INK)
    axes[1].text(
        0.98,
        0.92,
        f"Mean {float(summary['mean_nees_cv']):.0f}",
        transform=axes[1].transAxes,
        ha="right",
        va="top",
        color=INK,
        fontsize=10,
    )
    axes[1].legend(frameon=False, loc="upper left")
    fig.suptitle("NEES on the maneuver. Scales differ.", fontsize=13, color=INK)
    fig.tight_layout()
    fig.savefig(args.out_dir / "imm_nees.png", dpi=140)
    plt.close(fig)

    fig = plt.figure(figsize=(8.4, 5.0))
    phase = fig.add_axes([0.10, 0.84, 0.86, 0.07])
    ax = fig.add_axes([0.10, 0.22, 0.86, 0.56])
    phase.set_xlim(float(time[0]), float(time[-1]))
    phase.set_ylim(0.0, 1.0)
    phase.set_yticks([])
    phase.set_xticks([])
    phase.tick_params(length=0)
    for spine in phase.spines.values():
        spine.set_visible(False)
    for start, end, kind in regions:
        phase.add_patch(
            plt.Rectangle(
                (start, 0.0),
                end - start,
                1.0,
                facecolor=KIND_COLOR.get(kind, "#f1f5f9"),
                edgecolor="white",
                lw=0.8,
            )
        )
        phase.text(
            0.5 * (start + end),
            0.5,
            KIND_LABEL.get(kind, kind),
            ha="center",
            va="center",
            fontsize=9,
            color=INK,
        )
    ax.stackplot(
        time,
        data["mu_cv"],
        data["mu_ca"],
        data["mu_ct"],
        colors=["#64748b", "#ea580c", "#0f766e"],
        labels=["Constant velocity", "Constant acceleration", "Coordinated turn"],
        zorder=2,
        alpha=0.95,
    )
    ax.set_ylim(0.0, 1.0)
    ax.set_xlim(float(time[0]), float(time[-1]))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Model probability")
    ax.set_title(f"Mean IMM probabilities, {runs} runs", loc="left", color=INK)
    ax.grid(True, color="#e2e8f0", lw=0.6)
    ax.legend(
        frameon=False,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.20),
        ncol=3,
    )
    fig.savefig(args.out_dir / "imm_probabilities.png", dpi=140)
    plt.close(fig)

    metrics = {
        "runs": runs,
        "state_dim": state_dim,
        "duration_s": float(summary["duration_s"]),
        "meas_std_m": float(summary["meas_std_m"]),
        "turn_pos_rmse_cv_m": turn_cv,
        "turn_pos_rmse_imm_m": turn_imm,
        "straight_pos_rmse_cv_m": float(summary["straight_pos_rmse_cv"]),
        "straight_pos_rmse_imm_m": float(summary["straight_pos_rmse_imm"]),
        "speed_pos_rmse_cv_m": float(summary["speed_pos_rmse_cv"]),
        "speed_pos_rmse_imm_m": float(summary["speed_pos_rmse_imm"]),
        "weave_pos_rmse_cv_m": float(summary["weave_pos_rmse_cv"]),
        "weave_pos_rmse_imm_m": float(summary["weave_pos_rmse_imm"]),
        "turn_vel_rmse_cv_m_s": float(summary["turn_vel_rmse_cv"]),
        "turn_vel_rmse_imm_m_s": float(summary["turn_vel_rmse_imm"]),
        "mean_nees_cv": float(summary["mean_nees_cv"]),
        "mean_nees_imm": float(summary["mean_nees_imm"]),
        "nees_95_band": [nees_lo, nees_hi],
        "ct_probability_during_turn": float(summary["ct_probability_during_turn"]),
        "ca_probability_during_speed_change": float(summary["ca_probability_during_speed_change"]),
        "cv_probability_during_straight": float(summary["cv_probability_during_straight"]),
        "imm_turn_rmse_lower": True,
    }
    (args.out_dir / "imm_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
