#!/usr/bin/env python3
"""Step response and Bode margins for the pan cascaded loop.

The linear model is the unsaturated plant (Coulomb friction removed) plus the
same P + PI controller the C++ library integrates. Position ki and both
derivative gains are required to be zero, which is the tuning this model matches.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

import control


def load_axis(config: dict) -> dict:
    plant = config["plant"]["pan"]
    control_axis = config["controller"]["pan"]
    if control_axis["position"]["ki"] != 0 or control_axis["position"]["kd"] != 0:
        raise SystemExit("Linear model expects a P position loop (ki = kd = 0).")
    if control_axis["rate"]["kd"] != 0:
        raise SystemExit("Linear model expects no rate derivative term.")
    return {
        "J": plant["inertia_kg_m2"],
        "b": plant["viscous_n_m_s"],
        "R": plant["resistance_ohm"],
        "L": plant["inductance_h"],
        "Kt": plant["torque_const_n_m_per_a"],
        "Kb": plant["back_emf_v_s"],
        "Kpp": control_axis["position"]["kp"],
        "Kpr": control_axis["rate"]["kp"],
        "Ki": control_axis["rate"]["ki"],
        "Kffp": control_axis["position_kff"],
        "Kffr": control_axis["rate_kff"],
        "Kaff": control_axis["rate_kaff"],
        "step": config["analysis"]["step_rad"],
    }


def closed_loop(p: dict):
    """States [theta, omega, current, rate-integrator]. Input theta_ref."""
    J, b, R, L = p["J"], p["b"], p["R"], p["L"]
    Kt, Kb = p["Kt"], p["Kb"]
    Kpp, Kpr, Ki = p["Kpp"], p["Kpr"], p["Ki"]
    Kffr = p["Kffr"]
    A = np.zeros((4, 4))
    B = np.zeros((4, 1))
    C = np.array([[1.0, 0.0, 0.0, 0.0]])
    D = np.zeros((1, 1))
    A[0, 1] = 1.0
    A[1, 1] = -b / J
    A[1, 2] = Kt / J
    du_dtheta = -(Kpr + Kffr) * Kpp
    du_domega = -Kpr
    A[2, 0] = du_dtheta / L
    A[2, 1] = (-Kb + du_domega) / L
    A[2, 2] = -R / L
    A[2, 3] = Ki / L
    A[3, 0] = -Kpp
    A[3, 1] = -1.0
    B[2, 0] = (Kpr + Kffr) * Kpp / L
    B[3, 0] = Kpp
    return control.ss(A, B, C, D)


def loop_transfers(p: dict):
    J, b, R, L = p["J"], p["b"], p["R"], p["L"]
    Kt, Kb = p["Kt"], p["Kb"]
    Kpp, Kpr, Ki, Kffr = p["Kpp"], p["Kpr"], p["Ki"], p["Kffr"]
    plant = control.ss(
        [[-b / J, Kt / J], [-Kb / L, -R / L]],
        [[0.0], [1.0 / L]],
        [[1.0, 0.0]],
        0.0,
    )
    pi = control.tf([Kpr, Ki], [1.0, 0.0])
    inner_loop = pi * plant

    # Closed inner loop from rate command to rate, including the rate feedforward.
    A = np.array(
        [
            [-b / J, Kt / J, 0.0],
            [(-Kb - Kpr) / L, -R / L, Ki / L],
            [-1.0, 0.0, 0.0],
        ]
    )
    B = np.array([[0.0], [(Kpr + Kffr) / L], [1.0]])
    C = np.array([[1.0, 0.0, 0.0]])
    inner_closed = control.ss(A, B, C, 0.0)
    outer_loop = Kpp * inner_closed * control.tf([1.0], [1.0, 0.0])
    return inner_loop, outer_loop


def as_time_series(response):
    if hasattr(response, "time"):
        time = np.asarray(response.time, dtype=float).reshape(-1)
        output = np.asarray(response.outputs, dtype=float).reshape(-1)
        return time, output
    time, output = response[0], response[1]
    return np.asarray(time, dtype=float).reshape(-1), np.asarray(output, dtype=float).reshape(-1)


def step_metrics(time: np.ndarray, theta: np.ndarray, command: float) -> dict:
    band = 0.02 * abs(command)
    t10 = t90 = None
    peak = float(np.max(theta))
    last_outside = 0.0
    for t, y in zip(time, theta):
        fraction = y / command
        if t10 is None and fraction >= 0.1:
            t10 = float(t)
        if t90 is None and fraction >= 0.9:
            t90 = float(t)
        if abs(y - command) > band:
            last_outside = float(t)
    tail = time >= (time[-1] - 0.5)
    ess = float(abs(np.mean(theta[tail] - command)))
    return {
        "rise_time_s": None if t10 is None or t90 is None else t90 - t10,
        "overshoot_percent": max(0.0, (peak - command) / command * 100.0),
        "settling_time_s": last_outside,
        "steady_state_error_rad": ess,
        "peak_rad": peak,
    }


def margin_dict(sys) -> dict:
    gm, pm, wcg, wcp = control.margin(sys)

    def clean(value):
        if value is None:
            return None
        value = float(value)
        if not np.isfinite(value):
            return None
        return value

    gm = clean(gm)
    return {
        "gain_margin_db": None if gm is None or gm <= 0.0 else 20.0 * np.log10(gm),
        "phase_margin_deg": clean(pm),
        "phase_crossover_rad_s": clean(wcg),
        "gain_crossover_rad_s": clean(wcp),
    }


def read_csv(path: Path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data["t"], data["theta"], data["theta_ref"]


def bode_pair(axis, sys, omega, title: str, margins: dict):
    response = control.frequency_response(sys, omega)
    magnitude = np.squeeze(np.asarray(response.magnitude, dtype=float))
    phase = np.unwrap(np.squeeze(np.asarray(response.phase, dtype=float)))
    mag_db = 20.0 * np.log10(np.maximum(magnitude, 1e-16))
    phase_deg = np.degrees(phase)
    axis[0].semilogx(omega, mag_db, color="#1f4e79", lw=1.8)
    axis[0].axhline(0.0, color="#94a3b8", lw=0.8)
    axis[0].set_ylabel("Magnitude (dB)")
    axis[0].set_title(title)
    axis[1].semilogx(omega, phase_deg, color="#1f4e79", lw=1.8)
    axis[1].axhline(-180.0, color="#94a3b8", lw=0.8)
    axis[1].set_ylabel("Phase (deg)")
    axis[1].set_xlabel("Frequency (rad/s)")
    wcp = margins.get("gain_crossover_rad_s")
    if wcp:
        axis[0].axvline(wcp, color="#c2410c", ls="--", lw=0.9)
        axis[1].axvline(wcp, color="#c2410c", ls="--", lw=0.9)
    gm = margins["gain_margin_db"]
    pm = margins["phase_margin_deg"]
    gm_text = "inf" if gm is None else f"{gm:.1f} dB"
    pm_text = "n/a" if pm is None else f"{pm:.1f} deg"
    axis[0].text(
        0.02,
        0.05,
        f"GM {gm_text}\nPM {pm_text}",
        transform=axis[0].transAxes,
        fontsize=9,
        va="bottom",
        bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "#cbd5e1"},
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--friction-csv", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    with args.config.open() as handle:
        config = yaml.safe_load(handle)
    params = load_axis(config)
    time, theta, theta_ref = read_csv(args.csv)
    _, theta_friction, _ = read_csv(args.friction_csv)
    command = float(theta_ref[0])
    if not np.allclose(theta_ref, command):
        raise SystemExit("Step CSV command is not constant.")

    plant_model = closed_loop(params)
    _, linear = as_time_series(control.forced_response(plant_model, time, np.full_like(time, command)))
    rms = float(np.sqrt(np.mean((linear - theta) ** 2)))
    sim_metrics = step_metrics(time, theta, command)
    linear_metrics = step_metrics(time, linear, command)
    friction_metrics = step_metrics(time, theta_friction, command)

    inner, outer = loop_transfers(params)
    inner_m = margin_dict(inner)
    outer_m = margin_dict(outer)
    metrics = {
        "simulation_no_coulomb": sim_metrics,
        "simulation_with_coulomb": friction_metrics,
        "linear_model": linear_metrics,
        "linear_vs_simulation_rms_rad": rms,
        "inner_rate_loop": inner_m,
        "outer_position_loop": outer_m,
        "step_rad": command,
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "control_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")

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
    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    ax.plot(time, np.full_like(time, command), color="#64748b", ls="--", lw=1.2, label="Command")
    ax.plot(time, linear, color="#1f4e79", lw=2.0, label="Linear model")
    ax.plot(time, theta, color="#d97706", lw=1.5, label="Nonlinear sim, no Coulomb")
    ax.plot(time, theta_friction, color="#0f766e", lw=1.3, ls=":", label="Nonlinear sim, Coulomb")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pan angle (rad)")
    ax.set_title("Pan step response")
    ax.legend(frameon=False, loc="lower right")
    fig.tight_layout()
    fig.savefig(args.out_dir / "step_response.png", dpi=140)
    plt.close(fig)

    omega = np.logspace(-1, 3.5, 600)
    fig, axes = plt.subplots(2, 2, figsize=(9.2, 6.2), sharex=True)
    bode_pair(axes[:, 0], inner, omega, "Inner rate loop", inner_m)
    bode_pair(axes[:, 1], outer, omega, "Outer position loop", outer_m)
    fig.suptitle("Loop transfer Bode plots (unsaturated pan axis)", y=1.01)
    fig.tight_layout()
    fig.savefig(args.out_dir / "bode.png", dpi=140, bbox_inches="tight")
    plt.close(fig)

    print(json.dumps(metrics, indent=2))
    if rms > 0.03:
        raise SystemExit(f"Linear model and C++ step response differ by {rms:.4f} rad RMS")


if __name__ == "__main__":
    main()
