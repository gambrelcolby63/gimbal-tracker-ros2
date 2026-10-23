#!/usr/bin/env python3
"""Render the shipped flyby as a phone-friendly MP4 and GIF.

The pictures are a top-down view of the C++ tracking_demo log: turret, gimbal
barrel, true path, sensor dots, Kalman estimate, and lead intercept, plus pan
and tilt aim error. Playback is slower than the simulation so the motion is
easy to follow. This is a visualization of the simulator, not a weapon.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

# Portrait 9:16. Even pixel counts keep yuv420p happy.
WIDTH_PX = 720
HEIGHT_PX = 1280
DPI = 100
VIDEO_SECONDS = 24.0
FPS = 12
BARREL_M = 18.0
MAX_BYTES = 10 * 1024 * 1024

INK = "#1e293b"
MUTED = "#475569"
PAPER = "#f4f1ea"
FIELD = "#f8fafc"
GRID = "#e2e8f0"
TRUTH = "#1d4ed8"
TRUTH_FAINT = "#93c5fd"
SENSOR = "#ea580c"
ESTIMATE = "#7c3aed"
AIM = "#be123c"
BARREL = "#0f766e"
TURRET = "#1e293b"


def require_ffmpeg() -> None:
    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        raise SystemExit("ffmpeg and ffprobe are required to encode the demo.")


def load_flyby(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True)
    required = (
        "t",
        "gimbal_az",
        "truth_x",
        "truth_y",
        "meas_x",
        "meas_y",
        "est_x",
        "est_y",
        "aim_x",
        "aim_y",
        "pan_err",
        "tilt_err",
    )
    missing = [name for name in required if name not in data.dtype.names]
    if missing:
        raise SystemExit(
            f"{path} is missing {', '.join(missing)}. Rebuild tracking_demo and rerun it."
        )
    if data.shape == () or len(data) < 2:
        raise SystemExit(f"{path} does not contain a flyby.")
    return data


def encode_mp4(frame_dir: Path, destination: Path, fps: int) -> None:
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-framerate",
            str(fps),
            "-i",
            str(frame_dir / "frame_%04d.png"),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "20",
            "-movflags",
            "+faststart",
            str(destination),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )


def encode_gif(mp4: Path, destination: Path, width: int, fps: int) -> None:
    palette = destination.with_suffix(".palette.png")
    scale = f"fps={fps},scale={width}:-2:flags=lanczos"
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(mp4),
            "-vf",
            f"{scale},palettegen=stats_mode=diff",
            str(palette),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(mp4),
            "-i",
            str(palette),
            "-lavfi",
            f"{scale}[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle",
            "-loop",
            "0",
            str(destination),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    palette.unlink(missing_ok=True)


def media_duration(path: Path) -> float:
    probe = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-show_entries",
            "format=duration",
            "-of",
            "default=noprint_wrappers=1:nokey=1",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return float(probe.stdout.strip())


class FlybyScene:
    def __init__(self, data: np.ndarray):
        self.data = data
        self.t = np.asarray(data["t"], dtype=float)
        self.truth = np.column_stack((data["truth_x"], data["truth_y"]))
        self.meas = np.column_stack((data["meas_x"], data["meas_y"]))
        self.est = np.column_stack((data["est_x"], data["est_y"]))
        self.aim = np.column_stack((data["aim_x"], data["aim_y"]))
        self.pan = np.asarray(data["gimbal_az"], dtype=float)
        self.pan_deg = np.rad2deg(np.asarray(data["pan_err"], dtype=float))
        self.tilt_deg = np.rad2deg(np.asarray(data["tilt_err"], dtype=float))
        self.sim_end = float(self.t[-1])
        self.slow = VIDEO_SECONDS / self.sim_end

        span_x = 86.0
        span_y = 120.0
        self.xlim = (-18.0, -18.0 + span_x)
        self.ylim = (-52.0, -52.0 + span_y)

        fig_w = WIDTH_PX / DPI
        fig_h = HEIGHT_PX / DPI
        self.fig = plt.figure(figsize=(fig_w, fig_h), dpi=DPI)
        self.fig.patch.set_facecolor(PAPER)
        # Positions are figure fractions chosen so the map keeps equal scale
        # and the close-up stays square, with a gap under the subtitle.
        self.ax = self.fig.add_axes([0.16, 0.335, 0.74, 0.555])
        self.zoom = self.fig.add_axes([0.07, 0.038, 0.40, 0.225])
        self.pan_ax = self.fig.add_axes([0.60, 0.158, 0.35, 0.105])
        self.tilt_ax = self.fig.add_axes([0.60, 0.038, 0.35, 0.105])
        self._build_static()
        self._build_dynamic()

    def _style_map(self, ax, title: str | None) -> None:
        ax.set_facecolor(FIELD)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(colors=MUTED, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#cbd5e1")
        if title:
            ax.set_title(title, loc="left", color=INK, fontsize=11, pad=6)

    def _build_static(self) -> None:
        self.fig.text(
            0.08,
            0.978,
            "Simulated turret tracker",
            fontsize=20,
            fontweight="bold",
            color=INK,
            va="top",
        )
        self.fig.text(
            0.08,
            0.950,
            f"Crossing flyby, played at 1/{self.slow:.0f} speed. Simulation only.",
            fontsize=10.5,
            color=MUTED,
            va="top",
        )
        self.fig.text(
            0.08,
            0.928,
            "Barrel: where the gimbal points. Star: where a constant-speed shot meets the target.",
            fontsize=9,
            color=MUTED,
            va="top",
        )

        ax = self.ax
        self._style_map(ax, None)
        ax.set_xlim(*self.xlim)
        ax.set_ylim(*self.ylim)
        ax.set_xlabel("x (m)", color=MUTED, fontsize=10)
        ax.set_ylabel("y (m)", color=MUTED, fontsize=10)
        ax.text(
            0.0,
            1.01,
            "Top-down view",
            transform=ax.transAxes,
            ha="left",
            va="bottom",
            color=INK,
            fontsize=12,
        )
        for radius in (25.0, 50.0):
            ax.add_patch(
                Circle(
                    (0.0, 0.0),
                    radius,
                    fill=False,
                    linestyle=(0, (2, 3)),
                    linewidth=0.8,
                    edgecolor="#94a3b8",
                    zorder=0,
                )
            )
        ax.plot(self.truth[:, 0], self.truth[:, 1], color=TRUTH_FAINT, lw=2.0, zorder=1, solid_capstyle="round")
        ax.add_patch(Circle((0.0, 0.0), 1.7, facecolor=TURRET, edgecolor="white", lw=0.8, zorder=6))
        ax.text(-1.0, -5.2, "Turret", color=INK, fontsize=9, ha="center")
        scale_x, scale_y = 8.0, -46.5
        ax.plot([scale_x, scale_x + 20.0], [scale_y, scale_y], color=INK, lw=2.0, solid_capstyle="butt", zorder=2)
        ax.plot([scale_x, scale_x], [scale_y - 1.2, scale_y + 1.2], color=INK, lw=1.5, zorder=2)
        ax.plot([scale_x + 20.0, scale_x + 20.0], [scale_y - 1.2, scale_y + 1.2], color=INK, lw=1.5, zorder=2)
        ax.text(scale_x + 10.0, scale_y + 1.8, "20 m", color=INK, fontsize=8, ha="center")

        legend_handles = [
            Line2D([0], [0], color=TRUTH, lw=2.4, label="True path"),
            Line2D([0], [0], color=SENSOR, marker="o", lw=0, markersize=6, label="Sensor dots"),
            Line2D([0], [0], color=ESTIMATE, lw=2.0, ls="--", label="Filter estimate"),
            Line2D([0], [0], color=AIM, marker="*", lw=0, markersize=12, label="Aim point"),
            Line2D([0], [0], color=BARREL, lw=3.2, label="Gimbal barrel"),
        ]
        legend = ax.legend(
            handles=legend_handles,
            loc="upper left",
            frameon=True,
            fontsize=9,
            borderpad=0.6,
            labelcolor=INK,
        )
        legend.get_frame().set_facecolor("#fffdf8")
        legend.get_frame().set_edgecolor("#e7e5e4")

        self.zoom_box = Rectangle(
            (0.0, 0.0),
            1.0,
            1.0,
            fill=False,
            edgecolor="#64748b",
            linewidth=1.0,
            linestyle=(0, (3, 2)),
            zorder=7,
        )
        ax.add_patch(self.zoom_box)

        zoom = self.zoom
        self._style_map(zoom, None)
        zoom.set_xlabel("x (m)", color=MUTED, fontsize=8)
        zoom.set_ylabel("y (m)", color=MUTED, fontsize=8)
        zoom.text(
            0.04,
            0.96,
            "Near the target",
            transform=zoom.transAxes,
            ha="left",
            va="top",
            color=INK,
            fontsize=10,
            bbox={"boxstyle": "round,pad=0.15", "facecolor": "#fffdf8", "edgecolor": "none", "alpha": 0.9},
        )

        self._style_error(self.pan_ax, "Pan (°)", self.pan_deg)
        self._style_error(self.tilt_ax, "Tilt (°)", self.tilt_deg)
        self.pan_ax.set_title("Aim error", loc="left", color=INK, fontsize=10, pad=3)
        self.tilt_ax.set_xlabel("Time (s)", color=MUTED, fontsize=8)

    def _style_error(self, ax, ylabel: str, series: np.ndarray) -> None:
        ax.set_facecolor(FIELD)
        ax.set_xlim(0.0, self.sim_end)
        pad = max(0.15, 0.12 * float(np.nanmax(np.abs(series))))
        peak = float(np.nanmax(np.abs(series)))
        ax.set_ylim(-(peak + pad), peak + pad)
        ax.axhline(0.0, color="#cbd5e1", lw=0.8)
        ax.grid(True, color=GRID, lw=0.5)
        ax.tick_params(colors=MUTED, labelsize=7.5)
        ax.set_ylabel(ylabel, color=MUTED, fontsize=8)
        for spine in ax.spines.values():
            spine.set_color("#cbd5e1")

    def _build_dynamic(self) -> None:
        ax = self.ax
        (self.truth_past,) = ax.plot([], [], color=TRUTH, lw=2.6, zorder=3, solid_capstyle="round")
        self.sensor_dots = ax.scatter(
            [], [], s=22, c=SENSOR, linewidths=0.4, edgecolors="white", zorder=4
        )
        (self.est_past,) = ax.plot([], [], color=ESTIMATE, lw=1.8, ls="--", zorder=3)
        (self.aim_ray,) = ax.plot([], [], color=AIM, lw=1.2, ls=(0, (1.5, 1.8)), zorder=3)
        (self.barrel,) = ax.plot([], [], color=BARREL, lw=4.5, solid_capstyle="round", zorder=5)
        (self.truth_now,) = ax.plot([], [], marker="o", color=TRUTH, ms=8, zorder=8, markeredgecolor="white", markeredgewidth=0.6)
        (self.est_now,) = ax.plot([], [], marker="s", color=ESTIMATE, ms=6, zorder=8, markeredgecolor="white", markeredgewidth=0.5)
        (self.aim_now,) = ax.plot([], [], marker="*", color=AIM, ms=14, zorder=9, markeredgecolor="white", markeredgewidth=0.4)
        self.motion = ax.annotate(
            "",
            xy=(0.0, 0.0),
            xytext=(0.0, 0.0),
            arrowprops={"arrowstyle": "-|>", "color": TRUTH, "lw": 1.4, "mutation_scale": 10},
            zorder=8,
        )
        self.clock = ax.text(
            0.98,
            0.03,
            "",
            transform=ax.transAxes,
            ha="right",
            va="bottom",
            fontsize=10,
            color=INK,
            bbox={"boxstyle": "round,pad=0.25", "facecolor": "#fffdf8", "edgecolor": "#e7e5e4"},
        )

        zoom = self.zoom
        (self.z_truth,) = zoom.plot([], [], color=TRUTH, lw=2.2, zorder=3)
        self.z_sensor = zoom.scatter(
            [], [], s=36, c=SENSOR, linewidths=0.4, edgecolors="white", zorder=4
        )
        (self.z_est,) = zoom.plot([], [], color=ESTIMATE, lw=1.6, ls="--", zorder=3)
        (self.z_aim_ray,) = zoom.plot([], [], color=AIM, lw=1.0, ls=(0, (1.5, 1.8)), zorder=3)
        (self.z_truth_now,) = zoom.plot([], [], marker="o", color=TRUTH, ms=8, zorder=6, markeredgecolor="white")
        (self.z_est_now,) = zoom.plot([], [], marker="s", color=ESTIMATE, ms=7, zorder=6, markeredgecolor="white")
        (self.z_aim_now,) = zoom.plot([], [], marker="*", color=AIM, ms=13, zorder=7, markeredgecolor="white")

        (self.pan_line,) = self.pan_ax.plot([], [], color=BARREL, lw=1.8)
        (self.tilt_line,) = self.tilt_ax.plot([], [], color="#b45309", lw=1.8)
        self.pan_cursor = self.pan_ax.axvline(0.0, color="#94a3b8", lw=0.8)
        self.tilt_cursor = self.tilt_ax.axvline(0.0, color="#94a3b8", lw=0.8)

    def _cluster(self, index: int, count: int) -> np.ndarray:
        start = max(0, index - count + 1)
        return self.meas[start : index + 1]

    def _window(self, index: int) -> tuple[np.ndarray, float]:
        truth = self.truth[index]
        aim = self.aim[index]
        if np.isfinite(aim).all():
            center = 0.5 * (truth + aim)
            half = max(4.2, 0.5 * float(np.linalg.norm(aim - truth)) + 1.8)
        else:
            center = truth
            half = 4.2
        return center, half

    def draw(self, index: int) -> None:
        index = int(np.clip(index, 0, len(self.t) - 1))
        sl = slice(0, index + 1)
        self.truth_past.set_data(self.truth[sl, 0], self.truth[sl, 1])
        self.est_past.set_data(self.est[sl, 0], self.est[sl, 1])
        recent = self._cluster(index, 16)
        self.sensor_dots.set_offsets(recent[::2] if len(recent) else recent)
        aim = self.aim[index]
        if np.isfinite(aim).all():
            self.aim_ray.set_data([BARREL_M * np.cos(self.pan[index]), aim[0]], [BARREL_M * np.sin(self.pan[index]), aim[1]])
            self.aim_now.set_data([aim[0]], [aim[1]])
        else:
            self.aim_ray.set_data([], [])
            self.aim_now.set_data([], [])
        pan = self.pan[index]
        self.barrel.set_data([0.0, BARREL_M * np.cos(pan)], [0.0, BARREL_M * np.sin(pan)])
        truth = self.truth[index]
        est = self.est[index]
        self.truth_now.set_data([truth[0]], [truth[1]])
        self.est_now.set_data([est[0]], [est[1]])
        motion = np.array([0.0, 8.0])
        self.motion.xy = truth + motion
        self.motion.set_position(truth)
        self.clock.set_text(f"t = {self.t[index]:.1f} s")

        center, half = self._window(index)
        self.zoom_box.set_xy((center[0] - half, center[1] - half))
        self.zoom_box.set_width(2.0 * half)
        self.zoom_box.set_height(2.0 * half)
        self.zoom.set_xlim(center[0] - half, center[0] + half)
        self.zoom.set_ylim(center[1] - half, center[1] + half)
        near = slice(max(0, index - 40), index + 1)
        self.z_truth.set_data(self.truth[near, 0], self.truth[near, 1])
        self.z_est.set_data(self.est[near, 0], self.est[near, 1])
        cloud = self._cluster(index, 12)
        self.z_sensor.set_offsets(cloud)
        if np.isfinite(aim).all():
            self.z_aim_ray.set_data([0.0, aim[0]], [0.0, aim[1]])
            self.z_aim_now.set_data([aim[0]], [aim[1]])
        else:
            self.z_aim_ray.set_data([], [])
            self.z_aim_now.set_data([], [])
        self.z_truth_now.set_data([truth[0]], [truth[1]])
        self.z_est_now.set_data([est[0]], [est[1]])

        self.pan_line.set_data(self.t[sl], self.pan_deg[sl])
        self.tilt_line.set_data(self.t[sl], self.tilt_deg[sl])
        self.pan_cursor.set_xdata([self.t[index], self.t[index]])
        self.tilt_cursor.set_xdata([self.t[index], self.t[index]])

    def save(self, path: Path) -> None:
        self.fig.savefig(path, dpi=DPI, facecolor=self.fig.get_facecolor())


def frame_indices(count: int, nframes: int) -> np.ndarray:
    if nframes == 1:
        return np.array([count - 1])
    return np.linspace(0, count - 1, nframes).round().astype(int)


def render(data: np.ndarray, out_dir: Path, preview: list[float] | None) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    scene = FlybyScene(data)
    if preview:
        for stamp in preview:
            index = int(np.argmin(np.abs(scene.t - stamp)))
            scene.draw(index)
            path = out_dir / f"preview_{stamp:.1f}s.png"
            scene.save(path)
            print(f"Wrote {path}")
        return

    nframes = int(round(VIDEO_SECONDS * FPS))
    indices = frame_indices(len(scene.t), nframes)
    with tempfile.TemporaryDirectory(prefix="flyby-frames-") as tmp:
        frame_dir = Path(tmp)
        for frame, index in enumerate(indices):
            scene.draw(int(index))
            scene.save(frame_dir / f"frame_{frame:04d}.png")
            if frame % FPS == 0:
                print(f"frame {frame + 1}/{nframes}")
        mp4 = out_dir / "flyby.mp4"
        gif = out_dir / "flyby.gif"
        encode_mp4(frame_dir, mp4, FPS)
        encode_gif(mp4, gif, WIDTH_PX, FPS)
        for path in (mp4, gif):
            size = path.stat().st_size
            duration = media_duration(path)
            print(f"{path.name}: {size / 1e6:.2f} MB, {duration:.1f} s")
            if size > MAX_BYTES:
                raise SystemExit(f"{path} is {size / 1e6:.1f} MB, over the 10 MB budget.")
            if not 20.0 <= duration <= 30.0:
                raise SystemExit(f"{path} is {duration:.1f} s, outside 20–30 s.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument(
        "--preview",
        type=float,
        nargs="*",
        default=None,
        help="Write stills at these simulation times and skip the video.",
    )
    args = parser.parse_args()
    if not args.preview:
        require_ffmpeg()
    render(load_flyby(args.csv), args.out_dir, args.preview)


if __name__ == "__main__":
    main()
