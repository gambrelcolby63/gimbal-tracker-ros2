#!/usr/bin/env python3
"""Side-by-side portrait of a constant-velocity filter and the IMM on one maneuver.

Left panel: the filter that assumes straight flight. Right panel: the mixture
of straight, accelerating, and turning models. The strip underneath is that
run's model probabilities. Playback is a little slower than the simulation.
This is a picture of the simulator, not a weapon.
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
from matplotlib.patches import Rectangle

WIDTH_PX = 720
HEIGHT_PX = 1280
DPI = 100
VIDEO_SECONDS = 24.0
FPS = 12
GIF_WIDTH = 480
GIF_FPS = 8
MAX_BYTES = 10 * 1024 * 1024
# Local window, meters. Wide enough to see a turn being cut, tight enough
# that a few meters of lag stay obvious on a phone.
WINDOW_X_M = 56.0

INK = "#1e293b"
MUTED = "#475569"
PAPER = "#f4f1ea"
FIELD = "#f8fafc"
GRID = "#e2e8f0"
TRUTH = "#1d4ed8"
TRUTH_FAINT = "#93c5fd"
SENSOR = "#ea580c"
CV = "#be123c"
IMM = "#0f766e"
CV_PROB = "#64748b"
CA_PROB = "#ea580c"
CT_PROB = "#0f766e"

# Short label is painted on the phase bar. Long label is the headline.
PHASES = (
    (0.0, 4.0, "Straight", "Straight", "#cbd5e1"),
    (4.0, 9.0, "Turn", "Coordinated turn", "#93c5fd"),
    (9.0, 12.0, "Straight", "Straight", "#cbd5e1"),
    (12.0, 14.0, "Speed", "Speed change", "#fdba74"),
    (14.0, 20.0, "Weave", "Weave", "#c4b5fd"),
)


def require_ffmpeg() -> None:
    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        raise SystemExit("ffmpeg and ffprobe are required to encode the demo.")


def load_trace(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    required = (
        "t",
        "truth_x",
        "truth_y",
        "meas_x",
        "meas_y",
        "cv_x",
        "cv_y",
        "imm_x",
        "imm_y",
        "mu_cv",
        "mu_ca",
        "mu_ct",
    )
    names = data.dtype.names or ()
    missing = [name for name in required if name not in names]
    if missing:
        raise SystemExit(f"{path} is missing {', '.join(missing)}.")
    if data.shape == () or len(data) < 2:
        raise SystemExit(f"{path} does not contain a maneuver.")
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


def phase_at(time_s: float) -> str:
    for start, end, _short, label, _color in PHASES:
        if start <= time_s < end:
            return label
    return PHASES[-1][3]


class ManeuverScene:
    def __init__(self, data: np.ndarray):
        self.t = np.asarray(data["t"], dtype=float)
        self.truth = np.column_stack((data["truth_x"], data["truth_y"]))
        self.meas = np.column_stack((data["meas_x"], data["meas_y"]))
        self.cv = np.column_stack((data["cv_x"], data["cv_y"]))
        self.imm = np.column_stack((data["imm_x"], data["imm_y"]))
        self.mu = np.column_stack((data["mu_cv"], data["mu_ca"], data["mu_ct"]))
        self.sim_end = float(self.t[-1])
        self.slow = VIDEO_SECONDS / self.sim_end
        velocity = np.gradient(self.truth, self.t, axis=0)
        speed = np.linalg.norm(velocity, axis=1)
        speed = np.maximum(speed, 1e-6)
        self.heading = velocity / speed[:, None]

        fig_w = WIDTH_PX / DPI
        fig_h = HEIGHT_PX / DPI
        self.fig = plt.figure(figsize=(fig_w, fig_h), dpi=DPI)
        self.fig.patch.set_facecolor(PAPER)
        # Square maps so a few meters of lag fill the panel. The strip gets the rest.
        map_w = 0.42
        map_h = (map_w * fig_w) / fig_h
        self.left = self.fig.add_axes([0.06, 0.575, map_w, map_h])
        self.right = self.fig.add_axes([0.54, 0.575, map_w, map_h])
        self.phase_ax = self.fig.add_axes([0.14, 0.508, 0.74, 0.034])
        self.prob = self.fig.add_axes([0.14, 0.105, 0.74, 0.355])
        self.window_y = WINDOW_X_M * (map_h * fig_h) / (map_w * fig_w)
        self._build_static()
        self._build_dynamic()

    def _style_map(self, ax) -> None:
        ax.set_facecolor(FIELD)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(colors=MUTED, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#cbd5e1")
        ax.set_xlabel("x (m)", color=MUTED, fontsize=9)
        ax.set_ylabel("y (m)", color=MUTED, fontsize=9)

    def _build_static(self) -> None:
        self.fig.text(
            0.06,
            0.978,
            "Maneuvering target",
            fontsize=20,
            fontweight="bold",
            color=INK,
            va="top",
        )
        self.fig.text(
            0.06,
            0.952,
            f"Same measurements, played at 1/{self.slow:.1f} speed. Simulation only.",
            fontsize=10,
            color=MUTED,
            va="top",
        )
        self.phase_badge = self.fig.text(
            0.06,
            0.928,
            "",
            fontsize=12,
            fontweight="bold",
            color=INK,
            va="top",
        )

        for ax, title, color in (
            (self.left, "Constant velocity", CV),
            (self.right, "Interacting models", IMM),
        ):
            self._style_map(ax)
            ax.set_title(title, loc="left", color=color, fontsize=12, pad=6, fontweight="bold")
            ax.plot(
                self.truth[:, 0],
                self.truth[:, 1],
                color=TRUTH_FAINT,
                lw=1.6,
                zorder=1,
                solid_capstyle="round",
            )

        self.phase_ax.set_xlim(0.0, self.sim_end)
        self.phase_ax.set_ylim(0.0, 1.0)
        self.phase_ax.set_yticks([])
        self.phase_ax.tick_params(colors=MUTED, labelsize=7, length=0)
        self.phase_ax.set_xticks([])
        for spine in self.phase_ax.spines.values():
            spine.set_color("#e7e5e4")
        for start, end, short, _label, color in PHASES:
            self.phase_ax.add_patch(
                Rectangle((start, 0.0), end - start, 1.0, facecolor=color, edgecolor="white", lw=0.6)
            )
            self.phase_ax.text(
                0.5 * (start + end),
                0.5,
                short,
                ha="center",
                va="center",
                fontsize=8,
                color=INK,
            )
        self.right.set_ylabel("")
        bar = 20.0 / WINDOW_X_M
        self.left.plot(
            [0.97 - bar, 0.97],
            [0.90, 0.90],
            transform=self.left.transAxes,
            color=INK,
            lw=2.0,
            solid_capstyle="butt",
            zorder=7,
            clip_on=False,
        )
        self.left.text(
            0.97 - 0.5 * bar,
            0.92,
            "20 m",
            transform=self.left.transAxes,
            ha="center",
            va="bottom",
            fontsize=8,
            color=INK,
            zorder=7,
        )

        self._style_prob()
        legend_handles = [
            Line2D([0], [0], color=TRUTH, lw=2.4, label="True path"),
            Line2D([0], [0], color=SENSOR, marker="o", lw=0, markersize=6, label="Sensor"),
            Line2D([0], [0], color=CV, lw=2.0, ls="--", label="CV estimate"),
            Line2D([0], [0], color=IMM, lw=2.0, ls="--", label="IMM estimate"),
        ]
        legend = self.fig.legend(
            handles=legend_handles,
            loc="lower center",
            bbox_to_anchor=(0.50, 0.868),
            frameon=False,
            fontsize=8.5,
            borderpad=0.2,
            labelcolor=INK,
            ncol=4,
            columnspacing=1.1,
            handletextpad=0.4,
        )
        legend.get_frame().set_facecolor(PAPER)

    def _style_prob(self) -> None:
        ax = self.prob
        ax.set_facecolor(FIELD)
        ax.set_xlim(0.0, self.sim_end)
        ax.set_ylim(0.0, 1.0)
        ax.set_xlabel("Time (s)", color=MUTED, fontsize=9)
        ax.set_ylabel("Model probability", color=MUTED, fontsize=9)
        ax.set_title("Which model the IMM trusts", loc="left", color=INK, fontsize=12, pad=6)
        handles = [
            Line2D([0], [0], color=CV_PROB, lw=8, label="Constant velocity"),
            Line2D([0], [0], color=CA_PROB, lw=8, label="Acceleration"),
            Line2D([0], [0], color=CT_PROB, lw=8, label="Coordinated turn"),
        ]
        legend = ax.legend(
            handles=handles,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.20),
            frameon=False,
            fontsize=8.5,
            ncol=3,
            labelcolor=INK,
            borderaxespad=0.0,
            handlelength=1.3,
            columnspacing=1.1,
        )
        legend.set_zorder(5)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.grid(True, color=GRID, lw=0.5)
        for spine in ax.spines.values():
            spine.set_color("#cbd5e1")

    def _build_dynamic(self) -> None:
        def tracks(ax, color):
            past, = ax.plot([], [], color=TRUTH, lw=2.6, zorder=3, solid_capstyle="round")
            est, = ax.plot([], [], color=color, lw=2.2, ls="--", zorder=3)
            dots = ax.scatter([], [], s=18, c=SENSOR, linewidths=0.3, edgecolors="white", zorder=4)
            truth_now, = ax.plot([], [], marker="o", color=TRUTH, ms=8, zorder=6, markeredgecolor="white", markeredgewidth=0.6)
            est_now, = ax.plot([], [], marker="s", color=color, ms=7, zorder=6, markeredgecolor="white", markeredgewidth=0.5)
            miss = ax.annotate(
                "",
                xy=(0.0, 0.0),
                xytext=(0.0, 0.0),
                arrowprops={"arrowstyle": "-", "color": color, "lw": 1.0, "ls": (0, (2, 2))},
                zorder=5,
            )
            readout = ax.text(
                0.04,
                0.04,
                "",
                transform=ax.transAxes,
                ha="left",
                va="bottom",
                fontsize=9,
                color=INK,
                bbox={"boxstyle": "round,pad=0.25", "facecolor": "#fffdf8", "edgecolor": "#e7e5e4"},
                zorder=7,
            )
            return past, est, dots, truth_now, est_now, miss, readout

        (
            self.cv_truth,
            self.cv_est,
            self.cv_dots,
            self.cv_truth_now,
            self.cv_est_now,
            self.cv_miss,
            self.cv_readout,
        ) = tracks(self.left, CV)
        (
            self.imm_truth,
            self.imm_est,
            self.imm_dots,
            self.imm_truth_now,
            self.imm_est_now,
            self.imm_miss,
            self.imm_readout,
        ) = tracks(self.right, IMM)
        self.cursor = self.prob.axvline(0.0, color=INK, lw=1.0, zorder=4)
        self.stack_artists: list = []

    def _window(self, index: int) -> tuple[float, float, float, float]:
        truth = self.truth[index]
        direction = self.heading[index]
        center = truth - 0.18 * WINDOW_X_M * direction
        return (
            float(center[0] - 0.5 * WINDOW_X_M),
            float(center[0] + 0.5 * WINDOW_X_M),
            float(center[1] - 0.5 * self.window_y),
            float(center[1] + 0.5 * self.window_y),
        )

    def _follow(self, ax, index: int, past, est, dots, truth_now, est_now, miss, readout, estimate) -> None:
        sl = slice(0, index + 1)
        past.set_data(self.truth[sl, 0], self.truth[sl, 1])
        est.set_data(estimate[sl, 0], estimate[sl, 1])
        start = max(0, index - 10)
        cloud = self.meas[start : index + 1]
        dots.set_offsets(cloud)
        truth = self.truth[index]
        guess = estimate[index]
        truth_now.set_data([truth[0]], [truth[1]])
        est_now.set_data([guess[0]], [guess[1]])
        miss.xy = truth
        miss.set_position(guess)
        error = float(np.hypot(guess[0] - truth[0], guess[1] - truth[1]))
        readout.set_text(f"{error:.1f} m off")
        x0, x1, y0, y1 = self._window(index)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)

    def _draw_stack(self, index: int) -> None:
        for artist in self.stack_artists:
            artist.remove()
        self.stack_artists.clear()
        sl = slice(0, index + 1)
        series = self.mu[sl]
        if len(self.t[sl]) == 1:
            times = np.array([self.t[0], self.t[0] + 1e-3])
            series = np.vstack((series, series))
        else:
            times = self.t[sl]
        baseline = np.zeros(len(times))
        for column, color in enumerate((CV_PROB, CA_PROB, CT_PROB)):
            top = baseline + series[:, column]
            fill = self.prob.fill_between(times, baseline, top, color=color, alpha=0.92, lw=0, zorder=2)
            self.stack_artists.append(fill)
            baseline = top
        self.cursor.set_xdata([self.t[index], self.t[index]])

    def draw(self, index: int) -> None:
        index = int(np.clip(index, 0, len(self.t) - 1))
        label = phase_at(float(self.t[index]))
        self.phase_badge.set_text(f"{label}   ·   t = {self.t[index]:.1f} s")
        self._follow(
            self.left,
            index,
            self.cv_truth,
            self.cv_est,
            self.cv_dots,
            self.cv_truth_now,
            self.cv_est_now,
            self.cv_miss,
            self.cv_readout,
            self.cv,
        )
        self._follow(
            self.right,
            index,
            self.imm_truth,
            self.imm_est,
            self.imm_dots,
            self.imm_truth_now,
            self.imm_est_now,
            self.imm_miss,
            self.imm_readout,
            self.imm,
        )
        self._draw_stack(index)

    def save(self, path: Path) -> None:
        self.fig.savefig(path, dpi=DPI, facecolor=self.fig.get_facecolor())


def frame_indices(count: int, nframes: int) -> np.ndarray:
    if nframes == 1:
        return np.array([count - 1])
    return np.linspace(0, count - 1, nframes).round().astype(int)


def render(data: np.ndarray, out_dir: Path, preview: list[float] | None) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    scene = ManeuverScene(data)
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
    with tempfile.TemporaryDirectory(prefix="imm-frames-") as tmp:
        frame_dir = Path(tmp)
        for frame, index in enumerate(indices):
            scene.draw(int(index))
            scene.save(frame_dir / f"frame_{frame:04d}.png")
            if frame % FPS == 0:
                print(f"frame {frame + 1}/{nframes}", flush=True)
        mp4 = out_dir / "maneuver.mp4"
        gif = out_dir / "maneuver.gif"
        encode_mp4(frame_dir, mp4, FPS)
        encode_gif(mp4, gif, GIF_WIDTH, GIF_FPS)
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
    parser.add_argument("--preview", type=float, nargs="*", default=None)
    args = parser.parse_args()
    if not args.preview:
        require_ffmpeg()
    render(load_trace(args.csv), args.out_dir, args.preview)


if __name__ == "__main__":
    main()
