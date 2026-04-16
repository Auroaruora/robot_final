"""
simulate.py  —  overview + animated GIF of both robots' yarn-wrapping paths
============================================================================
Imports all logic from path_planner.py.

Usage:   python simulate.py
Output:  outputs/path_overview.png
         outputs/path_simulation.gif
"""

import os, math, tempfile, shutil
from legacy_version.path_planner import (
    POLES, POLE_R, ROBOT_R, SAFETY, ARC_R, ROBOTS,
    dist, build_path, path_distances, pos_at_dist,
)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import imageio.v2 as imageio

OUT_DIR = "./outputs"
COLA, COLB, CPOLE = "#2eaa57", "#2b7fd4", "#d85a30"


def draw_field(ax):
    ax.set_aspect("equal")
    ax.set_facecolor("#fafaf6")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_xlabel("x  (cm)")
    ax.set_ylabel("y  (cm)")

    for pid, (px, py) in POLES.items():
        ax.add_patch(plt.Circle((px, py), ARC_R,
                     color=CPOLE, alpha=0.08, lw=0))
        ax.add_patch(plt.Circle((px, py), ARC_R, fill=False,
                     ec=CPOLE, lw=0.7, ls="--", alpha=0.4))
        ax.add_patch(plt.Circle((px, py), POLE_R,
                     color=CPOLE, zorder=5))
        ax.text(px, py, str(pid), ha="center", va="center",
                fontsize=9, fontweight="bold", color="white", zorder=6)

    for rid, cfg in ROBOTS.items():
        sx, sy = cfg["start"]
        col = COLA if rid == "A" else COLB
        ax.plot(sx, sy, "x", color=col, markersize=10,
                markeredgewidth=2.5, zorder=7)
        ax.annotate(f"Robot {rid} start ({sx},{sy})",
                    (sx, sy), textcoords="offset points",
                    xytext=(8, -12 if rid == "A" else 8),
                    fontsize=7.5, color=col)


def draw_path(ax, pts, color, label):
    xs = [p["x"] for p in pts]
    ys = [p["y"] for p in pts]
    # thin line for wraps
    ax.plot(xs, ys, color=color, linewidth=1.0, alpha=0.45,
            label=label, zorder=3)
    # thick lines for straight transits
    for i in range(1, len(pts)):
        if pts[i]["type"] == "move":
            ax.plot([pts[i-1]["x"], pts[i]["x"]],
                    [pts[i-1]["y"], pts[i]["y"]],
                    color=color, linewidth=2.4, alpha=0.85, zorder=4)

    # number the sequence on entry to each pole
    seen = set()
    order = 1
    for p in pts:
        if p["type"] == "move" and p["pole"] not in seen and p["pole"] != 0:
            seen.add(p["pole"])
            ax.annotate(str(order), (p["x"], p["y"]),
                        fontsize=7, fontweight="bold", color=color,
                        textcoords="offset points", xytext=(6, 6),
                        zorder=11,
                        bbox=dict(boxstyle="round,pad=0.15",
                                  fc="white", ec=color, lw=0.5, alpha=0.85))
            order += 1


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    ptsA = build_path("A")
    ptsB = build_path("B")
    cumA, totA = path_distances(ptsA)
    cumB, totB = path_distances(ptsB)

    print(f"Robot A: {len(ptsA)} pts, {totA:.0f} cm, seq={ROBOTS['A']['seq']}, "
          f"dir={ROBOTS['A']['dir']}")
    print(f"Robot B: {len(ptsB)} pts, {totB:.0f} cm, seq={ROBOTS['B']['seq']}, "
          f"dir={ROBOTS['B']['dir']}")

    fig, ax = plt.subplots(figsize=(14, 10))
    fig.patch.set_facecolor("#fafaf6")
    draw_field(ax)
    draw_path(ax, ptsA, COLA, f"Robot A  ({ROBOTS['A']['dir'].upper()})")
    draw_path(ax, ptsB, COLB, f"Robot B  ({ROBOTS['B']['dir'].upper()})")
    ax.legend(loc="upper left", fontsize=10)
    ax.set_title("Yarn-wrapping path planner — tangent-optimised overview",
                 fontweight="bold", fontsize=13)

    # auto-fit view
    all_x = [p["x"] for p in ptsA + ptsB]
    all_y = [p["y"] for p in ptsA + ptsB]
    margin = 60
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    path = os.path.join(OUT_DIR, "path_overview.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved  {path}")

    # ==============================================================
    # ANIMATED GIF
    # ==============================================================
    print("Generating animation …")

    # auto-fit bounds for animation frames
    view_xmin, view_xmax = min(all_x) - margin, max(all_x) + margin
    view_ymin, view_ymax = min(all_y) - margin, max(all_y) + margin

    CM_PER_FRAME = 4.0
    B_DELAY      = 0
    max_d  = max(totA, totB + B_DELAY)
    n_frames = min(int(max_d / CM_PER_FRAME) + 1, 600)
    step   = max_d / n_frames

    tmpdir = tempfile.mkdtemp()
    frames = []

    for fi in range(n_frames + 1):
        dA = min(fi * step, totA)
        dB = max(0.0, min(fi * step - B_DELAY, totB))

        fig2, ax2 = plt.subplots(figsize=(10, 7.5))
        fig2.patch.set_facecolor("#fafaf6")
        draw_field(ax2)
        ax2.set_xlim(view_xmin, view_xmax)
        ax2.set_ylim(view_ymin, view_ymax)
        ax2.set_title(f"Simulation   frame {fi}/{n_frames}", fontsize=11)

        # draw trails
        for (pts_i, cum_i, d_now, col) in [
            (ptsA, cumA, dA, COLA),
            (ptsB, cumB, dB, COLB),
        ]:
            xs, ys = [], []
            for j in range(len(pts_i)):
                if cum_i[j] > d_now:
                    if j > 0:
                        frac = (d_now - cum_i[j-1]) / max(cum_i[j] - cum_i[j-1], 1e-9)
                        xs.append(pts_i[j-1]["x"] + (pts_i[j]["x"] - pts_i[j-1]["x"]) * frac)
                        ys.append(pts_i[j-1]["y"] + (pts_i[j]["y"] - pts_i[j-1]["y"]) * frac)
                    break
                xs.append(pts_i[j]["x"])
                ys.append(pts_i[j]["y"])
            if len(xs) > 1:
                ax2.plot(xs, ys, color=col, lw=1.4, alpha=0.35, zorder=3)
                tail = max(0, len(xs) - 40)
                ax2.plot(xs[tail:], ys[tail:], color=col, lw=2.2, alpha=0.8, zorder=4)

        # draw robot dots
        statuses = []
        for (pts_i, cum_i, d_now, col, rid) in [
            (ptsA, cumA, dA, COLA, "A"),
            (ptsB, cumB, dB, COLB, "B"),
        ]:
            if d_now <= 0 and rid == "B":
                continue
            x, y, seg = pos_at_dist(pts_i, cum_i, d_now)
            ax2.add_patch(plt.Circle((x, y), ROBOT_R * 0.6,
                          color=col, alpha=0.3, zorder=8))
            ax2.add_patch(plt.Circle((x, y), ROBOT_R * 0.6,
                          fill=False, ec=col, lw=1.5, zorder=9))
            ax2.text(x, y, rid, ha="center", va="center",
                     fontsize=8, fontweight="bold", color="white", zorder=10)
            if seg["type"] == "wrap":
                statuses.append(f"{rid}: wrapping pole {seg['pole']}")
            elif seg["type"] == "move":
                statuses.append(f"{rid}: → pole {seg['pole']}")

        ax2.text(0.02, 0.02, "  ·  ".join(statuses),
                 transform=ax2.transAxes, fontsize=8, color="#666",
                 verticalalignment="bottom")

        fp = os.path.join(tmpdir, f"f_{fi:04d}.png")
        fig2.savefig(fp, dpi=100, pad_inches=0.1)
        plt.close(fig2)
        frames.append(fp)

        if fi % 60 == 0:
            print(f"  frame {fi}/{n_frames}")

    gif_path = os.path.join(OUT_DIR, "path_simulation.gif")
    imgs = [imageio.imread(f) for f in frames]
    imageio.mimsave(gif_path, imgs, duration=0.05, loop=0)
    shutil.rmtree(tmpdir)
    print(f"Saved  {gif_path}")


if __name__ == "__main__":
    main()