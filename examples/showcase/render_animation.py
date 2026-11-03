"""Render a VehicleMPC showcase run to an H.264 MP4 and a PNG poster.

Usage (normally called by examples/showcase/race.jl and multiple_obstacles.jl after the
simulation of the same run):

    python examples/showcase/render_animation.py --run examples/showcase/output/race_dynamic [--fps 30] [--speed 0]
    python examples/showcase/render_animation.py --run ... --selftest

Inputs: <run>_meta.json, <run>_cycles.csv, <run>_plans.csv, <run>_trajectory.csv and
<run>_track.csv, all written by the Julia run. Nothing else is read: the animation shows
exactly the data of that run.

The vehicle is drawn at the plant state, interpolated linearly between samples of the 10 ms
plant trajectory, and oriented by the heading psi of the vehicle body (not the velocity
direction). The planned path is the accepted MPC solution of the latest control update.
Telemetry shows the latest completed control cycle. Requirements: numpy, matplotlib,
imageio-ffmpeg (bundles an ffmpeg with libx264).
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Polygon, Circle, FancyArrow, Rectangle  # noqa: E402

# ------------------------------------------------------------------ palette (reference design)
BG = "#0a1424"
GRID = "#12233a"
PANEL = "#0e1b2d"
PANEL_EDGE = "#24364f"
CARD = "#132338"
TEXT = "#eef3fa"
MUTED = "#93a4bb"
CYAN = "#37e2c4"
CYAN_DIM = "#1d6f66"
AMBER = "#f5a63c"
ROAD = "#1a2a3e"
EDGE = "#9fb3c8"
RED = "#ff5468"
GREEN = "#43d39e"

W, H, DPI = 1280, 720, 100


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return rows


def fcol(rows, key):
    out = []
    for r in rows:
        v = r[key]
        try:
            out.append(float(v))
        except ValueError:
            out.append(float("nan"))
    return np.array(out)


def load_run(prefix):
    meta = json.load(open(prefix + "_meta.json"))
    cyc = read_csv(prefix + "_cycles.csv")
    traj = read_csv(prefix + "_trajectory.csv")
    track = read_csv(prefix + "_track.csv")
    plans = read_csv(prefix + "_plans.csv")
    plan_by_k = {}
    for r in plans:
        plan_by_k.setdefault(int(r["k"]), []).append((float(r["x"]), float(r["y"])))
    run = {
        "meta": meta,
        "cyc": cyc,
        "ct": fcol(cyc, "t"),
        "t": fcol(traj, "t"),
        "X": fcol(traj, "X"),
        "Y": fcol(traj, "Y"),
        "psi": fcol(traj, "psi"),
        "vx": fcol(traj, "vx"),
        "s": fcol(traj, "s"),
        "ey": fcol(traj, "ey"),
        "track": {k: fcol(track, k) for k in ("xc", "yc", "xl", "yl", "xr", "yr")},
        "plans": {k: np.array(v) for k, v in plan_by_k.items()},
    }
    return run


# ------------------------------------------------------------------ geometry helpers
def body_polygon(x, y, psi, length, width):
    """Corners of the vehicle body: a rectangle centred at (x, y), long axis along psi."""
    c, s = math.cos(psi), math.sin(psi)
    hl, hw = length / 2, width / 2
    pts = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    return np.array([(x + c * a - s * b, y + s * a + c * b) for a, b in pts])


def heading_arrow(x, y, psi, length):
    """Start point and vector of the heading arrow: from the body centre along psi."""
    return x, y, length * math.cos(psi), length * math.sin(psi)


def body_centre(meta, X, Y, psi):
    """Centre of the illustrative body: the CoM (dynamic) or rear axle + half wheelbase (kinematic)."""
    if meta["model"] == "kinematic":
        L = meta["parameters"]["wheelbase"]
        return X + 0.5 * L * math.cos(psi), Y + 0.5 * L * math.sin(psi)
    return X, Y


def interp_state(run, t):
    tt = run["t"]
    i = int(np.searchsorted(tt, t, side="right") - 1)
    i = max(0, min(i, len(tt) - 2))
    w = 0.0 if tt[i + 1] == tt[i] else (t - tt[i]) / (tt[i + 1] - tt[i])
    w = max(0.0, min(1.0, w))
    g = lambda a: a[i] + w * (a[i + 1] - a[i])  # noqa: E731
    return i, g(run["X"]), g(run["Y"]), g(run["psi"]), g(run["vx"]), g(run["s"])


def cycle_at(run, t):
    k = int(np.searchsorted(run["ct"], t + 1e-9, side="right") - 1)
    return max(0, min(k, len(run["cyc"]) - 1))


# ------------------------------------------------------------------ drawing helpers
def rounded(ax_fig, x, y, w, h, fc, ec, lw=1.2, r=18, z=1):
    p = FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}", transform=ax_fig.transData,
                       fc=fc, ec=ec, lw=lw, zorder=z)
    ax_fig.add_patch(p)
    return p


def txt(ax, x, y, s, size=10, color=TEXT, weight="normal", ha="left", va="baseline", z=5, **kw):
    return ax.text(x, y, s, fontsize=size, color=color, fontweight=weight, ha=ha, va=va, zorder=z, **kw)


def solver_label(s):
    return {"LOCALLY_SOLVED": "LOCALLY SOLVED", "OPTIMAL": "OPTIMAL"}.get(s, s.replace("_", " "))


def status_label(st):
    return {"success": "ACCEPTED", "success_stale_schedule": "ACCEPTED (QUALIFIED)"}.get(st, "REJECTED · " + st.replace("_", " ").upper())


class Renderer:
    def __init__(self, run, fps, speed):
        self.run, self.fps = run, fps
        m = run["meta"]
        self.meta = m
        self.multi = len(m["obstacles"]) > 0
        self.T = float(run["t"][-1])
        self.speed = speed if speed > 0 else max(1.0, self.T / 25.0)
        self.nframes = int(math.ceil(self.T / self.speed * fps)) + 1
        self.summary_frames = int(3.5 * fps)
        self.body_len, self.body_wid = m["body_illustrative"]["length"], m["body_illustrative"]["width"]
        self.build()

    # ---------------------------------------------------------- static layout
    def build(self):
        m, run = self.meta, self.run
        fig = plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI, facecolor=BG)
        self.fig = fig
        bg = fig.add_axes([0, 0, 1, 1])
        bg.set_xlim(0, W)
        bg.set_ylim(H, 0)
        bg.axis("off")
        self.bg = bg
        for x in range(0, W, 40):
            bg.plot([x, x], [0, H], color=GRID, lw=0.6, alpha=0.55, zorder=0)
        for y in range(0, H, 40):
            bg.plot([0, W], [y, y], color=GRID, lw=0.6, alpha=0.55, zorder=0)
        # header
        txt(bg, 32, 58, "VEHICLE MPC", 25, TEXT, "bold")
        title = "NONLINEAR MPC RACING  /  JULIA + JuMP + IPOPT"
        txt(bg, 330, 52, title, 11.5, MUTED, "bold")
        rounded(bg, 1092, 28, 160, 36, "#123c3c", "#123c3c", r=16, z=2)
        txt(bg, 1172, 51, "SIMULATION", 10.5, CYAN, "bold", ha="center")
        # panels
        rounded(bg, 28, 100, 788, 566, PANEL, PANEL_EDGE, r=22)
        rounded(bg, 838, 100, 416, 566, PANEL, PANEL_EDGE, r=22)
        L = m["track"]["length"]
        txt(bg, 56, 134, "FULL MULTI-CURVATURE CIRCUIT", 12.5, TEXT, "bold")
        sub = f"Closed track  ·  {L:.2f} m/lap  ·  {m['track']['width']:.2f} m road width"
        if self.multi:
            sub += f"  ·  {len(m['obstacles'])} stationary obstacles"
        txt(bg, 57, 160, sub, 9.5, MUTED)
        rounded(bg, 640, 113, 158, 34, "#13283b", "#13283b", r=10, z=2)
        self.time_txt = txt(bg, 656, 136, "", 11, TEXT, "bold")
        # legend + time bar
        txt(bg, 56, 626, "CLOSED-LOOP TRAJECTORY", 8.5, MUTED, "bold")
        txt(bg, 56, 645, "—  TRACK EDGE    ···  CENTERLINE    —  VEHICLE TRAIL    - -  MPC PLAN (ACCEPTED)", 8.3, MUTED)
        bg.plot([56, 790], [656, 656], color="#22344b", lw=4, solid_capstyle="round", zorder=3)
        self.bar, = bg.plot([56, 56], [656, 656], color=CYAN, lw=4, solid_capstyle="round", zorder=4)
        # right panel
        txt(bg, 866, 140, "NONLINEAR MPC RACING", 13.5, TEXT, "bold")
        txt(bg, 866, 163, "Progress-maximizing trajectory optimization", 9.8, MUTED)
        bg.plot([866, 1226], [182, 182], color=PANEL_EDGE, lw=1.2, zorder=3)
        dyn = m["model"] == "dynamic"
        txt(bg, 866, 212, "SPEED  vₓ" if dyn else "SPEED  v", 9.5, MUTED, "bold")
        self.speed_txt = txt(bg, 866, 258, "", 30, CYAN, "bold")
        txt(bg, 1010, 258, "m/s", 11, MUTED)
        bg.plot([1080, 1226], [244, 244], color="#22344b", lw=8, solid_capstyle="round", zorder=3)
        self.speed_bar, = bg.plot([1080, 1080], [244, 244], color=CYAN, lw=8, solid_capstyle="round", zorder=4)
        txt(bg, 866, 298, "STEERING", 9.5, MUTED, "bold")
        self.steer_txt = txt(bg, 866, 332, "", 20, TEXT, "bold")
        txt(bg, 1050, 298, "LAP PROGRESS", 9.5, MUTED, "bold")
        self.lap_txt = txt(bg, 1050, 332, "", 20, TEXT, "bold")
        self.dist_txt = txt(bg, 1050, 352, "", 9.5, AMBER)
        y0 = 384
        if self.multi:
            txt(bg, 866, y0, "OBSTACLE CLEARANCE", 9.5, MUTED, "bold")
            self.clr_txt = txt(bg, 866, y0 + 32, "", 18, CYAN, "bold")
            txt(bg, 1050, y0, "MIN SO FAR", 9.5, MUTED, "bold")
            self.minclr_txt = txt(bg, 1050, y0 + 32, "", 18, TEXT, "bold")
            self.buf_txt = txt(bg, 866, y0 + 54, "", 9.0, MUTED)
            y0 += 78
        else:
            y0 += 10
        rounded(bg, 862, y0, 368, 112, CARD, "#28405c", r=14, z=2)
        txt(bg, 882, y0 + 30, "SOLVER", 9.5, MUTED, "bold", z=6)
        txt(bg, 882, y0 + 62, "CONTROL", 9.5, MUTED, "bold", z=6)
        txt(bg, 882, y0 + 94, "MPC TIME", 9.5, MUTED, "bold", z=6)
        self.solver_txt = txt(bg, 980, y0 + 30, "", 11, GREEN, "bold", z=6)
        self.control_txt = txt(bg, 980, y0 + 62, "", 11, GREEN, "bold", z=6)
        self.mpc_txt = txt(bg, 980, y0 + 94, "", 11, TEXT, "bold", z=6)
        y1 = y0 + 130
        model = m["model_description"]
        txt(bg, 866, y1 + 4, "MODEL", 8.5, MUTED, "bold")
        txt(bg, 866, y1 + 24, model, 10.5, TEXT, "bold")
        txt(bg, 866, y1 + 43, f"JuMP + Ipopt  ·  N = {m['horizon']}  ·  dt = {m['dt']} s", 9.0, MUTED)
        # footer
        txt(bg, 40, 700, f"Solver-generated closed-loop simulation computed in this run  ·  commit {m['git_commit']}", 8.5, "#7e8ea6")
        self.speed_chip = txt(bg, 1240, 700, "", 8.5, "#9fb0c6", "bold", ha="right")
        # track axes
        tr = run["track"]
        xs = np.concatenate([tr["xl"], tr["xr"]])
        ys = np.concatenate([tr["yl"], tr["yr"]])
        ax = fig.add_axes([56 / W, 1 - 612 / H, 734 / W, 436 / H])
        pad = 0.45
        ax.set_xlim(xs.min() - pad, xs.max() + pad)
        ax.set_ylim(ys.min() - pad, ys.max() + pad)
        self.ax = ax
        self.views = [self.make_view(ax, overview=True)]
        # Close-up inset (to scale), placed in the empty interior of the circuit.
        ax.set_aspect("equal")
        fig.canvas.draw()
        box = [xs.min() + 1.1, ys.max() - 6.0, xs.min() + 5.5, ys.max() - 1.6]   # inside the circuit, clear of the S-bend
        (fx0, fy0), (fx1, fy1) = fig.transFigure.inverted().transform(ax.transData.transform([(box[0], box[1]), (box[2], box[3])]))
        side = min(fx1 - fx0, (fy1 - fy0) * H / W)
        axz = fig.add_axes([fx0, fy0, side, side * W / H])
        for sp in axz.spines.values():
            sp.set_edgecolor("#3c5878")
            sp.set_linewidth(1.2)
        self.axz = axz
        self.zoom_half = 1.2
        self.views.append(self.make_view(axz, overview=False))
        boxstyle = dict(boxstyle="round,pad=0.25", fc="#0b1626", ec="none", alpha=0.85)
        axz.text(0.03, 0.97, "CLOSE-UP  ·  TO SCALE  ·  2.4 m", transform=axz.transAxes, fontsize=7.5, color=MUTED,
                 fontweight="bold", va="top", zorder=20, bbox=boxstyle)
        axz.text(0.03, 0.03, "body illustrative · collision proxy: 5 cm disk", transform=axz.transAxes, fontsize=6.8,
                 color="#9aaac0", va="bottom", zorder=20, bbox=boxstyle)
        self.banner = txt(bg, 422, 590, "", 11, RED, "bold", ha="center", z=9)

    def make_view(self, ax, overview):
        m, tr = self.meta, self.run["track"]
        ax.set_facecolor(PANEL if overview else "#0b1626")
        if overview:
            ax.axis("off")
        else:
            ax.set_xticks([])
            ax.set_yticks([])
        ax.set_aspect("equal")
        lw = 1.6 if overview else 2.2
        poly = np.vstack([np.column_stack([tr["xl"], tr["yl"]]), np.column_stack([tr["xr"], tr["yr"]])[::-1]])
        ax.add_patch(Polygon(poly, closed=True, fc=ROAD, ec="none", zorder=1))
        ax.plot(tr["xl"], tr["yl"], color=EDGE, lw=lw, zorder=2)
        ax.plot(tr["xr"], tr["yr"], color=EDGE, lw=lw, zorder=2)
        ax.plot(tr["xc"], tr["yc"], color="#5b6f88", lw=0.8, ls=(0, (1, 2.5)), zorder=2)
        ax.plot([tr["xr"][0], tr["xl"][0]], [tr["yr"][0], tr["yl"][0]], color="white", lw=2.2, zorder=3)
        rings = []
        for o in m["obstacles"]:
            ax.add_patch(Circle((o["x"], o["y"]), o["radius"] + o["buffer"], fc="none", ec="#ff8f9b", lw=0.9 if overview else 1.2,
                                ls=(0, (3, 2.5)), alpha=0.75, zorder=4))
            ax.add_patch(Circle((o["x"], o["y"]), o["radius"], fc=RED, ec="#ffd1d6", lw=0.8, zorder=5))
            ring = Circle((o["x"], o["y"]), o["radius"] + o["buffer"] + (0.05 if overview else 0.025), fc="none", ec=AMBER,
                          lw=1.6, zorder=5, visible=False)
            ax.add_patch(ring)
            rings.append(ring)
        glow, = ax.plot([], [], color=CYAN, lw=7 if overview else 10, alpha=0.18, solid_capstyle="round", zorder=6)
        trail, = ax.plot([], [], color=CYAN, lw=2.4 if overview else 3.0, solid_capstyle="round", zorder=7)
        plan, = ax.plot([], [], color="#e9fffb", lw=1.4 if overview else 1.8, ls=(0, (3, 2)), alpha=0.95, zorder=8)
        body = Polygon(np.zeros((4, 2)), closed=True, fc=AMBER, ec="#3a2306", lw=1.0, zorder=10)
        ax.add_patch(body)
        return {"ax": ax, "overview": overview, "rings": rings, "glow": glow, "trail": trail, "plan": plan, "body": body, "arrow": None}

    # ---------------------------------------------------------- per frame
    def frame(self, t):
        run, m = self.run, self.meta
        i, X, Y, psi, vx, s = interp_state(run, t)
        k = cycle_at(run, t)
        row = run["cyc"][k]
        n0 = np.searchsorted(run["t"], t, side="right")
        tx = np.append(run["X"][:n0], X)
        ty = np.append(run["Y"][:n0], Y)
        accepted = row["accepted"] == "true"
        plan = run["plans"].get(int(row["k"]))
        cx, cy = body_centre(m, X, Y, psi)
        for v in self.views:
            v["trail"].set_data(tx, ty)
            v["glow"].set_data(tx, ty)
            if accepted and plan is not None and len(plan):
                v["plan"].set_data(plan[:, 0], plan[:, 1])
            else:
                v["plan"].set_data([], [])
            v["body"].set_xy(body_polygon(cx, cy, psi, self.body_len, self.body_wid))
            if v["arrow"] is not None:
                v["arrow"].remove()
            alen, aw, hw, hl = (0.55, 0.045, 0.17, 0.15) if v["overview"] else (0.30, 0.018, 0.075, 0.07)
            ax0, ay0, adx, ady = heading_arrow(cx, cy, psi, alen)
            v["arrow"] = FancyArrow(ax0, ay0, adx, ady, width=aw, head_width=hw, head_length=hl, length_includes_head=True,
                                    fc="white", ec="#0a1424", lw=0.6, zorder=11)
            v["ax"].add_patch(v["arrow"])
        self.axz.set_xlim(cx - self.zoom_half, cx + self.zoom_half)
        self.axz.set_ylim(cy - self.zoom_half, cy + self.zoom_half)
        # telemetry
        L = m["track"]["length"]
        s0 = run["s"][0]
        laps = (s - s0) / L
        self.time_txt.set_text(f"T + {t:4.1f} s")
        self.speed_txt.set_text(f"{vx:.2f}")
        frac = max(0.0, min(1.0, vx / m["parameters"].get("vx_max", m["parameters"].get("v_max", 5.0))))
        self.speed_bar.set_data([1080, 1080 + 146 * frac], [244, 244])
        d = float(row["delta"])
        self.steer_txt.set_text("—" if math.isnan(d) else f"{math.degrees(d):+.1f}°")
        self.lap_txt.set_text(f"{laps:.2f} laps")
        self.dist_txt.set_text(f"{s - s0:.1f} m")
        self.bar.set_data([56, 56 + 734 * min(1.0, t / max(self.T, 1e-9))], [656, 656])
        self.solver_txt.set_text(solver_label(row["solver"]))
        self.solver_txt.set_color(GREEN if row["solver"] in ("LOCALLY_SOLVED", "OPTIMAL") else RED)
        self.control_txt.set_text(status_label(row["status"]))
        self.control_txt.set_color(GREEN if accepted else RED)
        self.mpc_txt.set_text(f"{float(row['total_ms']):.1f} ms")
        self.banner.set_text("" if accepted else f"CONTROL REJECTED AT T = {float(row['t']):.1f} s — RUN STOPPED ({row['status']})")
        if self.multi:
            px, py = X, Y  # collision proxy: CoM (dynamic) or rear axle (kinematic)
            best, bi = math.inf, -1
            for j, o in enumerate(m["obstacles"]):
                dd = math.hypot(px - o["x"], py - o["y"]) - o["radius"] - m["collision_proxy"]["ego_radius"]
                if dd < best:
                    best, bi = dd, j
            self.min_so_far = min(getattr(self, "min_so_far", math.inf), best)
            for v in self.views:
                for j, ring in enumerate(v["rings"]):
                    ring.set_visible(j == bi and best < 1.0)
            self.clr_txt.set_text(f"{100 * best:.1f} cm")
            self.clr_txt.set_color(CYAN if best > m["obstacles"][bi]["buffer"] else (AMBER if best > 0 else RED))
            self.minclr_txt.set_text(f"{100 * self.min_so_far:.1f} cm")
            buf = best < m["obstacles"][bi]["buffer"]
            self.buf_txt.set_text("Physical clearance (ego disk to obstacle)  ·  " + ("INSIDE BUFFER" if buf else "buffer clear"))
            self.buf_txt.set_color(AMBER if buf else MUTED)
        self.speed_chip.set_text(f"PLAYBACK {self.speed:.1f}× SIMULATION TIME  ·  {self.fps} FPS")

    def summary(self):
        st = self.meta["statistics"]
        # The card goes on its own full-figure axes added last, so it covers the track view
        # and the close-up inset (separate axes, drawn above the background axes).
        bg = self.fig.add_axes([0, 0, 1, 1], zorder=50)
        bg.set_xlim(0, W)
        bg.set_ylim(H, 0)
        bg.axis("off")
        bg.patch.set_alpha(0.0)
        artists = [rounded(bg, 200, 175, 880, 400, "#0c1828", CYAN_DIM, lw=1.6, r=24, z=20)]
        ok = st["status"] == "success"
        artists.append(txt(bg, 240, 228, "RUN SUMMARY", 18, TEXT, "bold", z=21))
        artists.append(txt(bg, 240, 254, f"{self.meta['model_description']}  ·  nonlinear MPC (JuMP + Ipopt)  ·  this run",
                           10, MUTED, z=21))
        artists.append(txt(bg, 1040, 228, "COMPLETED" if ok else "RUN FAILED", 13, GREEN if ok else RED, "bold", ha="right", z=21))
        fail = self.meta.get("failure")
        if not ok and fail:
            artists.append(txt(bg, 660, 516, f"STOPPED AT CYCLE {fail['cycle']}: {fail['status']}", 9.5, RED, "bold", z=21))
        rows = [
            ("Completed laps", f"{st['laps']:.2f}"),
            ("Simulated time", f"{st['simulated_time']:.1f} s"),
            ("Progress", f"{st['progress']:.1f} m"),
            ("Median MPC time", f"{st['median_ms']:.1f} ms"),
            ("Max MPC time", f"{st['max_ms']:.1f} ms"),
            ("Cycles within 100 ms", f"{st['within_100_pct']:.1f} %"),
            ("Accepted cycles", f"{st['accepted']}"),
            ("Failed cycles", f"{st['failed']}"),
        ]
        if st["obstacles"] > 0:
            rows += [
                ("Obstacles", f"{st['obstacles']}"),
                ("Min physical clearance", f"{100 * st['min_contact_clearance']:.2f} cm"),
                ("Min clearance to buffer", f"{100 * st['min_buffer_clearance']:+.2f} cm"),
                ("Physical collisions", f"{st['collisions']}"),
                ("Track-edge violations", f"{st['track_edge_violations']}"),
            ]
        else:
            rows += [("Track-edge violations", f"{st['track_edge_violations']}")]
        ncol = 2
        per = math.ceil(len(rows) / ncol)
        for j, (a, b) in enumerate(rows):
            col, r = divmod(j, per)
            x = 240 + col * 420
            y = 300 + r * 36
            artists.append(txt(bg, x, y, a.upper(), 9.5, MUTED, "bold", z=21))
            artists.append(txt(bg, x + 370, y, b, 13, CYAN if j < 3 else TEXT, "bold", ha="right", z=21))
        artists.append(txt(bg, 240, 556, "Locally optimal MPC solutions from a numerical simulation; not a real-time or safety guarantee.",
                           9, "#7e8ea6", z=21))
        return artists

    def render(self, out_mp4, out_png, poster_time=None):
        import imageio_ffmpeg
        self.fig.canvas.draw()
        w, h = self.fig.canvas.get_width_height()
        writer = imageio_ffmpeg.write_frames(out_mp4, (w, h), fps=self.fps, codec="libx264", pix_fmt_in="rgb24",
                                             pix_fmt_out="yuv420p", quality=None, bitrate=None,
                                             output_params=["-crf", "22", "-preset", "medium", "-movflags", "+faststart"])
        writer.send(None)
        poster_time = self.T * 0.62 if poster_time is None else poster_time
        poster_done = False
        for f in range(self.nframes):
            t = min(self.T, f * self.speed / self.fps)
            self.frame(t)
            self.fig.canvas.draw()
            img = np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3]
            writer.send(np.ascontiguousarray(img))
            if not poster_done and t >= poster_time:
                self.fig.savefig(out_png, dpi=DPI, facecolor=BG)
                poster_done = True
        self.frame(self.T)
        arts = self.summary()
        self.fig.canvas.draw()
        img = np.ascontiguousarray(np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3])
        for _ in range(self.summary_frames):
            writer.send(img)
        writer.close()
        if not poster_done:
            for a in arts:
                a.remove()
            self.frame(min(self.T, poster_time))
            self.fig.savefig(out_png, dpi=DPI, facecolor=BG)


def selftest(run):
    """Numerical checks of the renderer against the run data (no video)."""
    m = run["meta"]
    errs = []
    # 1. Heading arrow points along psi (unit-vector agreement), for every plant sample used.
    for j in range(0, len(run["t"]), 37):
        x0, y0, dx, dy = heading_arrow(run["X"][j], run["Y"][j], run["psi"][j], 1.0)
        if abs(dx - math.cos(run["psi"][j])) > 1e-12 or abs(dy - math.sin(run["psi"][j])) > 1e-12:
            errs.append(f"arrow direction mismatch at sample {j}")
        poly = body_polygon(run["X"][j], run["Y"][j], run["psi"][j], 0.36, 0.20)
        front_mid = (poly[0] + poly[1]) / 2 - np.array([run["X"][j], run["Y"][j]])
        ang = math.atan2(front_mid[1], front_mid[0])
        if abs(math.remainder(ang - run["psi"][j], 2 * math.pi)) > 1e-9:
            errs.append(f"body orientation mismatch at sample {j}")
    # 2. Interpolated frames reproduce the plant samples at the sample times.
    for j in range(0, len(run["t"]), 53):
        _, X, Y, psi, _, _ = interp_state(run, run["t"][j])
        if abs(X - run["X"][j]) > 1e-9 or abs(Y - run["Y"][j]) > 1e-9:
            errs.append(f"interpolation mismatch at t = {run['t'][j]}")
    # 3. Control-cycle rows and the plant trajectory start from the same state.
    c0 = run["cyc"][0]
    if abs(float(c0["X"]) - run["X"][0]) > 1e-9 or abs(float(c0["Y"]) - run["Y"][0]) > 1e-9:
        errs.append("first control cycle and plant trajectory start at different positions")
    # 4. Every row shown as ACCEPTED has an accepted status and a control.
    for r in run["cyc"]:
        if (r["accepted"] == "true") != (r["status"] in ("success", "success_stale_schedule")):
            errs.append(f"cycle {r['k']}: accepted flag and status disagree")
        if r["accepted"] != "true" and r["delta"] not in ("NaN", "nan"):
            errs.append(f"cycle {r['k']}: rejected cycle carries a control")
    # 5. Statistics in the metadata agree with the per-cycle data.
    st = m["statistics"]
    if st["cycles"] != len(run["cyc"]) or st["accepted"] != sum(r["accepted"] == "true" for r in run["cyc"]):
        errs.append("statistics disagree with the per-cycle data")
    return errs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="prefix of the exported run files")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--speed", type=float, default=0.0, help="playback factor relative to simulation time (0 = auto, at most 25 s)")
    ap.add_argument("--selftest", action="store_true", help="check renderer geometry against the run data and exit")
    a = ap.parse_args()
    run = load_run(a.run)
    errs = selftest(run)
    if a.selftest:
        print("selftest:", "passed" if not errs else "FAILED")
        for e in errs:
            print("  ", e)
        sys.exit(1 if errs else 0)
    if errs:
        print("refusing to render: the run data are inconsistent:\n  " + "\n  ".join(errs))
        sys.exit(1)
    r = Renderer(run, a.fps, a.speed)
    r.render(a.run + ".mp4", a.run + "_poster.png")
    print(f"wrote {a.run}.mp4 ({r.nframes + r.summary_frames} frames, {a.fps} fps, playback {r.speed:.2f}x) and {a.run}_poster.png")


if __name__ == "__main__":
    main()
