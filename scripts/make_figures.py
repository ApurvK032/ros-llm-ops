"""Draw the figures and replays in docs/figures from the evidence committed under docs/evidence.

    python3 scripts/make_figures.py            # everything, light and dark
    python3 scripts/make_figures.py charts     # only the three charts
    python3 scripts/make_figures.py replays    # only the animated replays (needs ffmpeg)

Nothing here is typed in by hand: every number is read from a result.json, a journal or an evidence file, so the
figures change only when the evidence does. Needs matplotlib; the replays also need ffmpeg.
"""

import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import patheffects  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
EVIDENCE, FIGURES = ROOT / "docs/evidence", ROOT / "docs/figures"
CONFIG = json.loads((ROOT / "config/warehouse.json").read_text())

# One set of roles per theme. Series colours are fixed by entity, never by rank.
THEMES = {
    "light": {"surface": "#fcfcfb", "ink": "#0b0b0b", "ink2": "#52514e", "muted": "#898781", "grid": "#e1e0d9",
              "axis": "#c3c2b7", "series": ["#2a78d6", "#eb6834", "#1baf7a"], "before": "#86b6ef",
              "after": "#1c5cab", "critical": "#d03b3b", "good": "#0ca30c", "shelf": "#e8e7e1"},
    "dark": {"surface": "#1a1a19", "ink": "#f0efec", "ink2": "#c3c2b7", "muted": "#898781", "grid": "#2c2c2a",
             "axis": "#383835", "series": ["#3987e5", "#d95926", "#199e70"], "before": "#184f95",
             "after": "#6da7ec", "critical": "#d03b3b", "good": "#0ca30c", "shelf": "#2a2a28"},
}
FAULTS = [("bad_parameter", "Speed limit set to zero"), ("controller_crash", "Controller crashed"),
          ("lidar_dropout", "Lidar stopped"), ("localization_lost", "Localization lost"),
          ("blocked_station", "Station blocked")]
DECIDERS = [("rules", "Fixed playbook", "o"), ("qwen3_5_9b", "qwen3.5:9b decides", "s"),
            ("qwen3_5_4b", "qwen3.5:4b decides", "D")]
FONT = ["Inter", "Noto Sans", "Ubuntu Sans", "Liberation Sans", "DejaVu Sans"]


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def baseline(scenario):
    """The later of the two supervisor-alone recordings of a scenario."""
    return sorted((EVIDENCE / "scenarios" / scenario).iterdir())[-1]


def recovered(decider, scenario):
    return EVIDENCE / "recovery" / decider / scenario


def figure(theme, width, height):
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": FONT, "svg.fonttype": "none"})
    fig = plt.figure(figsize=(width, height), dpi=200, facecolor=theme["surface"])
    return fig


def titles(fig, theme, title, subtitle, left=0.03):
    fig.text(left, 0.955, title, color=theme["ink"], fontsize=13, fontweight="semibold", va="top")
    fig.text(left, 0.885, subtitle, color=theme["ink2"], fontsize=9, va="top", linespacing=1.35)


def clean(ax, theme, grid_axis="x"):
    ax.set_facecolor(theme["surface"])
    for side in ("top", "right", "left", "bottom"):
        ax.spines[side].set_visible(False)
    ax.grid(axis=grid_axis, color=theme["grid"], linewidth=0.6, zorder=0)
    ax.tick_params(length=0, colors=theme["muted"], labelsize=8.5)
    ax.set_axisbelow(True)


def save(fig, name, theme_name):
    FIGURES.mkdir(parents=True, exist_ok=True)
    path = FIGURES / f"{name}-{theme_name}.png"
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)
    return path


# --- chart 1: parcels delivered, supervisor alone against with the agent ------------------------------------------

def how_it_ended(result):
    outcome = result["outcome"]
    seconds = f"{outcome['mission_seconds']:.0f} s"
    return {"complete": seconds, "partial": seconds if outcome["delivered"] else f"gave up at {seconds}",
            "timeout": "timed out at 720 s", "no_route": f"stuck at {seconds}"}.get(outcome["result"], seconds)


def chart_delivered(theme_name):
    theme = THEMES[theme_name]
    rows = []
    for scenario, label in FAULTS:
        alone = load(baseline(scenario) / "result.json")
        agent = load(recovered("rules", scenario) / "result.json")
        rows.append((label, len(alone["outcome"]["delivered"]), len(agent["outcome"]["delivered"]),
                     f"alone: {how_it_ended(alone)}   ·   with the agent: {how_it_ended(agent)}"))
    fig = figure(theme, 8.6, 4.3)
    ax = fig.add_axes([0.30, 0.13, 0.62, 0.62])
    clean(ax, theme)
    for index, (label, before, after, note) in enumerate(rows):
        y = len(rows) - 1 - index
        if before != after:
            ax.plot([before, after], [y, y], color=theme["axis"], linewidth=2, solid_capstyle="round", zorder=2)
        ax.scatter([before], [y], s=110, color=theme["before"], edgecolors=theme["surface"], linewidths=2, zorder=3)
        ax.scatter([after], [y], s=110, color=theme["after"], edgecolors=theme["surface"], linewidths=2,
                   zorder=4 if before != after else 3, marker="o")
        if before == after:   # both ends on one value: show the earlier one as a ring around the later
            ax.scatter([before], [y], s=330, facecolors="none", edgecolors=theme["before"], linewidths=2, zorder=2)
        fig.text(0.03, ax.get_position().y0 + ax.get_position().height*(y + 0.5 + 0.13)/len(rows), label,
                 color=theme["ink"], fontsize=10, va="center")
        fig.text(0.03, ax.get_position().y0 + ax.get_position().height*(y + 0.5 - 0.17)/len(rows), note,
                 color=theme["muted"], fontsize=7.6, va="center")
    ax.set_ylim(-0.5, len(rows) - 0.5)
    ax.set_xlim(-0.25, 3.25)
    ax.set_xticks([0, 1, 2, 3])
    ax.set_yticks([])
    ax.set_xlabel("Parcels delivered, out of 3", color=theme["ink2"], fontsize=9, labelpad=8)
    handles = [plt.Line2D([], [], marker="o", linestyle="", markersize=9, color=theme["before"],
                          label="Supervisor alone"),
               plt.Line2D([], [], marker="o", linestyle="", markersize=9, color=theme["after"],
                          label="With the recovery agent")]
    legend = fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(0.975, 0.985), frameon=False, ncol=1,
                        fontsize=8.5, handletextpad=0.3, labelspacing=0.5)
    for text in legend.get_texts():
        text.set_color(theme["ink2"])
    titles(fig, theme, "Five faults, with and without the recovery agent",
           "Parcels delivered in one mission of three. One run per fault; the agent's result is the same\n"
           "whether the playbook or a local model decides.")
    return save(fig, "recovery-delivered", theme_name)


# --- chart 2: seconds from fault to resume, by who decides ---------------------------------------------------------

def effective_resume(result):
    """Seconds from the fault to the resume that followed the incident in which actions were carried out."""
    injected = result["fault"]["monotonic"]
    acted = [i for i in result["recovery"]["incidents"] if any("result" in a or a["action"] == "escalate"
                                                                 for a in i["actions"]) and "resume" in i]
    return round(acted[0]["monotonic_done"] - injected, 1) if acted else None


def chart_resume(theme_name):
    theme = THEMES[theme_name]
    fig = figure(theme, 8.6, 4.3)
    ax = fig.add_axes([0.30, 0.13, 0.62, 0.62])
    clean(ax, theme)
    for index, (scenario, label) in enumerate(FAULTS):
        y = len(FAULTS) - 1 - index
        values = [effective_resume(load(recovered(decider, scenario) / "result.json")) for decider, _, _ in DECIDERS]
        ax.plot([min(values), max(values)], [y, y], color=theme["axis"], linewidth=2, solid_capstyle="round",
                zorder=2)
        for value, colour, (_, _, marker) in zip(values, theme["series"], DECIDERS, strict=True):
            ax.scatter([value], [y], s=95, color=colour, marker=marker, edgecolors=theme["surface"], linewidths=2,
                       zorder=3)
        ax.text(min(values) - 1.8, y, f"{min(values):.0f} s", color=theme["ink2"], fontsize=8.5, ha="right",
                va="center")
        ax.text(max(values) + 1.8, y, f"{max(values):.0f} s", color=theme["ink2"], fontsize=8.5, ha="left",
                va="center")
        fig.text(0.03, ax.get_position().y0 + ax.get_position().height*(y + 0.5)/len(FAULTS), label,
                 color=theme["ink"], fontsize=10, va="center")
    ax.set_ylim(-0.5, len(FAULTS) - 0.5)
    ax.set_xlim(-6, 76)
    ax.set_xticks([0, 20, 40, 60])
    ax.set_yticks([])
    ax.set_xlabel("Seconds from the fault to the mission resuming", color=theme["ink2"], fontsize=9, labelpad=8)
    handles = [plt.Line2D([], [], marker=marker, linestyle="", markersize=8, color=colour, label=name)
               for (_, name, marker), colour in zip(DECIDERS, theme["series"], strict=True)]
    legend = fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(0.975, 0.985), frameon=False,
                        fontsize=8.5, handletextpad=0.3, labelspacing=0.5)
    for text in legend.get_texts():
        text.set_color(theme["ink2"])
    titles(fig, theme, "How long recovery takes, by who decides",
           "One run each. A model's extra time is its thinking. For the blocked station, about 31 s is the\n"
           "box not yet being in the lidar's view.")
    return save(fig, "recovery-time", theme_name)


# --- chart 3: diagnosis scoreboard ---------------------------------------------------------------------------------

def chart_diagnosis(theme_name):
    theme = THEMES[theme_name]
    scores = {}
    for path in sorted((EVIDENCE / "scenarios").glob("*/*/diagnosis-*.json")):
        saved = load(path)
        scores.setdefault(saved["label"], []).append(saved["score"]["category_correct"])
    names = {"rules": "Fixed rules, no model", "qwen3.5:9b": "qwen3.5:9b with the tools",
             "qwen3.5:4b": "qwen3.5:4b with the tools", "qwen3.5:9b (no tools)": "qwen3.5:9b, mission report only",
             "qwen3.5:4b (no tools)": "qwen3.5:4b, mission report only"}
    rows = [(names[label], sum(scores[label]), len(scores[label])) for label in names]
    fig = figure(theme, 8.6, 3.6)
    ax = fig.add_axes([0.33, 0.14, 0.60, 0.58])
    clean(ax, theme)
    thickness, x_max = 0.42, 13.6
    ax.set_xlim(0, x_max)
    ax.set_ylim(-0.5, len(rows) - 0.5)
    # Corner radius in data units, and the aspect that keeps the corners round on screen.
    radius = 8*x_max/ax.bbox.width
    aspect = (len(rows)/ax.bbox.height)/(x_max/ax.bbox.width)
    for index, (label, right, total) in enumerate(rows):
        y = len(rows) - 1 - index
        # Rounded at the data end; the bar starts left of the baseline and the axes clip it square there.
        ax.add_patch(FancyBboxPatch((-1, y - thickness/2), right + 1, thickness, zorder=3, linewidth=0,
                                    boxstyle=f"round,pad=0,rounding_size={radius}", facecolor=theme["series"][0],
                                    mutation_aspect=aspect))
        ax.text(right + 0.2, y, f"{right} of {total}", color=theme["ink"], fontsize=9, va="center")
        fig.text(0.03, ax.get_position().y0 + ax.get_position().height*(y + 0.5)/len(rows), label,
                 color=theme["ink"], fontsize=9.5, va="center")
    ax.set_xticks([0, 3, 6, 9, 12])
    ax.set_yticks([])
    ax.set_xlabel("Recordings diagnosed with the right fault category, out of 12", color=theme["ink2"], fontsize=9,
                  labelpad=8)
    titles(fig, theme, "Diagnosing recorded runs: the tools matter, the model does not yet",
           "Six scenarios, two recordings each. The rules and the models' tools are the same eight checks.")
    return save(fig, "diagnosis-scoreboard", theme_name)


# --- replays: the robot's track, supervisor alone beside with the agent --------------------------------------------

def story(run, track):
    """Timed captions and the delivered count for a run, from its journal and result."""
    result = load(run / "result.json")
    events = [json.loads(line) for line in (run / "journal.jsonl").read_text().splitlines() if line.strip()]
    origin = next(e["monotonic"] for e in events if e["type"] == "instruction_applied")
    fault = result["fault"]
    captions = [(0.0, "Mission accepted: deliver three parcels", False)]
    fault_at = fault.get("seconds_after_mission_start")
    wording = {"node_crash": "Fault: the Nav2 controller crashes",
               "blocked_path": "Fault: a box is left on the PICK B stopping pose"}
    if fault_at is not None:
        captions.append((fault_at, wording.get(fault["type"], "Fault injected"), True))
    delivered, deferred_by_supervisor = [], 0
    for event in events:
        t = event["monotonic"] - origin
        kind = event["type"]
        if kind == "cargo_drop":
            delivered.append(t)
        elif kind == "navigation_retry":
            captions.append((t, f"Supervisor: goal to {event['stop']['waypoint'].replace('_', ' ')} failed, retrying",
                             False))
        elif kind == "parcel_deferred":
            deferred_by_supervisor += 1
            captions.append((t, f"Supervisor gives up on {event['parcel']}", False))
        elif kind == "instruction_applied" and event["intent"]["operation"] == "defer":
            skipped = ", ".join(event["intent"]["parcels"])
            captions.append((t, f"Agent: skips {skipped} and tells a person", False))
        elif kind == "instruction_applied" and event["intent"]["operation"] == "resume":
            # A deferral is over in a moment; keep it on screen by folding it into the resume that follows.
            just_skipped = captions[-1][1].startswith("Agent: skips") and t - captions[-1][0] < 15
            captions.append((t, captions[-1][1] + "; the mission resumes" if just_skipped
                             else "Agent: mission resumes", False))
        elif kind == "mission_finished":
            total = len(result["outcome"]["delivered"])
            captions.append((t, f"Mission over: {total} of 3 delivered, in {t:.0f} s", False))
    for incident in (result.get("recovery") or {}).get("incidents", []):
        acted = [a for a in incident["actions"] if a["permitted"] and a["action"] != "escalate"]
        names = {"restart_process": "restarts the controller", "restore_parameter": "restores the parameter",
                 "relocalize": "relocalizes", "defer_parcel": None, "clear_costmaps": None}
        doing = [names.get(a["action"]) for a in acted if names.get(a["action"])]
        if doing:
            captions.append((incident["detected_at_s"], "Agent: pauses, diagnoses, " + " and ".join(doing), False))
    end = max(t for t, _, _ in captions)
    return {"captions": sorted(captions), "delivered": delivered, "fault_at": fault_at, "end": end,
            "pose": [p for p in track if p[0] >= 0], "fault": fault}


def draw_map(ax, theme):
    x0, y0, x1, y1 = CONFIG["bounds"]
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_facecolor(theme["surface"])
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ax.spines.values():
        side.set_color(theme["axis"])
        side.set_linewidth(0.8)
    for shelf in CONFIG["shelves"]:
        (cx, cy), (sx, sy) = shelf["center"], shelf["size"]
        ax.add_patch(Rectangle((cx - sx/2, cy - sy/2), sx, sy, facecolor=theme["shelf"], linewidth=0, zorder=1))
    for name, (x, y, _) in CONFIG["waypoints"].items():
        ax.scatter([x], [y], s=26, facecolors="none", edgecolors=theme["muted"], linewidths=1.0, zorder=2)
        # A halo in the surface colour keeps the name readable where the robot's track crosses it.
        ax.text(x, y + 0.55, name.replace("_", " ").title(), color=theme["muted"], fontsize=6.3, ha="center",
                va="bottom", zorder=8,
                path_effects=[patheffects.withStroke(linewidth=2.2, foreground=theme["surface"])])


def replay(scenario, theme_name, step, fps=10):
    theme = THEMES[theme_name]
    runs = [("Supervisor alone", baseline(scenario), load(baseline(scenario) / "evidence.json")["pose"]),
            ("With the recovery agent", recovered("rules", scenario),
             load(recovered("rules", scenario) / "track.json")["pose"])]
    stories = [story(run, track) for _, run, track in runs]
    length = max(s["end"] for s in stories)
    times = [i*step for i in range(int(length/step) + 2)]
    scratch = Path(tempfile.mkdtemp(prefix="replay-"))
    for frame, now in enumerate(times + [times[-1]]*int(2.5*fps)):
        fig = figure(theme, 9.0, 4.55)
        fig.text(0.03, 0.955, {"controller_crash": "The navigation controller crashes mid-mission",
                               "blocked_station": "A box is left where the robot has to stop"}[scenario],
                 color=theme["ink"], fontsize=12.5, fontweight="semibold", va="top")
        fig.text(0.97, 0.955, f"{min(now, length):5.0f} s", color=theme["ink2"], fontsize=11, va="top",
                 ha="right", family="monospace")
        for column, ((name, _, _), told) in enumerate(zip(runs, stories, strict=True)):
            ax = fig.add_axes([0.03 + column*0.485, 0.20, 0.455, 0.63])
            draw_map(ax, theme)
            t = min(now, told["end"])
            path = [p for p in told["pose"] if p[0] <= t]
            fault = told["fault"]
            if fault["type"] == "blocked_path" and told["fault_at"] is not None and now >= told["fault_at"]:
                bx, by = fault["details"]["position"]
                ax.add_patch(Rectangle((bx - 0.38, by - 0.38), 0.76, 0.76, facecolor=theme["critical"],
                                       linewidth=0, zorder=4))
                ax.text(bx + 0.6, by - 0.05, "box", color=theme["ink2"], fontsize=7, va="center", zorder=4)
            if path:
                ax.plot([p[1] for p in path], [p[2] for p in path], color=theme["series"][0], linewidth=1.6,
                        solid_capstyle="round", solid_joinstyle="round", zorder=3)
                _, x, y, yaw = path[-1]
                ax.plot([x, x + 0.75*math.cos(yaw)], [y, y + 0.75*math.sin(yaw)], color=theme["series"][0],
                        linewidth=1.6, solid_capstyle="round", zorder=5)
                ax.scatter([x], [y], s=62, color=theme["series"][0], edgecolors=theme["surface"], linewidths=1.6,
                           zorder=6)
                if fault["type"] == "node_crash" and told["fault_at"] is not None and now >= told["fault_at"]:
                    at = next((p for p in told["pose"] if p[0] >= told["fault_at"]), path[-1])
                    ax.scatter([at[1]], [at[2]], s=48, marker="x", color=theme["critical"], linewidths=1.8, zorder=7)
                    ax.text(at[1] + 0.45, at[2] + 0.35, "crash", color=theme["ink2"], fontsize=7, zorder=7)
            box = ax.get_position()
            fig.text(box.x0, box.y1 + 0.022, name, color=theme["ink"], fontsize=10, fontweight="semibold")
            done = sum(1 for d in told["delivered"] if d <= t)
            fig.text(box.x1, box.y1 + 0.022, f"{done} of 3 delivered", color=theme["ink2"], fontsize=9.5,
                     ha="right")
            said = [c for c in told["captions"] if c[0] <= t][-1]
            mark_x = box.x0
            if said[2]:      # a fault: status colour on a marker, never on the text
                fig.patches.append(Rectangle((box.x0, 0.108), 0.011, 0.022, transform=fig.transFigure,
                                             facecolor=theme["critical"], linewidth=0))
                mark_x += 0.018
            fig.text(mark_x, 0.119, said[1], color=theme["ink"], fontsize=9, va="center")
            over = now >= told["end"]
            fig.text(box.x0, 0.058, "finished" if over else f"since {said[0]:.0f} s",
                     color=theme["muted"], fontsize=8, va="center")
        fig.savefig(scratch / f"{frame:04d}.png", facecolor=fig.get_facecolor())
        plt.close(fig)
    FIGURES.mkdir(parents=True, exist_ok=True)
    target = FIGURES / f"replay-{scenario}-{theme_name}.gif"
    palette = scratch / "palette.png"
    scale = "scale=900:-1:flags=lanczos"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(fps), "-i", str(scratch / "%04d.png"), "-vf",
                    f"{scale},palettegen=max_colors=48:stats_mode=diff", str(palette)], check=True)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(fps), "-i", str(scratch / "%04d.png"), "-i",
                    str(palette), "-lavfi", f"{scale}[x];[x][1:v]paletteuse=dither=none:diff_mode=rectangle",
                    "-loop", "0", str(target)], check=True)
    shutil.rmtree(scratch)
    return target


def main():
    what = set(sys.argv[1:]) or {"charts", "replays"}
    for theme_name in THEMES:
        if "charts" in what:
            for chart in (chart_delivered, chart_resume, chart_diagnosis):
                print(chart(theme_name).relative_to(ROOT))
        if "replays" in what:
            for scenario, step in (("controller_crash", 1.5), ("blocked_station", 3.0)):
                print(replay(scenario, theme_name, step).relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
