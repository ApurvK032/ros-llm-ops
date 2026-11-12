"""Draw the figures and replays in docs/figures from the evidence committed under docs/evidence.

    python3 scripts/make_figures.py            # everything, light and dark
    python3 scripts/make_figures.py charts     # only the three single-run charts
    python3 scripts/make_figures.py batch      # only the charts of the repeated runs
    python3 scripts/make_figures.py replays    # only the animated replays (needs ffmpeg)
    python3 scripts/make_figures.py belief     # only the replay of believed against true pose

Nothing here is typed in by hand: every number is read from a result.json, a journal or an evidence file, so the
figures change only when the evidence does. Needs matplotlib; the replays also need ffmpeg.
"""

import json
import math
import os
import shutil
import statistics
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
sys.path.insert(0, str(ROOT))
from warehouse_agent.batch import collect  # noqa: E402

EVIDENCE, FIGURES = ROOT / "docs/evidence", ROOT / "docs/figures"
BATCH = Path(os.environ.get("BATCH_DIR", EVIDENCE / "batch"))
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


# --- charts 4 and 5: the repeated runs ---------------------------------------------------------------------------

GROUPS = [
    ("Faults it was built on", [("bad_parameter", "Speed limit set to zero"),
                                ("controller_crash", "Controller crashed"), ("lidar_dropout", "Lidar stopped"),
                                ("localization_lost", "Localization lost"), ("blocked_station", "Station blocked")]),
    ("Faults it had not seen", [("planner_crash", "Planner crashed"), ("amcl_crash", "Localization node crashed"),
                                ("lidar_frozen", "Lidar stuck on one scan"), ("double_fault", "Two faults at once"),
                                ("distractor_lidar", "Harmless change, then lidar stopped")]),
    ("Nothing to fix", [("nominal", "No fault"), ("speed_limit", "Speed limit lowered on purpose"),
                        ("blocked_aisle", "Aisle blocked, another route open")]),
]
WHO = [("none", "Supervisor\nalone"), ("rules", "Fixed\nplaybook"), ("qwen3.5:9b", "qwen3.5:9b\ndecides"),
       ("qwen3.5:4b", "qwen3.5:4b\ndecides")]


def who_colour(theme, decider):
    return theme["muted"] if decider == "none" else theme["series"][[d for d, _ in WHO].index(decider) - 1]


def batch_runs():
    """Valid runs by (scenario, decider), in the order they were made."""
    cells = {}
    for run in collect(BATCH):
        if run["valid"]:
            cells.setdefault((run["scenario"], run["decider"]), []).append(run)
    return cells


def chart_batch_success(theme_name):
    theme, cells = THEMES[theme_name], batch_runs()
    rows = sum(len(scenarios) for _, scenarios in GROUPS)
    height = 1.9 + 0.33*(rows + 1) + 0.36*len(GROUPS)
    fig = figure(theme, 8.6, height)
    ax = fig.add_axes([0.03, 0.03, 0.94, 1 - 1.15/height - 0.03])
    ax.set_facecolor(theme["surface"])
    ax.axis("off")
    # One unit is one row; columns are placed in axes fractions so labels and squares share one grid.
    total = rows + len(GROUPS)*1.1 + 3.1
    ax.set_xlim(0, 1)
    ax.set_ylim(0, total)
    columns = [0.36 + 0.16*i for i in range(len(WHO))]
    side = 0.52                                             # square height in row units
    width = side*(ax.bbox.height/total)/ax.bbox.width       # the same length across, so squares are square
    gap = width*0.3
    y = total - 0.55
    for x, (_, name) in zip(columns, WHO, strict=True):
        ax.text(x, y, name, color=theme["ink2"], fontsize=8.5, va="center", ha="left", linespacing=1.25)
    y -= 0.75
    tally = {decider: [0, 0] for decider, _ in WHO}
    for group, scenarios in GROUPS:
        y -= 1.1
        ax.text(0, y + 0.1, group.upper(), color=theme["muted"], fontsize=7.6, va="center", fontweight="semibold")
        ax.plot([0, 1], [y - 0.42, y - 0.42], color=theme["grid"], linewidth=0.8)
        for scenario, label in scenarios:
            y -= 1
            ax.text(0, y, label, color=theme["ink"], fontsize=9.5, va="center")
            for x, (decider, _) in zip(columns, WHO, strict=True):
                runs = cells.get((scenario, decider), [])
                colour = who_colour(theme, decider)
                for index, run in enumerate(runs):
                    ax.add_patch(Rectangle((x + index*(width + gap), y - side/2), width, side, linewidth=1.3,
                                           edgecolor=colour, facecolor=colour if run["success"] else "none"))
                tally[decider][0] += sum(bool(run["success"]) for run in runs)
                tally[decider][1] += len(runs)
    y -= 1.25
    ax.plot([0, 1], [y + 0.6, y + 0.6], color=theme["axis"], linewidth=0.8)
    ax.text(0, y, "Missions that ended as they should", color=theme["ink"], fontsize=9.5, va="center",
            fontweight="semibold")
    for x, (decider, _) in zip(columns, WHO, strict=True):
        ax.text(x, y, f"{tally[decider][0]} of {tally[decider][1]}", color=theme["ink"], fontsize=9.5, va="center",
                fontweight="semibold")
    key = fig.add_axes([0.66, 1 - 0.47/height, 0.32, 0.26/height])
    key.axis("off")
    key.set_xlim(0, 1)
    key.set_ylim(0, 1)
    unit = 0.7*(key.bbox.height/key.bbox.width)
    for x, filled, text in ((0.0, True, "ended as it should"), (0.66, False, "did not")):
        key.add_patch(Rectangle((x, 0.15), unit, 0.7, linewidth=1.3, edgecolor=theme["ink2"],
                                facecolor=theme["ink2"] if filled else "none"))
        key.text(x + unit + 0.025, 0.5, text, color=theme["ink2"], fontsize=8.5, va="center")
    fig.text(0.03, 1 - 0.2/height, "Thirteen situations, each run several times", color=theme["ink"], fontsize=13,
             fontweight="semibold", va="top")
    fig.text(0.03, 1 - 0.56/height,
             "One square is one simulated mission. It ended as it should when every parcel that could be delivered\n"
             "was, confirmed by the simulator's true pose, with nothing left on the robot.",
             color=theme["ink2"], fontsize=9, va="top", linespacing=1.35)
    return save(fig, "batch-success", theme_name)


def chart_batch_time(theme_name):
    theme, cells = THEMES[theme_name], batch_runs()
    scenarios = [(scenario, label) for _, group in GROUPS for scenario, label in group
                 if any(run["success"] for (name, _), runs in cells.items() if name == scenario for run in runs)]
    fig = figure(theme, 8.6, 1.5 + 0.42*len(scenarios))
    bottom, top = 0.55/fig.get_figheight(), 1 - 1.15/fig.get_figheight()
    ax = fig.add_axes([0.36, bottom, 0.60, top - bottom])
    clean(ax, theme)
    lanes = {decider: 0.27 - 0.18*index for index, (decider, _) in enumerate(WHO)}
    markers = {"none": "o", "rules": "o", "qwen3.5:9b": "s", "qwen3.5:4b": "D"}
    # The axis is sized for the missions an agent handled; a supervisor left alone for eight minutes is written
    # at the edge instead of flattening everything else.
    longest = max(run["mission_seconds"] for (_, decider), runs in cells.items() if decider != "none"
                  for run in runs if run["success"])
    limit = math.ceil(longest/20)*20 + 5
    for row, (scenario, label) in enumerate(scenarios):
        y = len(scenarios) - 1 - row
        for decider, _ in WHO:
            times = [run["mission_seconds"] for run in cells.get((scenario, decider), []) if run["success"]]
            if not times:
                continue
            colour, lane = who_colour(theme, decider), y + lanes[decider]
            if statistics.median(times) > limit:
                ax.text(limit - 2, lane, f"supervisor alone, off this scale: {statistics.median(times):.0f} s "
                        f"({min(times):.0f} to {max(times):.0f})", color=theme["ink2"], fontsize=8.5, ha="right",
                        va="center")
                continue
            ax.plot([min(times), max(times)], [lane, lane], color=colour, linewidth=2, solid_capstyle="round",
                    zorder=2)
            ax.scatter([statistics.median(times)], [lane], s=46, color=colour, marker=markers[decider],
                       edgecolors=theme["surface"], linewidths=1.2, zorder=3)
        fig.text(0.03, bottom + (top - bottom)*(y + 0.5)/len(scenarios), label, color=theme["ink"], fontsize=9.5,
                 va="center")
    ax.set_ylim(-0.5, len(scenarios) - 0.5)
    ax.set_xlim(100, limit)
    ax.set_yticks([])
    ax.set_xlabel("Mission time in seconds, for the missions that ended as they should: median, with the range",
                  color=theme["ink2"], fontsize=9, labelpad=8)
    handles = [plt.Line2D([], [], marker=markers[decider], linestyle="-", linewidth=2, markersize=6.5,
                          color=who_colour(theme, decider), label=name.replace("\n", " "))
               for decider, name in WHO]
    legend = fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(0.975, 1 - 0.5/fig.get_figheight()),
                        frameon=False, ncol=4, fontsize=8.5, handletextpad=0.4, columnspacing=1.4)
    for text in legend.get_texts():
        text.set_color(theme["ink2"])
    fig.text(0.03, 1 - 0.2/fig.get_figheight(), "What a recovery costs in mission time", color=theme["ink"],
             fontsize=13, fontweight="semibold", va="top")
    return save(fig, "batch-time", theme_name)


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
               "blocked_path": "Fault: a box is left on the PICK B stopping pose",
               "sensor_dropout": "Fault: the lidar keeps sending its last scan"
               if fault.get("details", {}).get("mode") == "freeze" else "Fault: the lidar stops sending scans"}
    if fault_at is not None:
        captions.append((fault_at, wording.get(fault["type"], "Fault injected"), True))
    # Runs with a true-pose check: a parcel is confirmed when the robot really was at both of its stations.
    checked = (result.get("pose_check") or {}).get("transfers")
    at_station = {(c["parcel"], c["kind"]): c["verified"] for c in checked or []}
    delivered, confirmed, deferred_by_supervisor = [], [], 0
    for event in events:
        t = event["monotonic"] - origin
        kind = event["type"]
        if kind == "cargo_drop":
            delivered.append(t)
            if at_station.get((event["parcel"], "pickup")) and at_station.get((event["parcel"], "drop")):
                confirmed.append(t)
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
            if checked is not None and len(confirmed) != total:
                captions.append((t, f"Reported: {total} of 3 delivered. Confirmed by the true pose: "
                                 f"{len(confirmed)}", True))
            else:
                captions.append((t, f"Mission over: {total} of 3 delivered, in {t:.0f} s", False))
    for incident in (result.get("recovery") or {}).get("incidents", []):
        acted = [a for a in incident["actions"] if a["permitted"] and a["action"] != "escalate"]
        restarts = {"controller_server": "the controller", "planner_server": "the planner",
                    "amcl": "localization", "lidar_driver": "the lidar driver", "lidar": "the lidar driver"}
        names = {"restore_parameter": "restores the parameter", "relocalize": "relocalizes", "defer_parcel": None,
                 "clear_costmaps": None}
        doing = ["restarts " + restarts.get(a["target"], a["target"]) if a["action"] == "restart_process"
                 else names.get(a["action"]) for a in acted]
        doing = [text for text in doing if text]
        if doing:
            captions.append((incident["detected_at_s"], "Agent: pauses, diagnoses, " + " and ".join(doing), False))
    end = max(t for t, _, _ in captions)
    return {"captions": sorted(captions), "delivered": delivered, "confirmed": confirmed, "fault_at": fault_at,
            "end": end, "pose": [p for p in track if p[0] >= 0], "fault": fault}


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


def encode(scratch, target, fps):
    """Frames in a scratch directory to one looping GIF with a shared palette; the scratch directory is removed."""
    FIGURES.mkdir(parents=True, exist_ok=True)
    palette = scratch / "palette.png"
    scale = "scale=900:-1:flags=lanczos"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(fps), "-i", str(scratch / "%04d.png"), "-vf",
                    f"{scale},palettegen=max_colors=48:stats_mode=diff", str(palette)], check=True)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(fps), "-i", str(scratch / "%04d.png"), "-i",
                    str(palette), "-lavfi", f"{scale}[x];[x][1:v]paletteuse=dither=none:diff_mode=rectangle",
                    "-loop", "0", str(target)], check=True)
    shutil.rmtree(scratch)
    return target


def replay_belief(theme_name, step=2.0, fps=10):
    """The frozen lidar: where the robot is beside where it thinks it is, alone and with the agent."""
    theme = THEMES[theme_name]
    cells = BATCH / "lidar_frozen"
    runs = [("Supervisor alone", cells / "none/run-03"), ("With the recovery agent", cells / "rules/run-01")]
    tracks = [load(run / "track.json") for _, run in runs]
    stories = [story(run, track["pose"]) for (_, run), track in zip(runs, tracks, strict=True)]
    length = max(s["end"] for s in stories)
    times = [i*step for i in range(int(length/step) + 2)]
    real, believed = theme["series"][0], theme["series"][1]
    scratch = Path(tempfile.mkdtemp(prefix="replay-"))
    for frame, now in enumerate(times + [times[-1]]*int(3.5*fps)):
        fig = figure(theme, 9.0, 4.75)
        fig.text(0.03, 0.958, "The lidar gets stuck on one scan", color=theme["ink"], fontsize=12.5,
                 fontweight="semibold", va="top")
        fig.text(0.97, 0.958, f"{min(now, length):5.0f} s", color=theme["ink2"], fontsize=11, va="top", ha="right",
                 family="monospace")
        # One key for both panels: identity is carried by line style and marker as well as colour.
        key = fig.add_axes([0.40, 0.905, 0.46, 0.05])
        key.axis("off")
        key.set_xlim(0, 1)
        key.set_ylim(0, 1)
        key.plot([0.0, 0.07], [0.5, 0.5], color=real, linewidth=1.8, solid_capstyle="round")
        key.scatter([0.07], [0.5], s=40, color=real, edgecolors=theme["surface"], linewidths=1.2, zorder=3)
        key.text(0.095, 0.5, "where the robot is", color=theme["ink2"], fontsize=8.5, va="center")
        key.plot([0.47, 0.54], [0.5, 0.5], color=believed, linewidth=1.5, linestyle=(0, (2.5, 2)))
        key.scatter([0.54], [0.5], s=40, facecolors="none", edgecolors=believed, linewidths=1.5, zorder=3)
        key.text(0.565, 0.5, "where it thinks it is", color=theme["ink2"], fontsize=8.5, va="center")
        for column, ((name, _), told, track) in enumerate(zip(runs, stories, tracks, strict=True)):
            ax = fig.add_axes([0.03 + column*0.485, 0.19, 0.455, 0.605])
            draw_map(ax, theme)
            t = min(now, told["end"])
            thought = [p for p in told["pose"] if p[0] <= t]
            actual = [p for p in track["truth"] if 0 <= p[0] <= t]
            gap = math.dist(thought[-1][1:3], actual[-1][1:3]) if thought and actual else 0.0
            if thought:
                ax.plot([p[1] for p in thought], [p[2] for p in thought], color=believed, linewidth=1.4,
                        linestyle=(0, (2.5, 2)), zorder=3)
            if actual:
                ax.plot([p[1] for p in actual], [p[2] for p in actual], color=real, linewidth=1.7,
                        solid_capstyle="round", solid_joinstyle="round", zorder=4)
                _, x, y, yaw = actual[-1]
                ax.plot([x, x + 0.75*math.cos(yaw)], [y, y + 0.75*math.sin(yaw)], color=real, linewidth=1.7,
                        solid_capstyle="round", zorder=6)
                ax.scatter([x], [y], s=62, color=real, edgecolors=theme["surface"], linewidths=1.6, zorder=7)
            if thought and gap >= 0.5:      # apart: the belief gets its own marker, hollow
                ax.scatter([thought[-1][1]], [thought[-1][2]], s=58, facecolors=theme["surface"],
                           edgecolors=believed, linewidths=1.7, zorder=5)
            box = ax.get_position()
            fig.text(box.x0, box.y1 + 0.022, name, color=theme["ink"], fontsize=10, fontweight="semibold")
            reported = sum(1 for d in told["delivered"] if d <= t)
            really = sum(1 for d in told["confirmed"] if d <= t)
            fig.text(box.x1, box.y1 + 0.022, f"reported {reported}  \u00b7  confirmed {really}",
                     color=theme["ink2"], fontsize=9.5, ha="right")
            said = [c for c in told["captions"] if c[0] <= t][-1]
            mark_x = box.x0
            if said[2]:      # a fault: status colour on a marker, never on the text
                fig.patches.append(Rectangle((box.x0, 0.101), 0.011, 0.022, transform=fig.transFigure,
                                             facecolor=theme["critical"], linewidth=0))
                mark_x += 0.018
            fig.text(mark_x, 0.112, said[1], color=theme["ink"], fontsize=9, va="center")
            fig.text(box.x0, 0.052, f"pose estimate {gap:.1f} m from the robot" if gap >= 0.5
                     else "pose estimate on the robot", color=theme["muted"], fontsize=8, va="center")
        fig.savefig(scratch / f"{frame:04d}.png", facecolor=fig.get_facecolor())
        plt.close(fig)
    return encode(scratch, FIGURES / f"replay-lidar_frozen-{theme_name}.gif", fps)


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
    return encode(scratch, FIGURES / f"replay-{scenario}-{theme_name}.gif", fps)


def main():
    what = set(sys.argv[1:]) or {"charts", "replays"}
    for theme_name in THEMES:
        if "charts" in what:
            for chart in (chart_delivered, chart_resume, chart_diagnosis):
                print(chart(theme_name).relative_to(ROOT))
        if "batch" in what or not sys.argv[1:]:
            for chart in (chart_batch_success, chart_batch_time):
                print(chart(theme_name).relative_to(ROOT))
        if "replays" in what:
            for scenario, step in (("controller_crash", 1.5), ("blocked_station", 3.0)):
                print(replay(scenario, theme_name, step).relative_to(ROOT))
        if "replays" in what or "belief" in what:
            print(replay_belief(theme_name).relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
