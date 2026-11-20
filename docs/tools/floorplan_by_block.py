#!/usr/bin/env python3
"""Colour every placed standard cell of a LibreLane DEF by the RTL block it belongs to.

LibreLane flattens the design and renames instances (_12345_), but net names keep
the hierarchy (soc.u_cpu.ir[3], ...).  Each instance is assigned to the block named
by the nets on its output pin (falling back to its input nets).

usage: floorplan_by_block.py <final.def> <out.png> <cells.lef> [title]
"""
import collections
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PatchCollection  # noqa: E402

BLOCKS = [
    ("soc.u_cpu", "RV32E CPU", "#4C78A8"),
    ("soc.u_foc", "FOC engine", "#E45756"),
    ("soc.u_spi", "SPI bus (flash/ADC/IMU)", "#54A24B"),
    ("soc.u_link", "host link (UART+CRC)", "#B279A2"),
    ("soc.u_hall", "hall estimator", "#F58518"),
    ("soc.u_pwm", "PWM / gate drive", "#EECA3B"),
    ("soc.ram", "128 B RAM", "#72B7B2"),
    ("soc.", "registers / bus", "#9D755D"),
]
OUT_PINS = {"Q", "X", "Y", "Q_N", "COUT", "SUM", "HI", "LO"}


def block_of(net):
    n = net.replace("\\", "")
    for pre, name, _ in BLOCKS:
        if n.startswith(pre):
            return name
    return None


def main(defp, outp, lefp, title="tt_um_apr0t0_foc_mcu"):
    txt = open(defp).read()
    dbu = int(re.search(r"UNITS DISTANCE MICRONS (\d+)", txt).group(1))
    die = [int(v) / dbu for v in re.search(r"DIEAREA \( (\d+) (\d+) \) \( (\d+) (\d+) \)", txt).groups()]
    comps = {}
    sec = txt[txt.index("\nCOMPONENTS"):txt.index("END COMPONENTS")]
    for m in re.finditer(r"- (\S+) (\S+) \+ (?:PLACED|FIXED) \( (-?\d+) (-?\d+) \) (\S+)", sec):
        inst, cell, x, y, _ = m.groups()
        comps[inst] = (cell, int(x) / dbu, int(y) / dbu)
    nets = txt[txt.index("\nNETS"):txt.index("END NETS")]
    out_vote = collections.defaultdict(collections.Counter)
    in_vote = collections.defaultdict(collections.Counter)
    net_insts = []
    for m in re.finditer(r"- (\S+)((?:\s+\( \S+ \S+ \))+)", nets):
        conns = [c for c in re.findall(r"\( (\S+) (\S+) \)", m.group(2)) if c[0] != "PIN"]
        net_insts.append([i for i, _ in conns])
        b = block_of(m.group(1))
        if not b:
            continue
        for inst, pin in conns:
            (out_vote if pin in OUT_PINS else in_vote)[inst][b] += 1
    assign = {}
    for inst in comps:
        v = out_vote.get(inst) or in_vote.get(inst)
        if v:
            assign[inst] = v.most_common(1)[0][0]
    # propagate through low-fanout anonymous nets (skips clock / reset trees)
    for _ in range(6):
        votes = collections.defaultdict(collections.Counter)
        for insts in net_insts:
            if len(insts) > 24:
                continue
            bs = [assign[i] for i in insts if i in assign]
            if not bs:
                continue
            for i in insts:
                if i not in assign:
                    for bb in bs:
                        votes[i][bb] += 1
        if not votes:
            break
        for i, v in votes.items():
            assign[i] = v.most_common(1)[0][0]
    width = {}
    for m in re.finditer(r"MACRO (\S+)\s.*?SIZE ([\d.]+) BY ([\d.]+)", open(lefp).read(), re.S):
        width[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    patches = collections.defaultdict(list)
    counts = collections.Counter()
    for inst, (cell, x, y) in comps.items():
        if any(k in cell for k in ("fill", "tap", "decap", "PHY")):
            continue
        b = assign.get(inst, "buffers / clock / misc")
        w, h = width.get(cell, (2.76, 2.72))
        patches[b].append(mpatches.Rectangle((x, y), w, h))
        counts[b] += 1
    colors = {name: c for _, name, c in BLOCKS}
    colors["buffers / clock / misc"] = "#BAB0AC"
    fig, ax = plt.subplots(figsize=(16, 16 * (die[3] - die[1]) / (die[2] - die[0]) + 1.2), dpi=110)
    for b, ps in patches.items():
        ax.add_collection(PatchCollection(ps, facecolor=colors[b], edgecolor="none"))
    ax.set_xlim(die[0], die[2])
    ax.set_ylim(die[1], die[3])
    ax.set_aspect("equal")
    ax.set_xlabel("µm")
    ax.set_title(f"{title}: placed standard cells by block ({sum(counts.values())} cells, "
                 f"die {die[2]-die[0]:.0f} × {die[3]-die[1]:.0f} µm)")
    handles = [mpatches.Patch(color=colors[b], label=f"{b} ({counts[b]})") for b in colors if counts[b]]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=4, frameon=False)
    fig.tight_layout()
    fig.savefig(outp, bbox_inches="tight")
    print(dict(counts))


if __name__ == "__main__":
    main(*sys.argv[1:])
