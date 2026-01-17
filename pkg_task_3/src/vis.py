#!/usr/bin/env python3
# vis_paths_vscode.py
# VSCode에서 그냥 실행: python3 vis_paths_vscode.py
# - path3_1~4.json + zone/hv csv 점 표시
# - 사지/회전교차로 박스 표시
# - 점 클릭하면 (파일/인덱스/x/y) 화면+터미널 출력

import os
import json
import csv
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# =========================
# 설정 (원하면 여기만 건드려)
# =========================
LABEL_EVERY = 80          # path 점 라벨 몇 개마다 찍을지 (너무 작으면 난장판)
SHOW_ZONE_LABEL = False   # zone/hv csv 점 라벨까지 찍을지 (기본 False)
POINT_SIZE_PATH = 8
POINT_SIZE_CSV  = 18

# 너가 준 박스 2개
RAW_SLOW_ZONES = [
    (-0.4, -4.3,  1.8, -1.8), # 사지교차로 범위
    ( 0.4,  2.7,  1.4, -1.4), # 회전교차로 범위
]

# =========================
# IO
# =========================
def load_path_json(json_file):
    if not os.path.exists(json_file):
        return []
    with open(json_file, "r") as f:
        data = json.load(f)
    xs = data.get("x") or data.get("X")
    ys = data.get("y") or data.get("Y")
    if not xs or not ys:
        return []
    pts = []
    for x, y in zip(xs, ys):
        try:
            pts.append((float(x), float(y)))
        except:
            pass
    return pts

def load_csv_points(csv_file, has_header=True):
    if not os.path.exists(csv_file):
        return []
    pts = []
    with open(csv_file, "r") as f:
        r = csv.reader(f)
        if has_header:
            next(r, None)
        for row in r:
            if not row or len(row) < 2:
                continue
            try:
                pts.append((float(row[0]), float(row[1])))
            except:
                continue
    return pts

# =========================
# Plot helpers
# =========================
def normalize_box(x1, x2, y1, y2):
    return (min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2))

def draw_box(ax, box, label):
    x_min, x_max, y_min, y_max = box
    rect = Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                     fill=False, linewidth=2)
    ax.add_patch(rect)
    ax.text(x_min, y_max, f" {label}", va="bottom", ha="left")

def annotate_sparse(ax, pts, every, prefix, fontsize=7):
    if every <= 0:
        return
    for i in range(0, len(pts), every):
        x, y = pts[i]
        ax.text(x, y, f"{prefix}{i}\n({x:.2f},{y:.2f})", fontsize=fontsize)

def add_click_picker(fig, ax, scatter_to_meta):
    """
    scatter 점 클릭하면 터미널 + 그래프에 표시
    scatter_to_meta: {scatter_obj: {"name":..., "pts":[(x,y),...]}}
    """
    ann = ax.annotate(
        "", xy=(0, 0), xytext=(10, 10), textcoords="offset points",
        bbox=dict(boxstyle="round", fc="w"),
        arrowprops=dict(arrowstyle="->")
    )
    ann.set_visible(False)

    def on_pick(event):
        sc = event.artist
        if sc not in scatter_to_meta:
            return
        meta = scatter_to_meta[sc]
        idx = int(event.ind[0])
        x, y = meta["pts"][idx]
        msg = f"[CLICK] {meta['name']}  idx={idx}  x={x:.3f}  y={y:.3f}"
        print(msg)

        ann.xy = (x, y)
        ann.set_text(msg)
        ann.set_visible(True)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("pick_event", on_pick)

# =========================
# Main
# =========================
def main():
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    PATH_DIR = os.path.join(BASE_DIR, "path")

    slow_boxes = [normalize_box(*b) for b in RAW_SLOW_ZONES]

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title("Task3 Paths (VSCode)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    # Boxes
    draw_box(ax, slow_boxes[0], "4-Way Box")
    draw_box(ax, slow_boxes[1], "Roundabout Box")

    scatter_to_meta = {}

    # ---------- paths ----------
    for i in range(1, 5):
        jf = os.path.join(PATH_DIR, f"path3_{i}.json")
        pts = load_path_json(jf)
        if not pts:
            print(f"[WARN] missing/empty: {jf}")
            continue

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        ax.plot(xs, ys, linewidth=1, label=f"path3_{i}")
        sc = ax.scatter(xs, ys, s=POINT_SIZE_PATH, picker=True, label=f"path3_{i}_pts")
        scatter_to_meta[sc] = {"name": f"path3_{i}.json", "pts": pts}

        annotate_sparse(ax, pts, every=LABEL_EVERY, prefix=f"C{i}_", fontsize=7)

        # start/end
        ax.scatter([xs[0]], [ys[0]], s=60, marker="o")
        ax.text(xs[0], ys[0], f" START{i}", fontsize=9)
        ax.scatter([xs[-1]], [ys[-1]], s=60, marker="x")
        ax.text(xs[-1], ys[-1], f" END{i}", fontsize=9)

    # ---------- csv zones / hv ----------
    csv_candidates = [
        # start zones
        "path3_1_zone.csv", "path3_2_zone.csv", "path3_3_zone.csv", "path3_4_zone.csv",
        # out zones
        "path3_1_out_zone.csv", "path3_2_out_zone.csv",
        # hv paths
        "path_hv_1_1.csv", "path_hv_1_2.csv", "path_hv_2_1.csv", "path_hv_2_2.csv",
        "path_hv_3_1.csv", "path_hv_3_2.csv",
    ]

    for name in csv_candidates:
        f = os.path.join(PATH_DIR, name)
        pts = load_csv_points(f, has_header=True)
        if not pts:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        sc = ax.scatter(xs, ys, s=POINT_SIZE_CSV, marker=".", alpha=0.9, picker=True, label=name)
        scatter_to_meta[sc] = {"name": name, "pts": pts}

        if SHOW_ZONE_LABEL:
            annotate_sparse(ax, pts, every=max(1, LABEL_EVERY // 2), prefix=f"{name}_", fontsize=6)

    add_click_picker(fig, ax, scatter_to_meta)

    ax.legend(loc="best", fontsize=8)
    plt.show()


if __name__ == "__main__":
    main()
