#!/usr/bin/env python3
"""
Gesture Visualizer

gesture_collector.py が生成した CSV を読み込み、
各ジェスチャーの x/y/z 加速度波形をグラフ表示する。

Usage:
    python3 gesture_visualizer.py [gesture_data.csv]
"""

import sys
import csv
import argparse
from collections import defaultdict
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

LABEL_COLORS = {
    "recording_start": "#2196F3",   # 青
    "other_motion":    "#FF9800",   # 橙
}
LABEL_NAMES = {
    "recording_start": "録音開始",
    "other_motion":    "その他",
}


def load_csv(path: str):
    gestures = defaultdict(lambda: {"label": "", "t": [], "x": [], "y": [], "z": []})
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            gid = int(row["gesture_id"])
            gestures[gid]["label"] = row["label"]
            gestures[gid]["t"].append(int(row["elapsed_ms"]))
            gestures[gid]["x"].append(float(row["x"]))
            gestures[gid]["y"].append(float(row["y"]))
            gestures[gid]["z"].append(float(row["z"]))
    return dict(sorted(gestures.items()))


def build_figure(gestures: dict, title: str):
    ids = list(gestures.keys())
    n = len(ids)
    ROW_HEIGHT = 2.5
    fig, axes = plt.subplots(n, 3, figsize=(13, ROW_HEIGHT * n), squeeze=False)
    fig.suptitle(title, fontsize=13, fontweight="bold")

    axes[0][0].set_title("X 軸", fontsize=11)
    axes[0][1].set_title("Y 軸", fontsize=11)
    axes[0][2].set_title("Z 軸", fontsize=11)

    seen_labels = set()
    legend_handles = []

    for row_idx, gid in enumerate(ids):
        g = gestures[gid]
        label = g["label"]
        color = LABEL_COLORS.get(label, "#9C27B0")
        t = g["t"]

        for col_idx, axis_data in enumerate([g["x"], g["y"], g["z"]]):
            ax = axes[row_idx][col_idx]
            ax.plot(t, axis_data, color=color, linewidth=1.2)
            ax.set_ylabel(f"#{gid}\n(m/s²)", fontsize=7)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)

            if label not in seen_labels:
                seen_labels.add(label)
                legend_handles.append(
                    plt.Line2D([0], [0], color=color, linewidth=2,
                               label=LABEL_NAMES.get(label, label))
                )

        for col_idx in range(3):
            if row_idx < n - 1:
                axes[row_idx][col_idx].set_xticklabels([])
            else:
                axes[row_idx][col_idx].set_xlabel("elapsed (ms)", fontsize=8)

    fig.legend(handles=legend_handles, loc="upper right", fontsize=9)
    fig.tight_layout(rect=[0, 0, 1.0, 0.97])
    return fig


def plot(gestures: dict, title: str):
    if not gestures:
        print("データがありません。")
        return

    root = tk.Tk()
    root.title(title)
    root.geometry("1100x800")

    # スクロール可能なキャンバス
    outer = tk.Frame(root)
    outer.pack(fill=tk.BOTH, expand=True)

    vbar = tk.Scrollbar(outer, orient=tk.VERTICAL)
    vbar.pack(side=tk.RIGHT, fill=tk.Y)

    canvas = tk.Canvas(outer, yscrollcommand=vbar.set)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    vbar.config(command=canvas.yview)

    inner = tk.Frame(canvas)
    canvas_window = canvas.create_window((0, 0), window=inner, anchor="nw")

    fig = build_figure(gestures, title)
    fc = FigureCanvasTkAgg(fig, master=inner)
    fc.draw()
    fc.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def on_configure(event):
        canvas.configure(scrollregion=canvas.bbox("all"))
        canvas.itemconfig(canvas_window, width=event.width)

    inner.bind("<Configure>", on_configure)
    canvas.bind("<Configure>", lambda e: canvas.itemconfig(canvas_window, width=e.width))

    # マウスホイールスクロール
    def on_mousewheel(event):
        canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    root.bind_all("<MouseWheel>", on_mousewheel)
    # macOS trackpad
    root.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
    root.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

    root.mainloop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", default="gesture_data.csv",
                        help="入力CSVファイル (default: gesture_data.csv)")
    args = parser.parse_args()

    try:
        gestures = load_csv(args.csv)
    except FileNotFoundError:
        print(f"ERROR: {args.csv} が見つかりません。")
        sys.exit(1)

    labels = {g["label"] for g in gestures.values()}
    counts = {lb: sum(1 for g in gestures.values() if g["label"] == lb) for lb in labels}
    print(f"読み込み: {len(gestures)} ジェスチャー")
    for lb, cnt in counts.items():
        print(f"  {LABEL_NAMES.get(lb, lb)}: {cnt} 件")

    title = f"Gesture IMU Data — {args.csv}"
    plot(gestures, title)


if __name__ == "__main__":
    main()
