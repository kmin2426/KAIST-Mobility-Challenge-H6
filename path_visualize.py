#!/usr/bin/env python3
import json
from typing import List, Tuple

import matplotlib.pyplot as plt


FILES = [
    "/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/lane_change.json",
    "/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/not_lane_change.json",
    "/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_1.json",
    "/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_2.json",
    "/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_3.json",
    "/home/dongminkim/Desktop/Mobility_Challenge_Simulator/src/central_control/path/path_2_4.json",

]

# 옵션
SHOW_POINTS = True   # 점도 같이 보고 싶으면 True
POINT_SIZE = 8       # 점 크기


def load_xy(path: str) -> Tuple[List[float], List[float]]:
    with open(path, "r") as f:
        data = json.load(f)

    # {"X":[...], "Y":[...]} 형식 고정
    if "X" not in data or "Y" not in data:
        raise KeyError(f"{path}: keys must be 'X' and 'Y'")

    x = [float(v) for v in data["X"]]
    y = [float(v) for v in data["Y"]]

    if len(x) != len(y):
        raise ValueError(f"{path}: len(X)={len(x)} != len(Y)={len(y)}")
    if len(x) < 2:
        raise ValueError(f"{path}: need at least 2 points")

    return x, y


def main():
    plt.figure(figsize=(10, 8))

    for p in FILES:
        x, y = load_xy(p)
        name = p.split("/")[-1]

        # "점들을 이은 하나의 차선" = 그냥 plot으로 순서대로 연결
        plt.plot(x, y, linewidth=2, label=name)

        if SHOW_POINTS:
            plt.scatter(x, y, s=POINT_SIZE)

    plt.title("Lane polylines (2D)")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis("equal")   # 왜곡 없이 보기
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
