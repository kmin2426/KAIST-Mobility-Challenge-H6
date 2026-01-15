#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt

PATH_FILES = [
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path_hv_1_1.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path_hv_1_2.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path_hv_2_1.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path_hv_2_2.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path3_1_zone.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path3_2_zone.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path3_3_zone.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path3_4_zone.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path3_1_out_zone.csv',
    '/home/dongminkim/Desktop/KAIST-Mobility-Challenge-H6/task3/path/path3_2_out_zone.csv',
]

LABELS = [
    "HV 1-1",
    "HV 1-2",
    "HV 2-1",
    "HV 2-2",
    "CAV1 stop",
    "CAV2 stop",
    "CAV3 stop",
    "CAV4 stop",
    "CAV1 stop (out)",
    "CAV2 stop (out)",
]

def load_csv_xy(path):
    xs, ys = [], []
    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
            except:
                continue
            xs.append(x)
            ys.append(y)
    return xs, ys


def main():
    plt.figure(figsize=(10, 10))

    for path, label in zip(PATH_FILES, LABELS):
        xs, ys = load_csv_xy(path)
        plt.plot(xs, ys, marker='o', markersize=3, linewidth=1, label=label)

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title("Trigger Zones & Stop Zones Visualization")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.show()


if __name__ == "__main__":
    main()
