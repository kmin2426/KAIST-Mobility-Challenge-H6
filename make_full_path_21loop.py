import os

NODES = [21, 51, 46, 40, 63, 34, 27, 31, 1, 3, 7, 9, 56, 59, 18, 21]

WAYPOINT_DIR = os.path.join(os.path.dirname(__file__), "waypoint")
OUT_CSV = os.path.join(os.path.dirname(__file__), "full_path_21loop.csv")

def read_xy_csv(path):
    pts = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # 헤더(X,Y) 있으면 건너뜀
            if line[0].isalpha():
                continue
            a = line.split(",")
            if len(a) < 2:
                continue
            x = float(a[0].strip())
            y = float(a[1].strip())
            pts.append((x, y))
    return pts

all_pts = []
missing = []

for i in range(len(NODES) - 1):
    a = NODES[i]
    b = NODES[i+1]
    fname = f"{a}_{b}.csv"
    fpath = os.path.join(WAYPOINT_DIR, fname)
    if not os.path.exists(fpath):
        missing.append(fname)
        continue

    seg = read_xy_csv(fpath)

    # 이어붙일 때 "중복 첫점" 제거 (튀는 거 방지)
    if all_pts and seg:
        if seg[0] == all_pts[-1]:
            seg = seg[1:]

    all_pts.extend(seg)
    print(f"Appending {fname} ({len(seg)} pts)")

if missing:
    print("\n[ERROR] Missing files:")
    for m in missing:
        print(" -", m)
    raise SystemExit(1)

with open(OUT_CSV, "w") as f:
    for x, y in all_pts:
        f.write(f"{x},{y}\n")

print(f"\n[OK] Saved: {OUT_CSV}  (total {len(all_pts)} points)")

