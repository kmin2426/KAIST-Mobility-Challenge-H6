import os
import json

# 1. 노드 경로 설정
NODES = [21, 51, 46, 40, 63, 34, 27, 31, 1, 3, 7, 9, 56, 59, 18, 21]

# 2. 경로 설정
# 'tool' 폴더 내에서 실행한다면 아래와 같이 설정됩니다.
WAYPOINT_DIR = os.path.join(os.path.dirname(__file__), "waypoint")
OUT_JSON = os.path.join(os.path.dirname(__file__), "new_path.json")

def read_xy_csv(path):
    pts = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line[0].isalpha():
                continue
            a = line.split(",")
            if len(a) < 2:
                continue
            # JSON에 넣기 편하게 딕셔너리 형태로 저장
            pts.append({
                "x": float(a[0].strip()),
                "y": float(a[1].strip())
            })
    return pts

all_pts = []
missing = []

# 3. 파일 순회하며 데이터 결합
for i in range(len(NODES) - 1):
    a = NODES[i]
    b = NODES[i+1]
    fname = f"{a}_{b}.csv"
    fpath = os.path.join(WAYPOINT_DIR, fname)
    
    if not os.path.exists(fpath):
        missing.append(fname)
        continue

    seg = read_xy_csv(fpath)

    # 중복 첫점 제거 로직 (좌표 비교)
    if all_pts and seg:
        if seg[0]["x"] == all_pts[-1]["x"] and seg[0]["y"] == all_pts[-1]["y"]:
            seg = seg[1:]

    all_pts.extend(seg)
    print(f"Appending {fname} ({len(seg)} pts)")

# 4. 에러 체크 및 JSON 저장
if missing:
    print("\n[ERROR] Missing files:")
    for m in missing:
        print(" -", m)
    # 어떤 파일이 없는지 확인 후 종료
    # raise SystemExit(1) 

with open(OUT_JSON, "w", encoding='utf-8') as f:
    json.dump(all_pts, f, indent=4)

print(f"\n[OK] Saved JSON: {OUT_JSON} (total {len(all_pts)} points)")