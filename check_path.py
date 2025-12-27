import json
import matplotlib.pyplot as plt
import numpy as np

# 1. 파일 읽기
filename = "path_new.json"

try:
    with open(filename, 'r') as f:
        data = json.load(f)
        path_x = data['X']
        path_y = data['Y']
        print(f"Path Loaded: {len(path_x)} points")
except Exception as e:
    print(f"Error loading file: {e}")
    exit()

# 2. 시각화 설정
plt.figure(figsize=(10, 10))

# 3. 경로 그리기 (순서에 따라 색상이 변함: 보라 -> 청록 -> 노랑)
# 이렇게 하면 경로가 끊기거나 되돌아가는지 바로 알 수 있습니다.
sc = plt.scatter(path_x, path_y, c=range(len(path_x)), cmap='viridis', s=5, label='Waypoints')
plt.colorbar(sc, label='Index Progress (Start -> End)')

# 4. 시작점(초록색 원)과 끝점(빨간색 X) 표시
plt.plot(path_x[0], path_y[0], 'go', markersize=15, label='START', markeredgewidth=2)
plt.plot(path_x[-1], path_y[-1], 'rx', markersize=15, label='END', markeredgewidth=2)

# 5. 그래프 꾸미기
plt.title(f"Path Visualization: {filename}")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis('equal') # 실제 비율 유지 (찌그러짐 방지)
plt.grid(True)
plt.legend()

# 6. 저장 및 알림
output_file = "path_check.png"
plt.savefig(output_file)
print(f"이미지가 저장되었습니다: {output_file}")
print("VS Code 파일 목록에서 path_check.png를 클릭해서 확인해보세요.")