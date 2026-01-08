import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import sys

def create_lane_csv(filename, key_points):
    """
    key_points: [(x, y), (x, y), ...] 
    사용자가 지정한 점들을 반드시 통과하는 부드러운 곡선을 생성
    """
    # 1. 데이터 분리
    x = np.array([p[0] for p in key_points])
    y = np.array([p[1] for p in key_points])

    # 2. 거리 누적 계산 (곡선 길이에 따라 파라미터화)
    # 점들 사이의 거리를 계산해서 t로 사용
    dx = np.diff(x)
    dy = np.diff(y)
    distances = np.sqrt(dx**2 + dy**2)
    t = np.concatenate(([0], np.cumsum(distances)))

    # 3. 3차 스플라인 생성 (Cubic Spline)
    # 입력한 점들을 부드럽게 잇는 함수 생성
    cs_x = CubicSpline(t, x, bc_type='natural')
    cs_y = CubicSpline(t, y, bc_type='natural')

    # 4. 촘촘하게 점 생성 (0.01m 간격)
    total_length = t[-1]
    num_points = int(total_length / 0.01)  # 0.01m 간격
    t_new = np.linspace(0, total_length, num_points)

    new_x = cs_x(t_new)
    new_y = cs_y(t_new)

    # 5. CSV 저장 (헤더 없이)
    df = pd.DataFrame({'X': new_x, 'Y': new_y})
    df.to_csv(filename, header=False, index=False)
    
    print(f"[Success] {filename} created with {len(new_x)} points.")
    return new_x, new_y

# ==========================================

tasks = [
    {
        "filename": "3_5.csv",
        "points": [
            (4.17166666666667, -2.55),
            (3.97166666666667, -2.55),
            (3.79166666666667, -2.4261666666667),
            (3.61166666666667, -2.3023333333333),
            (3.41166666666667, -2.3023333333333),
        ]
    },
    # 3_7, 7_9 도 이런 식으로 추가
]

# ==========================================

# 실행 및 시각화
plt.figure(figsize=(8, 8))

for task in tasks:
    try:
        nx, ny = create_lane_csv(task['filename'], task['points'])
        
        # 그래프 그리기
        plt.plot(nx, ny, label=f"Result: {task['filename']}")
        
        # 사용자가 찍은 점 표시 (빨간 점)
        px = [p[0] for p in task['points']]
        py = [p[1] for p in task['points']]
        plt.scatter(px, py, c='red', s=50, zorder=5, label='Key Points')
        
    except Exception as e:
        print(f"Error creating {task['filename']}: {e}")

plt.legend()
plt.grid(True)
plt.axis('equal')
plt.title("Designed Lane Path (Cubic Spline)")
plt.show()



'''
27_31.csv 파일 생성 시 사용한 spline 점
(2.78166666666667,2.55),
(3.2 ,2.55),
(3.54166666666667, 2.55),
(4,2.30833333333333),
(4.42,2.30833333333333)

3_7.csv 파일 생성 시 사용한 spline 점
(4.1, -2.55),
(3.9, -2.55),
(3.64, -2.671),
(3.4, -2.79166666666667),
(3.2, -2.79166666666667)

7_9.csv 파일 생성 시 사용한 spline 점
(1.15, -2.79166666666667),
(0.95, -2.79166666666667),
(0.783333333333333, -2.671),
(0.6, -2.55),
(0.4, -2.55)
'''