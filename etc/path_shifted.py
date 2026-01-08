import json

# ==========================================
# [보정 값 설정]
# 차 위치: -3.865
# 맵 위치: -2.442
# 결론: 왼쪽으로 약 1.423m 당겨야 함
SHIFT_X = -0.2416666666
SHIFT_Y = 0.0      # Y축은 일단 그대로 둠 (필요하면 수정)
# ==========================================

input_file = "lane3.json"      # 원본 파일명 (없으면 path.json으로 바꾸세요)
output_file = "lane3_s.json" # 저장될 파일명

try:
    with open(input_file, 'r') as f:
        data = json.load(f)
        
    old_x = data['X']
    old_y = data['Y']
    
    # 오차 보정 적용
    new_x = [x + SHIFT_X for x in old_x]
    new_y = [y + SHIFT_Y for y in old_y]
    
    new_data = {"X": new_x, "Y": new_y}
    
    with open(output_file, 'w') as f:
        json.dump(new_data, f)
        
    print(f"✅ 보정 완료!")
    print(f"X축을 {SHIFT_X}만큼 이동했습니다.")
    print(f"저장된 파일: {output_file}")
    
except FileNotFoundError:
    print(f"❌ 에러: '{input_file}' 파일을 찾을 수 없습니다. 파일 이름을 확인해주세요.")
except Exception as e:
    print(f"❌ 에러 발생: {e}")