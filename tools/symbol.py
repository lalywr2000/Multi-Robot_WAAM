import math
import matplotlib.pyplot as plt

radius = 70
num_points = 24  # 6+6+6
layer = 5
separate = 2

xy_1q = []  # 1사분면만

# 곡선 생성 (1사분면)
for i in range(0, 6):
    theta = (2 * math.pi / num_points) * i
    x = radius * math.cos(theta) + 140.0
    y = radius * math.sin(theta)
    xy_1q.append((x, y))

for i in range(18, 12, -1):
    theta = (2 * math.pi / num_points) * i
    x = radius * math.cos(theta) + 140.0
    y = radius * math.sin(theta) + 140.0
    xy_1q.append((x, y))

for i in range(0, 6):
    theta = (2 * math.pi / num_points) * i
    x = radius * math.cos(theta)
    y = radius * math.sin(theta) + 140.0
    xy_1q.append((x, y))

# 회전 함수 정의
def rotate_points(points, angle_deg):
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return [(x * cos_a - y * sin_a, x * sin_a + y * cos_a) for x, y in points]

# 전체 4사분면 구성
xy_full = []
for k in range(4):
    rotated = rotate_points(xy_1q, 90 * k)
    xy_full.extend(rotated)

for i in range(layer):
    xy_full = xy_full[separate:] + xy_full[:separate]

    for x, y in xy_full:
        print(f"{round(x, 6)} {round(y, 6)} {i}.0")
    print(f"{round(xy_full[0][0], 6)} {round(xy_full[0][1], 6)} {i}.0")
    print()

