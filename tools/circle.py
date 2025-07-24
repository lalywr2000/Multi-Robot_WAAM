import math

radius = 150
num_points = 24

for i in range(num_points):
    theta = (2 * math.pi / num_points) * i
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    print(f"{round(x, 6)} {round(y, 6)} 0.0")

