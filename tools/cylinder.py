import math

radius = 150
num_points = 24
layer = 5

xy = []  # [(x, y)]

for i in range(num_points):
    theta = (2 * math.pi / num_points) * i
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    xy.append((x, y))
xy.append(xy[0])

for i in range(layer):
    c1, c2, c3 = xy[:9], xy[8:17], xy[16:]

    for x, y in c1:
        print(f"{round(x, 6)} {round(y, 6)} {i}.0")
    print()

    for x, y in c2:
        print(f"{round(x, 6)} {round(y, 6)} {i}.0")
    print()

    for x, y in c3:
        print(f"{round(x, 6)} {round(y, 6)} {i}.0")
    print()
