import os
import math


file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'puzzle.txt')

avoidance = 0


def rotate_points(points, angle_deg):
            angle_rad = math.radians(angle_deg)
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            return [(x * cos_a - y * sin_a, x * sin_a + y * cos_a) for x, y in points]


with open(file_path, "w") as file:
    for z in range(3, 50, 3):
        temp = []

        temp.append((200, 40))
        temp.append((150, 40))
        temp.append((150, 120))
        temp.append((200, 120))
        temp.append((200, 200))
        temp.append((120, 200))
        temp.append((120, 150))
        temp.append((40, 150))
        temp.append((40, 200))

        full_path = []
        for k in range(4):
            rotated = rotate_points(temp, 90 * k)
            full_path.extend(rotated)

        full_path = full_path[avoidance:] + full_path[:avoidance]

        for x, y in full_path:
            file.write(f"{float(x)} {float(y)} {float(z)}\n")

        file.write(f"{float(full_path[0][0])} {float(full_path[0][1])} {float(z)}\n")
        file.write(f"\n")

        avoidance += 1
        if avoidance == 2:
            avoidance = -1

