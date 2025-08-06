import math


twist = 0

with open("/home/ha/kamic/path/manual/puzzle.txt", "w") as file:
    for layer in range(0, 50, 3):
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

        def rotate_points(points, angle_deg):
            angle_rad = math.radians(angle_deg)
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            return [(x * cos_a - y * sin_a, x * sin_a + y * cos_a) for x, y in points]

        full_path = []
        for k in range(4):
            rotated = rotate_points(temp, 90 * k)
            full_path.extend(rotated)

        twisted = full_path[twist:] + full_path[:twist]

        for i in range(len(twisted)):
            file.write(f"{twisted[i][0]} {twisted[i][1]} {layer}\n")
            
        file.write(f"{twisted[0][0]} {twisted[0][1]} {layer}\n")
        file.write(f"\n")

        twist += 1
        if twist == 2:
            twist = -1

