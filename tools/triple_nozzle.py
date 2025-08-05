import math


layer = 0
num_points = 24
twist = 0


with open("/home/ha/kamic/manual/triple_nozzle.txt", "w") as file:
    for z in range(0, 50, 3):
        r = math.sqrt(40000 - 60*z)

        temp = []

        for i in range(num_points):
            theta = (2 * math.pi / num_points) * (i + num_points // 4)
            x = r * math.cos(theta)
            y = r * math.sin(theta)

            temp.append((x, y, float(z)))

        temp = temp[twist:] + temp[:twist]

        for i in range(num_points):
            file.write(f"{temp[i][0]} {temp[i][1]} {temp[i][2]}\n")
            if i == (num_points // 3) * 1 + twist or i == (num_points // 3) * 2 + twist:
                file.write(f"\n")
                file.write(f"{temp[i][0]} {temp[i][1]} {temp[i][2]}\n")
        
        file.write(f"{temp[0][0]} {temp[0][1]} {temp[0][2]}\n")
        file.write(f"\n")

        layer += 1

        twist += 1
        if twist == 2:
            twist = -1

