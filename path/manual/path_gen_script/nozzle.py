import os
import math


file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nozzle.txt')

circle_sampling_cnt = 24
avoidance = 0

with open(file_path, "w") as file:
    for z in range(3, 500, 3):
        r = math.sqrt(40000 - 60*z)

        temp = []

        for i in range(circle_sampling_cnt):
            theta = (2 * math.pi / circle_sampling_cnt) * (i + circle_sampling_cnt // 4)
            x = r * math.cos(theta)
            y = r * math.sin(theta)

            temp.append((x, y, float(z)))

        temp = temp[avoidance:] + temp[:avoidance]

        for i in range(circle_sampling_cnt):
            file.write(f"{temp[i][0]} {temp[i][1]} {temp[i][2]}\n")
            if i == (circle_sampling_cnt // 3) * 1 + avoidance or i == (circle_sampling_cnt // 3) * 2 + avoidance:
                file.write(f"\n")
                file.write(f"{temp[i][0]} {temp[i][1]} {temp[i][2]}\n")
        
        file.write(f"{temp[0][0]} {temp[0][1]} {temp[0][2]}\n")
        file.write(f"\n")

        avoidance += 1
        if avoidance == 2:
            avoidance = -1

