import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import sys
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy.polynomial import Polynomial
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib
from matplotlib.widgets import Slider, Button
import time
import os

################### Main Process Parameters ######################
R = 318                     # unit: mm      (FuelTank Radius)
D = 33.35                   # unit: mm      (Fixed Cylinder Diameter)

TRAVEL_SPEED = 500          # unit: mm/min
WIRE_FEEDRATE = 3.5         # unit: m/min
LASER_POWER = 3000          # unit: W
WORKING_DISTANCE = 101      # unit: mm      (Distance from substrate to PRESITEC laser tip)
TARGET_THICKNESS = 10       # unit: mm      (Fueltank have Thickness)
START_THETA = 0             # unit: degree  (Start E2 Angle, When you see jig on the top, + = counter clock wise, - = clock wise)
DISTANCE_FROM_JIG = 85      # unit: mm      (Distance from jig to Center Line)

SEAM_AVOIDANCE = 11         # unit: degree  (range: -180 ~ 180, When Head move to next layer, E2's degree)
AIR_TRAVEL_SPEED = 1500     # unit: mm/min  (When Head move to next layer, Head had speed)
AIR_DISTANCE = 30           # unit: mm      (Distance from End Point to Air Point/ Recommand 100mm)

SCANNER_DISTANCE = 50       # unit: mm      (Delta Distance from Center of Wire to 3D Scanner, +)
DEFORMATION_DISTANCE = 30   # unit: mm      (Delta Distance from Center of Wire to Deformation Sensor, +)

######################### Toolpath ###############################
TOOLPATH = 7                # range: 1~7     No.1 = 1-2-1-2, No.2 = 2-1-2-1, No.3 = 1-2-2-1, No.4 = 2-1-1-2, No.5 = 1-2-1-2(Ladder), No.6 = Custom, No.7 = Odd & Even Custom
CUSTOM_TOOLPATH = [2,3,1]   # unit: number  (When Customize toolpath number, Enter in order bead number)
CCUSTOM_TOOLPATH = [2,1,3]  # unit: number  (When Customize toolpath number even & odd Layer, Enter in order even layer's bead number)
TEST_SRC = 1                # range: 1~2     No.1 = on, No.2 = off (Fueltank Test Code)
TARGET_E1 = -80.00          # unit: degree  (If you turn on DIVIDE_E1_MODE, deposition head & e1 angle divide each other to achieve TARGET_E1
WRONG_E1 = -0.378           # unit: degree  (How degree E1 wrong tilt?)

######################## Bead Parameters ############################
LAYER_THICKNESS = 1.7       # unit: mm      (Bead's Height)
BEAD_WIDTH = 4.6            # unit: mm
BEAD_COUNT = 3              # unit: count   (Bead's count)
RADIAL_OVERLAP = 130        # unit: %, mm   (Bead's Radial Overlap)
AXIAL_OVERLAP = 10          # unit: %, mm   (Bead's Axial Overlap) (% = 0 < Range < 100)
MATERIAL_DENSITY = 4.4      # unit: g/cm^3  

################## Additional Type ###############################
ADDITIONAL_LAYER = 9        # unit: layer   (When Add Hemisphere's height, How many layer do you want? Total Additional Height is belong to Layer thickness)
DEFORMATION_SENSOR = 1      # range: 1~2     No.1 = on, No.2 = off (Defromation code on & off)
SCANNER_SENSOR = 2          # range: 1~2     No.1 = on, No.2 = off (3D Scanner code on & off)
AUTO_BEAD_MODE = 1          # range: 1~2     No.1 = on, No.2 = off (Auto picking bead position & count)
OVERLAP_TYPE = 1            # range: 1~2     No.1 = %, No.2 = mm   (Determine bead's axial & radial overlap type)
REINFORCEMENT_TYPE = 3      # range: 1~3     No.1 = No Reinforcement, No.2 = In,External Reinforcement, No.3 = Only External Reinforcement (Bead count = n /2n,2n-1,2n-2,,,,n)
DOUBLE_REINFORCEMENT = 1    # range: 1~2     No.1 = on, No.2 = off (Bead count = n /2n,2n,2n-1,2n-1,2n-2,2n-2,,,,n)
WELD_REINFORCEMENT = 1      # range: 1~2     No.1 = on, No.2 = off 
SPHERE_TYPE = 1             # range: 1~2     No.1 = Hemisphere, No.2 = Sphere
COOLING_TYPE = 1            # range: 1~2     No.1 = None, No.2 = Cooling Clockwise, No.3 = Cooling Counter Clockwise
SIMULATION_TYPE = 2         # range: 1~2     No.1 = on, No.2 = off (Simulation Additive Manufacturing)
REPORT_TYPE = 1             # range: 1~2     No.1 = on, No.2 = off (Each Layer Radius,Coordinate,, etc Report generate)
WORKING_SUPPORT_MODE = 1    # range: 1~2     No.1 = on, No.2 = off (Turn On&Off Working Supporting)
DIVIDE_E1_MODE = 1          # range: 1~2     No.1 = on, No.2 = off (Divide from original e1 angle to e1 & bc each other)
E1_DIRECTION = 2            # range: 1~2     No.1 = clockwise, No.2 = Counter Clockwise (Where ti-tank tilt do you want?)

#################### Adaptive Slicing Parameters ###################
ADAPTIVE_SLICING = 2        # range: 1~2     No.1 = on, No.2 = off
ADAPTIVE_LIST = [0,0,0.8,1.1,1.25,
                 1.5,1.7,1.75,1.75,1.75,
                 1.75,1.6,1.6,1.5,1.35,
                 1.25,1.2,1.2,1.1,1.0,
                 1.0,1.0,0.9,0.85,0.85,
                 0.8,0.8,0.75,0.75,0.7,
                 0.7,0.65,0.65,0.6,0.6,
                 0.55,0.5,0.4,0.3,0.25,
                 0.2,0.1,0.05,0,0]
ADAPTIVE_LIST_DELTA = 2     # unit: degree  (Degree of Adaptive Slicing)

################### Weld Parameters ######################
# WELD_X = 332.19              
# WELD_Z = 434
WELD_X = 333.63              
WELD_Z = 437.03
WELD_TEST_SRC = 1           # range: 1~2     No.1 = Test Code on, No.2 = Test Code off
WELD_REINFORCEMENT_DIS = 46 # unit: mm      (When making Reinforcement at region of Welding that each other Hemisphere)
WELD_REINFORCE_LAYER = 5    # unit: layer   (When making Reinforcement at region of Welding that Total layer)

######################### Bead Sub Parameters ############################
CIRCLE_SEPERATE = 1.5       # unit: mm      (At Each Layer, Each Circle is Seperated by this)
if OVERLAP_TYPE == 1:
    if RADIAL_OVERLAP == 100:                # Bead's Radial Hatch Distance
        RADIAL_HATCHDIS = 0
    else:
        RADIAL_HATCHDIS = BEAD_WIDTH * (100-RADIAL_OVERLAP)/100
    if AXIAL_OVERLAP == 100:             # Bead's Axial Hatch Distance
        AXIAL_HATCHDIS = 0
    else:
        AXIAL_HATCHDIS = BEAD_WIDTH * (100-AXIAL_OVERLAP)/100
elif OVERLAP_TYPE == 2:
    RADIAL_HATCHDIS = -RADIAL_OVERLAP
    AXIAL_HATCHDIS = -AXIAL_OVERLAP
if AUTO_BEAD_MODE == 1:
    BEAD_COUNT = math.ceil(((TARGET_THICKNESS - BEAD_WIDTH)/AXIAL_HATCHDIS) + 1)    # How many Beads are needed
BEAD_EXPECT_WIDTH = round(BEAD_WIDTH*(1 + (BEAD_COUNT - 1)*(100 - AXIAL_OVERLAP)/100), 3)
BEAD_AREA = BEAD_EXPECT_WIDTH*LAYER_THICKNESS
if TOOLPATH == 6:
    if BEAD_COUNT != len(CUSTOM_TOOLPATH):
        print("The Counts written in CUSTOM_TOOLPATH are not equal to Bead Counts!!\n------Entered Bead Count:",len(CUSTOM_TOOLPATH),"-----Real Bead Count:",BEAD_COUNT,
              "\n#### Input",(BEAD_COUNT-len(CUSTOM_TOOLPATH)),"Bead More ###")
        exit()
elif TOOLPATH == 7:
    if BEAD_COUNT != len(CUSTOM_TOOLPATH) and BEAD_COUNT != len(CCUSTOM_TOOLPATH):
        print("The Counts written in CUSTOM_TOOLPATH are not equal to Bead Counts!!\n------Entered Bead Count:",len(CUSTOM_TOOLPATH),"-----Real Bead Count:",BEAD_COUNT,
              "\n#### Input",(BEAD_COUNT-len(CUSTOM_TOOLPATH)),"Bead More ###")
        print("The Counts written in CCUSTOM_TOOLPATH are not equal to Bead Counts!!\n------Entered Bead Count:",len(CCUSTOM_TOOLPATH),"-----Real Bead Count:",BEAD_COUNT,
              "\n#### Input",(BEAD_COUNT-len(CCUSTOM_TOOLPATH)),"Bead More ###")
        exit()
    elif BEAD_COUNT != len(CUSTOM_TOOLPATH):
        print("The Counts written in CUSTOM_TOOLPATH are not equal to Bead Counts!!\n------Entered Bead Count:",len(CUSTOM_TOOLPATH),"-----Real Bead Count:",BEAD_COUNT,
              "\n#### Input",(BEAD_COUNT-len(CUSTOM_TOOLPATH)),"Bead More ###")
        exit()
    elif BEAD_COUNT != len(CCUSTOM_TOOLPATH):
        print("The Counts written in CCUSTOM_TOOLPATH are not equal to Bead Counts!!\n------Entered Bead Count:",len(CCUSTOM_TOOLPATH),"-----Real Bead Count:",BEAD_COUNT,
              "\n#### Input",(BEAD_COUNT-len(CCUSTOM_TOOLPATH)),"Bead More ###")
        exit()

############################ Variables ##################################
to_deg = 180/np.pi    # Change Radian to Degree
to_rad = np.pi/180    # Change Degree to Radian
day = datetime.now()

############### Savepath ################
base_folder = r"Z:\KAMIC\05.KAMIC_과제\(2022~2024_산업부) (LW-DED)우주항공-방산 티타늄 특수부품 고속유연 생산을 위한 열변형 저감 레이저 기반 금속 와이어 3D프린팅 기술개발\7. Toolpath\2025_CODE"
folder_name = f"{day.strftime('%Y%m%d_%H시%M분')}"
folder_path = os.path.join(base_folder, folder_name)
os.makedirs(folder_path, exist_ok=True)

########################### Sub Parameters ##############################
Start_E1 = round((np.arcsin(D/(2*R))*to_deg), 3)
Delta_E1 = round(2*np.arcsin(LAYER_THICKNESS/2/R)*to_deg, 3)
# Delta_E12 = round(2*np.arcsin(LAYER_THICKNESS/2/CENTER_RADIUS2)*to_deg, 3)
Ladder_E1 = round(2*np.arcsin(LAYER_THICKNESS/4/R)*to_deg, 3)
Volume_E1 = Start_E1 + Ladder_E1
if SPHERE_TYPE == 1:
    if TOOLPATH == 5:
        TOTAL_LAYER = round(((90 - (Start_E1 + Delta_E1*BEAD_COUNT))/Ladder_E1)+0.5) + BEAD_COUNT
    else:
        TOTAL_LAYER = round(((90 - Start_E1)/Delta_E1)+0.5)
elif SPHERE_TYPE == 2:
    if TOOLPATH == 5:
        HALF_TOTAL_LAYER = round(((90 - (Start_E1 + Delta_E1*BEAD_COUNT))/Ladder_E1) + 0.5) + BEAD_COUNT
        TOTAL_LAYER = round(((180 - (2*Start_E1 + Delta_E1*BEAD_COUNT))/Ladder_E1) + 0.5) + BEAD_COUNT
    else:
        HALF_TOTAL_LAYER = round(((90-Start_E1)/Delta_E1)+0.5)
        TOTAL_LAYER = round(((180-2*Start_E1)/Delta_E1)+0.5)
JIG_DIS = DISTANCE_FROM_JIG + 40                  # To Compensate Distance from Jig (Safe Distance) 
HEAD_WORKING_DISTANCE = WORKING_DISTANCE + 182.09 # To Compensate Distance from Substrate to Head (Working Distance)
AVOIDANCE = SEAM_AVOIDANCE * -1                   # To Compensate Seam Avoidance
ROBOT_SPEED = TRAVEL_SPEED/60000                  # Travel Speed KUKA can read
FEEDER_SPEED = WIRE_FEEDRATE*6553.5               # Wire Feedrate KUKA can read
AO_LASER_POWER = LASER_POWER/4000                 # Laser Power KUKA can read
AIR_SPEED = AIR_TRAVEL_SPEED/60000                # Air Travel Speed KUKA can read
if np.abs(SEAM_AVOIDANCE) <= 60:
    AIR_DISTANCE_SEAM = AIR_DISTANCE 
elif np.abs(SEAM_AVOIDANCE) > 60 and np.abs(SEAM_AVOIDANCE) <= 120:
    AIR_DISTANCE_SEAM = 2*AIR_DISTANCE
elif np.abs(SEAM_AVOIDANCE) > 120 and np.abs(SEAM_AVOIDANCE) <= 180:
    AIR_DISTANCE_SEAM = 3*AIR_DISTANCE
CUSTOM_TOOLPATH = [x - 1 for x in CUSTOM_TOOLPATH]
CCUSTOM_TOOLPATH = [x - 1 for x in CCUSTOM_TOOLPATH]
if DOUBLE_REINFORCEMENT == 1:
    bead_limit = 2*BEAD_COUNT
else:
    bead_limit = BEAD_COUNT

############ Adaptive Slicing Sub Parameters ##############
x_adaptive_list = np.linspace(0, 10, len(ADAPTIVE_LIST))
y_adaptive_list = ADAPTIVE_LIST  
poly_degree = 5
poly = Polynomial.fit(x_adaptive_list, y_adaptive_list, poly_degree)
adaptive_inter_x = np.linspace(x_adaptive_list.min(), x_adaptive_list.max(), TOTAL_LAYER)
adaptive_inter_y = poly(adaptive_inter_x)
poly_df = pd.DataFrame({'x': adaptive_inter_x, 'y': adaptive_inter_y})
poly_df['y'] = poly_df['y'].apply(lambda x: 0 if x <= 0 else x)
poly_df['y'] = poly_df['y'].round(3)
add_poly = pd.DataFrame({'x':[0]*ADDITIONAL_LAYER, 'y':[0]*ADDITIONAL_LAYER})
poly_df = pd.concat([poly_df, add_poly], ignore_index=True)
BACK_INTERPOLATE_RADIUS = [0]
for i in range(TOTAL_LAYER - 1 + ADDITIONAL_LAYER):
    back_interpolate_radius = round(poly_df['y'].iloc[i+1] - poly_df['y'].iloc[i],3)
    BACK_INTERPOLATE_RADIUS.append(back_interpolate_radius)

#################### Weld Sub Parameters ########################
WELD_BEAD_COUNT = math.ceil(((WELD_REINFORCEMENT_DIS - BEAD_WIDTH)/AXIAL_HATCHDIS) + 1)

################### Generating Report ####################
def make_row(original):
    new = []
    if REINFORCEMENT_TYPE != 1:
        for id, row in enumerate(original, start= 1):
            new_row = row + [0]*(len(original[0]) - len(row))
            new.append(new_row)
    return new

###########################################################
for repeat_num in range(2):
    if repeat_num == 1:                  # To make toolpath report. calculate one more time baby
        TOOLPATH = 1
    ##################### Bead Radius & Working Distance & Adaptive Radius #############################
    BEAD_RADIUS = []                     # 2 Dimension of List, [layer][bead_count]
    BEAD_WORKING_DISTANCE = []           # 2 Dimension of List, [layer][bead_count]
    BEAD_JIG_DIS = []                    # 2 Dimension of List, [layer][bead_count]
    BEAD_RADIUS_INTERPOLATE = []         # 1 Dimension of List, [layer]
    ADAPTIVE_SLICING_RADIUS = []         # 2 Dimension of List, [layer][0= interpolate_radius, 1= adaptive_radius]
    for i in range(1, TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        bead_radius = []
        bead_working_distance = []
        bead_jig_dis = []
        interpolate_radius = poly_df['y'].iloc[i - 1]
        BEAD_RADIUS_INTERPOLATE.append(interpolate_radius)
        adaptive_radius = R + interpolate_radius
        adaptive_slicing_radius = [interpolate_radius, adaptive_radius]
        ADAPTIVE_SLICING_RADIUS.append(adaptive_slicing_radius)
        if REINFORCEMENT_TYPE == 1:
            current_bead_count = BEAD_COUNT  # BEAD_COUNT remains constant
            for j in range(1, current_bead_count + 1):
                if ADAPTIVE_SLICING == 1:
                    bead_radius.append(round((R + interpolate_radius) + (j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                    bead_jig_dis.append(round(JIG_DIS - interpolate_radius - (j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                else:
                    bead_radius.append(round(R + (j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                    bead_jig_dis.append(round(JIG_DIS - (j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                bead_working_distance.append(round(-((j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS) * np.tan(Delta_E1 * to_rad), 3))
            if TOOLPATH == 5:
                if i % 2 == 1:
                    bead_radius = bead_radius[:(BEAD_COUNT//2)]
                    bead_working_distance = bead_working_distance[:(BEAD_COUNT//2)]
                    bead_jig_dis = bead_jig_dis[:(BEAD_COUNT//2)]
                else:
                    bead_radius = bead_radius[(BEAD_COUNT//2):]
                    bead_working_distance = bead_working_distance[(BEAD_COUNT//2):]
                    bead_jig_dis = bead_jig_dis[(BEAD_COUNT//2):]
            elif TOOLPATH == 6:
                bead_radius = [bead_radius[i] for i in CUSTOM_TOOLPATH]
                bead_working_distance = [bead_working_distance[i] for i in CUSTOM_TOOLPATH]
                bead_jig_dis = [bead_jig_dis[i] for i in CUSTOM_TOOLPATH]
            elif TOOLPATH == 7:
                if i % 2 == 1:
                    bead_radius = [bead_radius[i] for i in CUSTOM_TOOLPATH]
                    bead_working_distance = [bead_working_distance[i] for i in CUSTOM_TOOLPATH]
                    bead_jig_dis = [bead_jig_dis[i] for i in CUSTOM_TOOLPATH]
                else:
                    bead_radius = [bead_radius[i] for i in CCUSTOM_TOOLPATH]
                    bead_working_distance = [bead_working_distance[i] for i in CCUSTOM_TOOLPATH]
                    bead_jig_dis = [bead_jig_dis[i] for i in CCUSTOM_TOOLPATH]
        elif REINFORCEMENT_TYPE == 2:
            if DOUBLE_REINFORCEMENT == 1:
                current_bead_count = max(BEAD_COUNT, 2*BEAD_COUNT - (i - 1)//2)
            else:
                current_bead_count = max(BEAD_COUNT, 2*BEAD_COUNT - (i - 1))
            for j in range(1, current_bead_count + 1):
                if ADAPTIVE_SLICING == 1:
                    bead_radius.append(round((R + interpolate_radius)+ (j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                    bead_jig_dis.append(round(JIG_DIS - interpolate_radius - (j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                else:
                    bead_radius.append(round(R + (j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                    bead_jig_dis.append(round(JIG_DIS - (j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                if i == 1:
                    bead_working_distance.append(round(-((j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS) * np.tan(Start_E1 * to_rad), 3))
                else:
                    bead_working_distance.append(round(-((j - (current_bead_count / 2 + 0.5)) * AXIAL_HATCHDIS) * np.tan(Delta_E1 * to_rad), 3))
        elif REINFORCEMENT_TYPE == 3:
            if DOUBLE_REINFORCEMENT == 1:
                current_bead_count = max(BEAD_COUNT, 2*BEAD_COUNT - (i - 1)//2)
            else:
                current_bead_count = max(BEAD_COUNT, 2*BEAD_COUNT - (i - 1))
            for j in range(1, current_bead_count + 1):
                if j <= BEAD_COUNT:
                    if ADAPTIVE_SLICING == 1:
                        bead_radius.append(round((R + interpolate_radius)+ (j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                        bead_jig_dis.append(round(JIG_DIS - interpolate_radius - (j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                    else:
                        bead_radius.append(round(R + (j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                        bead_jig_dis.append(round(JIG_DIS - (j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                    if i == 1:
                        bead_working_distance.append(round(-((j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS) * np.tan(Start_E1 * to_rad), 3))
                    else:
                        bead_working_distance.append(round(-((j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS) * np.tan(Delta_E1 * to_rad), 3))
                else:
                    if ADAPTIVE_SLICING == 1:
                        bead_radius.append(round((R + interpolate_radius)+ (j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                        bead_jig_dis.append(round(JIG_DIS - interpolate_radius - (j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                    else:
                        bead_radius.append(round(R + (j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                        bead_jig_dis.append(round(JIG_DIS - (j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS, 3))
                    if i == 1:
                        bead_working_distance.append(round(-((j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS) * np.tan(Start_E1 * to_rad), 3)) 
                    else:
                        bead_working_distance.append(round(-((j - (BEAD_COUNT / 2 + 0.5)) * AXIAL_HATCHDIS) * np.tan(Delta_E1 * to_rad), 3)) 
        if REINFORCEMENT_TYPE in [2,3]:
            if TOOLPATH == 5 and i > bead_limit:
                if i % 2 == 1:
                    bead_radius = bead_radius[:(BEAD_COUNT//2)]
                    bead_working_distance = bead_working_distance[:(BEAD_COUNT//2)]
                    bead_jig_dis = bead_jig_dis[:(BEAD_COUNT//2)]
                else:
                    bead_radius = bead_radius[(BEAD_COUNT//2):]
                    bead_working_distance = bead_working_distance[(BEAD_COUNT//2):]
                    bead_jig_dis = bead_jig_dis[(BEAD_COUNT//2):]
            if i > bead_limit:
                if TOOLPATH == 6:
                    bead_radius = [bead_radius[i] for i in CUSTOM_TOOLPATH]
                    bead_working_distance = [bead_working_distance[i] for i in CUSTOM_TOOLPATH]
                    bead_jig_dis = [bead_jig_dis[i] for i in CUSTOM_TOOLPATH]
                elif TOOLPATH == 7:
                    if i % 2 == 1:
                        bead_radius = [bead_radius[i] for i in CUSTOM_TOOLPATH]
                        bead_working_distance = [bead_working_distance[i] for i in CUSTOM_TOOLPATH]
                        bead_jig_dis = [bead_jig_dis[i] for i in CUSTOM_TOOLPATH]
                    else:
                        bead_radius = [bead_radius[i] for i in CCUSTOM_TOOLPATH]
                        bead_working_distance = [bead_working_distance[i] for i in CCUSTOM_TOOLPATH]
                        bead_jig_dis = [bead_jig_dis[i] for i in CCUSTOM_TOOLPATH]
        if TOOLPATH == 2:
            bead_radius.reverse()
            bead_working_distance.reverse()
            bead_jig_dis.reverse()
        if TOOLPATH == 3:
            if i % 2 == 0:
                bead_radius.reverse()
                bead_working_distance.reverse()
                bead_jig_dis.reverse()
        if TOOLPATH == 4:
            if i % 2 == 1:
                bead_radius.reverse()
                bead_working_distance.reverse()
                bead_jig_dis.reverse()
        if ADDITIONAL_LAYER != 0 and i > TOTAL_LAYER:
            bead_working_distance = [0]*BEAD_COUNT
        BEAD_RADIUS.append(bead_radius)
        BEAD_WORKING_DISTANCE.append(bead_working_distance)
        BEAD_JIG_DIS.append(bead_jig_dis)
    max_columns = max(len(row) for row in BEAD_RADIUS)
    report_bead_radius = []
    report_bead_radius_columns = [f"Bead{i+1}" for i in range(len(BEAD_RADIUS[0]))]
    for id, row in enumerate(BEAD_RADIUS, start= 1):
            new_row = row + [0]*(max_columns - len(row))
            report_bead_radius.append(new_row)
    BEAD_RADIUS_df = pd.DataFrame(report_bead_radius, columns = report_bead_radius_columns)
    ADAPTIVE_SLICING_RADIUS_df = pd.DataFrame(ADAPTIVE_SLICING_RADIUS, columns=['Interpolate', 'Center Radius'])

    ############################ INITIAL ROBOT POSITION ##############################
    initial_x = 345
    initial_z = 0
    initial_x1 = 229.791
    initial_z1 = 0
    initial_x = -initial_x if E1_DIRECTION == 2 else initial_x
    initial_x1 = -initial_x1 if E1_DIRECTION == 2 else initial_x1
    start_ta = -START_THETA 
    initial_a = 90 + START_THETA
    if initial_a >=180:
        initial_a = initial_a -360
    if initial_a <=-180:
        initial_a = initial_a +360
    initial_X = initial_x*np.cos(start_ta*to_rad) - initial_z*np.sin(start_ta*to_rad)       
    initial_Z = initial_x*np.sin(start_ta*to_rad) + initial_z*np.cos(start_ta*to_rad)
    initial_X1 = initial_x1*np.cos(start_ta*to_rad) - initial_z1*np.sin(start_ta*to_rad)       
    initial_Z1 = initial_x1*np.sin(start_ta*to_rad) + initial_z1*np.cos(start_ta*to_rad) 
    initial_e1 = 80.0
    initial_e1 = -initial_e1 if E1_DIRECTION == 2 else initial_e1

    ######################### Theta List ###########################
    ta_list = []
    for i in range(1,TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        if TOOLPATH == 5:
            if i <= bead_limit:
                ta = Start_E1 + (i - 1)*Delta_E1
            else:
                ta = Start_E1 + BEAD_COUNT*Delta_E1 + (i - BEAD_COUNT)*Ladder_E1
        else:
            ta = Start_E1 + (i - 1)*Delta_E1
        if ADDITIONAL_LAYER != 0:
            if i > TOTAL_LAYER:
                ta = 90.0
        ta_list.append(ta)

    #################### E1 Angle ####################
    E1_ANGLE = []                                 # 1 Dimension of list, [layer]
    for i in range(1,TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        if ta_list[i-1] >= 90:
            ttaa = round((ta_list[i-1] - 90),3)
        else:
            ttaa = round((90 - ta_list[i-1]),3)  
        ttaa = ttaa if E1_DIRECTION == 1 else -ttaa
        E1_ANGLE.append(ttaa)

    ######################### Divide Angle E1 & B Each Other #####################
    C_ANGLE = []                                  # 1 Dimension of list, [layer]
    c_angle_count = 0
    E1_ANGLE = []
    for i in range(1, TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        if DIVIDE_E1_MODE == 1:
            if ta_list[i-1] <= 90 - np.abs(TARGET_E1):
                c_angle = round(90 - ta_list[i-1] - np.abs(TARGET_E1), 3)
                c_angle = c_angle if E1_DIRECTION == 2 else -c_angle
                C_ANGLE.append(c_angle)
                E1_ANGLE.append(TARGET_E1)
                c_angle_count += 1
            else:
                if ta_list[i-1] >= 90:
                    ttaa = round((ta_list[i-1] - 90),3)
                else:
                    ttaa = round((90 - ta_list[i-1]),3)
                ttaa = ttaa if E1_DIRECTION == 1 else -ttaa
                C_ANGLE.append(0.00) 
                E1_ANGLE.append(ttaa)
        else:
            C_ANGLE.append(0.00)

    ################## Initial Coordinate & E2 Radial Overlap & Point Count ##################
    BEAD_INITIAL_COORDINATE_X = []                # 2 Dimension of list, [layer][bead_count]
    BEAD_INITIAL_COORDINATE_Z = []                # 2 Dimension of list, [layer][bead_count]
    E2_ANGLE = []                                 # 2 Dimension of list, [layer][bead_count]
    E2_RADIAL_OVERLAP = []                        # 2 Dimension of list, [layer][bead_count], Each Layer's have radius, how E2 angle degree
    POINT_GROUP = []                              # 2 Dimension of list, [layer][bead_count]
    for i in range(1,TOTAL_LAYER + 1 + ADDITIONAL_LAYER):   
        INITIAL_COORDINATE_X = []
        INITIAL_COORDINATE_Z = []
        E2_angle = []
        E2_radial_overlap = []
        point_group = []
        for j in range(len(BEAD_RADIUS[i-1])):
            BEAD_X = round(float(BEAD_RADIUS[i-1][j])*np.sin(ta_list[i-1]*to_rad), 3)
            if ADAPTIVE_SLICING == 1:
                BEAD_Z = round(float(BEAD_RADIUS[i-1][j]) - (float(BEAD_RADIUS[i-1][j]) * np.cos(ta_list[i-1]*to_rad) - BACK_INTERPOLATE_RADIUS[i-1]) + BEAD_JIG_DIS[i-1][j],3) 
            else:
                BEAD_Z = round(float(BEAD_RADIUS[i-1][j]) - (float(BEAD_RADIUS[i-1][j]) * np.cos(ta_list[i-1]*to_rad)) + BEAD_JIG_DIS[i-1][j],3)
            if ADDITIONAL_LAYER != 0 and i >= TOTAL_LAYER + 1:
                BEAD_Z = round(BEAD_Z + (i - TOTAL_LAYER)*LAYER_THICKNESS, 3)
            BEAD_X = -BEAD_X if E1_DIRECTION == 2 else BEAD_X
            DELTA_E2 = (CIRCLE_SEPERATE/np.abs(BEAD_X))*to_deg
            EACH_BEAD_ROUND = 2*np.abs(BEAD_X)*np.pi
            E2_radial = (RADIAL_HATCHDIS/EACH_BEAD_ROUND)*360
            point_count = (EACH_BEAD_ROUND-RADIAL_HATCHDIS)/CIRCLE_SEPERATE      # Count from start to end point
            if point_count % 1 == 0:
                point_count = int(point_count) - 1
            else:
                point_count = int(point_count)
            INITIAL_COORDINATE_X.append(BEAD_X)
            INITIAL_COORDINATE_Z.append(BEAD_Z)    
            E2_angle.append(DELTA_E2)
            E2_radial_overlap.append(E2_radial)
            point_group.append(point_count)
        BEAD_INITIAL_COORDINATE_X.append(INITIAL_COORDINATE_X)
        BEAD_INITIAL_COORDINATE_Z.append(INITIAL_COORDINATE_Z)
        E2_ANGLE.append(E2_angle)
        E2_RADIAL_OVERLAP.append(E2_radial_overlap)
        POINT_GROUP.append(point_group)
    ta_list_df = pd.DataFrame(E1_ANGLE, columns= ['E1 Angle'])

    ############################# Start(3D Scanner/Deformation Sensor/ Deposition Head) & End & Air Coordinate #############################
    BEAD_START_COORDINATE_X = []                  # 2 Dimension of list, [layer][bead_count]
    BEAD_START_COORDINATE_Y = []                  # 2 Dimension of list, [layer][bead_count]
    SCANNER_COORDINATE_X = []                     # 2 Dimension of list, [layer][bead_count]
    SCANNER_COORDINATE_Y = []                     # 2 Dimension of list, [layer][bead_count]
    DEFORMATION_COORDINATE_X = []                 # 2 Dimension of list, [layer][bead_count]
    DEFORMATION_COORDINATE_Y = []                 # 2 Dimension of list, [layer][bead_count]
    BEAD_END_COORDINATE_X = []                    # 2 Dimension of list, [layer][bead_count]
    BEAD_END_COORDINATE_Y = []                    # 2 Dimension of list, [layer][bead_count]
    BEAD_AIR_COORDINATE_X = []                    # 2 Dimension of list, [layer][bead_count]
    BEAD_AIR_COORDINATE_Y = []                    # 2 Dimension of list, [layer][bead_count]
    BEAD_AIR_COORDINATE_Z = []                    # 2 Dimension of list, [layer][bead_count]
    REPORT_BEAD_E2_START_ANGLE = []               # 2 Dimension of list, [layer][bead_count], +
    REPORT_BEAD_E2_END_ANGLE = []                 # 2 Dimension of list, [layer][bead_count], +
    BEAD_E2_START_ANGLE = []                      # 2 Dimension of list, [layer][bead_count], +
    BEAD_E2_END_ANGLE = []                        # 2 Dimension of list, [layer][bead_count], +
    ROTATING_Y = 0                                # Just Set Initial (X0,Y0) Help Rotating
    E2_COMPENSATE = 0                             # Compensate Radial Overlap Angle
    THETA = 0
    for i in range(1,TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        START_COORDINATE_X = []
        START_COORDINATE_Y = []
        SCANNER_X = []
        SCANNER_Y = []
        DEFORMATION_X = []
        DEFORMATION_Y = []
        END_COORDINATE_X = []
        END_COORDINATE_Y = []
        AIR_COORDINATE_X = []
        AIR_COORDINATE_Y = []
        AIR_COORDINATE_Z = []
        REPORT_E2_START_ANGLE = []
        REPORT_E2_END_ANGLE = []
        E2_START_ANGLE = []
        E2_END_ANGLE = []
        air_ta = -ta_list[i-1] if ta_list[i-1] > 90 else ta_list[i-1]
        for j in range(len(BEAD_RADIUS[i-1])):
            if E1_DIRECTION == 1:
                air_coefficient = (float(BEAD_INITIAL_COORDINATE_X[i-1][j]) + AIR_DISTANCE_SEAM*np.cos(air_ta*to_rad))/float(BEAD_INITIAL_COORDINATE_X[i-1][j])
            else:
                air_coefficient = (float(BEAD_INITIAL_COORDINATE_X[i-1][j]) - AIR_DISTANCE_SEAM*np.cos(air_ta*to_rad))/float(BEAD_INITIAL_COORDINATE_X[i-1][j])
            air_coefficient_full = (float(BEAD_INITIAL_COORDINATE_X[i-1][j]) - AIR_DISTANCE_SEAM*np.cos(air_ta*to_rad))/float(BEAD_INITIAL_COORDINATE_X[i-1][j])
            if i == 1 and j == 0:
                THETA += start_ta
            else:
                THETA += AVOIDANCE
            if SPHERE_TYPE == 1:
                XS = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) - ROTATING_Y*np.sin(THETA*to_rad),3)
                YS = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) + ROTATING_Y*np.cos(THETA*to_rad),3)
                scan_x = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) - (-SCANNER_DISTANCE)*np.sin(THETA*to_rad),3)
                scan_y = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) + (-SCANNER_DISTANCE)*np.cos(THETA*to_rad),3)
                deform_x = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) - (-DEFORMATION_DISTANCE)*np.sin(THETA*to_rad),3)
                deform_y = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) + (-DEFORMATION_DISTANCE)*np.cos(THETA*to_rad),3)
                layer_start_theta = round(((-THETA/360) - int(-THETA/360))*360, 3)
                REPORT_E2_START_ANGLE.append(layer_start_theta)
                E2_START_ANGLE.append(f"{-THETA:.3f}")
                THETA += float(E2_RADIAL_OVERLAP[i-1][j]) - 360 
                XE = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) - ROTATING_Y*np.sin(THETA*to_rad),3)
                YE = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) + ROTATING_Y*np.cos(THETA*to_rad),3)
                layer_end_theta = round(((-THETA/360) - int(-THETA/360))*360, 3)
                REPORT_E2_END_ANGLE.append(layer_end_theta)
                E2_END_ANGLE.append(f"{-THETA:.3f}")
                XA = round(XE*air_coefficient,3)
                YA = round(YE*air_coefficient,3)
                ZA = round(float(BEAD_INITIAL_COORDINATE_Z[i-1][j]) + AIR_DISTANCE_SEAM*np.sin(air_ta*to_rad),3)
            elif SPHERE_TYPE == 2 and ADDITIONAL_LAYER == 0:
                if ADDITIONAL_LAYER != 0:
                    print("Can't Generate Sphere Start Coordinate")
                else:
                    if i <= HALF_TOTAL_LAYER:
                        XS = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) - ROTATING_Y*np.sin(THETA*to_rad),3)
                        YS = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) + ROTATING_Y*np.cos(THETA*to_rad),3)
                        scan_x = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) - (-SCANNER_DISTANCE)*np.sin(THETA*to_rad),3)
                        scan_y = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) + (-SCANNER_DISTANCE)*np.cos(THETA*to_rad),3)
                        deform_x = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) - (-DEFORMATION_DISTANCE)*np.sin(THETA*to_rad),3)
                        deform_y = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) + (-DEFORMATION_DISTANCE)*np.cos(THETA*to_rad),3)
                        layer_start_theta = round(((-THETA/360) - int(-THETA/360))*360, 3)
                        REPORT_E2_START_ANGLE.append(layer_start_theta)
                        E2_START_ANGLE.append(f"{-THETA:.3f}")
                        THETA += float(E2_RADIAL_OVERLAP[i-1][j]) - 360 
                        XE = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) - ROTATING_Y*np.sin(THETA*to_rad),3)
                        YE = round(float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) + ROTATING_Y*np.cos(THETA*to_rad),3)
                        layer_end_theta = round(((-THETA/360) - int(-THETA/360))*360, 3)
                        REPORT_E2_END_ANGLE.append(layer_end_theta)
                        E2_END_ANGLE.append(f"{-THETA:.3f}")
                        XA = round(XE*air_coefficient,3)
                        YA = round(YE*air_coefficient,3)
                        ZA = round(float(BEAD_INITIAL_COORDINATE_Z[i-1][j]) + AIR_DISTANCE_SEAM*np.sin(air_ta*to_rad),3)
                    else:
                        XS = round(-float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) + ROTATING_Y*np.sin(THETA*to_rad),3)
                        YS = round(-float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) - ROTATING_Y*np.cos(THETA*to_rad),3)
                        scan_x = round(-float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) + (-SCANNER_DISTANCE)*np.sin(THETA*to_rad),3)
                        scan_y = round(-float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) - (-SCANNER_DISTANCE)*np.cos(THETA*to_rad),3)
                        deform_x = round(-float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) + (-DEFORMATION_DISTANCE)*np.sin(THETA*to_rad),3)
                        deform_y = round(-float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) - (-DEFORMATION_DISTANCE)*np.cos(THETA*to_rad),3)
                        layer_start_theta = round(((-THETA/360) - int(-THETA/360))*360, 3)
                        REPORT_E2_START_ANGLE.append(layer_start_theta)
                        E2_START_ANGLE.append(f"{-THETA:.3f}")
                        THETA += float(E2_RADIAL_OVERLAP[i-1][j]) - 360 
                        XE = round(-float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.cos(THETA*to_rad) + ROTATING_Y*np.sin(THETA*to_rad),3)
                        YE = round(-float(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.sin(THETA*to_rad) - ROTATING_Y*np.cos(THETA*to_rad),3)
                        layer_end_theta = round(((-THETA/360) - int(-THETA/360))*360, 3)
                        REPORT_E2_END_ANGLE.append(layer_end_theta)
                        E2_END_ANGLE.append(f"{-THETA:.3f}")
                        XA = round(XE*air_coefficient_full,3)
                        YA = round(YE*air_coefficient_full,3)
                        ZA = round(float(BEAD_INITIAL_COORDINATE_Z[i-1][j]) + AIR_DISTANCE_SEAM*np.sin(air_ta*to_rad),3)
            START_COORDINATE_X.append(XS)
            START_COORDINATE_Y.append(YS)
            SCANNER_X.append(scan_x)
            SCANNER_Y.append(scan_y)
            DEFORMATION_X.append(deform_x)
            DEFORMATION_Y.append(deform_y)
            END_COORDINATE_X.append(XE)
            END_COORDINATE_Y.append(YE)
            AIR_COORDINATE_X.append(XA)
            AIR_COORDINATE_Y.append(YA)
            AIR_COORDINATE_Z.append(ZA)
        BEAD_START_COORDINATE_X.append(START_COORDINATE_X)
        BEAD_START_COORDINATE_Y.append(START_COORDINATE_Y)
        SCANNER_COORDINATE_X.append(SCANNER_X)
        SCANNER_COORDINATE_Y.append(SCANNER_Y)
        DEFORMATION_COORDINATE_X.append(DEFORMATION_X)
        DEFORMATION_COORDINATE_Y.append(DEFORMATION_Y)
        BEAD_END_COORDINATE_X.append(END_COORDINATE_X)
        BEAD_END_COORDINATE_Y.append(END_COORDINATE_Y)
        BEAD_AIR_COORDINATE_X.append(AIR_COORDINATE_X)
        BEAD_AIR_COORDINATE_Y.append(AIR_COORDINATE_Y)
        BEAD_AIR_COORDINATE_Z.append(AIR_COORDINATE_Z)
        REPORT_BEAD_E2_START_ANGLE.append(REPORT_E2_START_ANGLE)
        REPORT_BEAD_E2_END_ANGLE.append(REPORT_E2_END_ANGLE)
        BEAD_E2_START_ANGLE.append(E2_START_ANGLE)
        BEAD_E2_END_ANGLE.append(E2_END_ANGLE)

    ######################### Among Coordinate (Start ~ End) ######################
    BEAD_AMONG_COORDINATE_X = []                      # 3 Dimension of List, [layer][bead_count][point_group-2]
    BEAD_AMONG_COORDINATE_Y = []                      # 3 Dimension of List, [layer][bead_count][point_group-2]
    BEAD_E2_AMONG_ANGLE = []
    for i in range(1,TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        AMONG_COORDINATE_X = []
        AMONG_COORDINATE_Y = []
        E2_AMONG_ANGLE = []
        for j in range(len(BEAD_RADIUS[i-1])):
            AMONG_X = []
            AMONG_Y = []
            AMONG_ANGLE = []
            for k in range(POINT_GROUP[i-1][j]):
                Delta_E2 = -(k+1)*float(E2_ANGLE[i-1][j])
                XA = round(float(BEAD_START_COORDINATE_X[i-1][j])*np.cos(Delta_E2*to_rad) - float(BEAD_START_COORDINATE_Y[i-1][j])*np.sin(Delta_E2*to_rad),3)
                YA = round(float(BEAD_START_COORDINATE_X[i-1][j])*np.sin(Delta_E2*to_rad) + float(BEAD_START_COORDINATE_Y[i-1][j])*np.cos(Delta_E2*to_rad),3)
                AMONG_X.append(f"{XA}")
                AMONG_Y.append(f"{YA}")
                AMONG_ANGLE.append(f"{float(BEAD_E2_START_ANGLE[i-1][j]) - Delta_E2:.3f}")
            AMONG_COORDINATE_X.append(AMONG_X)
            AMONG_COORDINATE_Y.append(AMONG_Y)
            E2_AMONG_ANGLE.append(AMONG_ANGLE)
        BEAD_AMONG_COORDINATE_X.append(AMONG_COORDINATE_X)
        BEAD_AMONG_COORDINATE_Y.append(AMONG_COORDINATE_Y)
        BEAD_E2_AMONG_ANGLE.append(E2_AMONG_ANGLE)

    #################### Welding List ####################

    ################### Welding Radius, Delta1&2 #################
    BONG_X = 0
    BONG_Z = round(JIG_DIS + R, 3)
    WELD_R = []
    DELTA_THETA = []
    WELD_BEAD = []
    WELD_JIG_DIS = []
    weld_r = round(((BONG_X - WELD_X)**2+(BONG_Z - WELD_Z)**2)**0.5, 3)
    for i in range(WELD_REINFORCE_LAYER):
        Delta_Theta = round((AXIAL_HATCHDIS/weld_r)*to_deg, 3)
        weld_bead = WELD_BEAD_COUNT 
        weld_jig_dis = round(JIG_DIS - (weld_r - R), 3)
        WELD_R.append(f"{weld_r:.3f}")
        weld_r += LAYER_THICKNESS
        WELD_JIG_DIS.append(weld_jig_dis)
        DELTA_THETA.append(Delta_Theta)
        WELD_BEAD.append(weld_bead)

    ############## Welding E1 Angle ################
    WELD_E1_LIST = []
    for i in range(WELD_REINFORCE_LAYER):
        weld_e1_list = []
        for j in range(WELD_BEAD[i]):
            weld_e1 = round(90 + j*DELTA_THETA[i], 3)
            weld_e1_list.append(weld_e1)
        WELD_E1_LIST.append(weld_e1_list)

    ############## Welding Initial Coordinate ###############
    WELD_START_X = []
    WELD_START_Z = []
    DELTA_THETA2 = []
    WELD_E2_RADIAL_OVERLAP = []
    WELD_POINT_GROUP = []
    WELD_TOTAL_BEAD_COUNT = 0
    for i in range(WELD_REINFORCE_LAYER):
        weld_start_x = []
        weld_start_z = []
        Delta_Theta2 = []
        weld_e2_radial_overlap = []
        weld_point_group = []
        WELD_INITIAL_X = WELD_X + i*LAYER_THICKNESS
        FAKE_Z = 0
        WELD_INITIAL_Z = WELD_Z
        for j in range(WELD_BEAD[i]):
            INITIAL_X = round(WELD_INITIAL_X*np.cos(-j*DELTA_THETA[i]*to_rad) - FAKE_Z*np.sin(-j*DELTA_THETA[i]*to_rad), 3)
            INITIAL_Z = round(WELD_INITIAL_Z + WELD_INITIAL_X*np.sin(-j*DELTA_THETA[i]*to_rad) + FAKE_Z*np.cos(-j*DELTA_THETA[i]*to_rad), 3)
            delta_theta2 = (CIRCLE_SEPERATE/INITIAL_X)*to_deg 
            WELD_EACH_BEAD_ROUND = 2*INITIAL_X*np.pi
            weld_e2_radial = (RADIAL_HATCHDIS/WELD_EACH_BEAD_ROUND)*360
            weld_point_count = (WELD_EACH_BEAD_ROUND - RADIAL_HATCHDIS)/CIRCLE_SEPERATE
            if weld_point_count % 1 == 0:
                weld_point_count = int(weld_point_count) - 1
            else:
                weld_point_count = int(weld_point_count)
            WELD_TOTAL_BEAD_COUNT += 1
            weld_start_x.append(INITIAL_X)
            weld_start_z.append(f"{INITIAL_Z:.3f}")
            Delta_Theta2.append(delta_theta2)
            weld_e2_radial_overlap.append(weld_e2_radial)
            weld_point_group.append(weld_point_count)
        WELD_START_X.append(weld_start_x)
        WELD_START_Z.append(weld_start_z)
        DELTA_THETA2.append(Delta_Theta2)
        WELD_E2_RADIAL_OVERLAP.append(weld_e2_radial_overlap)
        WELD_POINT_GROUP.append(weld_point_group)  
    
    ############ Welding Start & End Coordinate #############
    WELD_START_COORDINATE_X = []
    WELD_START_COORDINATE_Y = []
    WELD_END_COORDINATE_X =[]
    WELD_END_COORDINATE_Y =[]
    WELD_E2_START_ANGLE = []
    WELD_E2_END_ANGLE = []
    WELD_ROTATING_Y = 0
    WELD_E2_COMPENSATE = 0
    WELD_THETA = -5
    for i in range(WELD_REINFORCE_LAYER):
        weld_start_coordinate_x = []
        weld_start_coordinate_y = []
        weld_end_coordinate_x = []
        weld_end_coordinate_y = []
        weld_e2_start_angle = []
        weld_e2_end_angle = []
        for j in range(WELD_BEAD[i]):
            if i == 0 and j == 0:
                WELD_THETA += start_ta
            else:
                WELD_THETA += AVOIDANCE
            WXS = round(float(WELD_START_X[i][j])*np.cos(WELD_THETA*to_rad) - WELD_ROTATING_Y*np.sin(WELD_THETA*to_rad),3)
            WYS = round(float(WELD_START_X[i][j])*np.sin(WELD_THETA*to_rad) + WELD_ROTATING_Y*np.cos(WELD_THETA*to_rad),3)
            weld_e2_start_angle.append(f"{-WELD_THETA:.3f}")
            WELD_THETA += float(WELD_E2_RADIAL_OVERLAP[i][j]) - 360 
            WXE = round(float(WELD_START_X[i][j])*np.cos(WELD_THETA*to_rad) - WELD_ROTATING_Y*np.sin(WELD_THETA*to_rad),3)
            WYE = round(float(WELD_START_X[i][j])*np.sin(WELD_THETA*to_rad) + WELD_ROTATING_Y*np.cos(WELD_THETA*to_rad),3)
            weld_e2_end_angle.append(f"{-WELD_THETA:.3f}")
            weld_start_coordinate_x.append(WXS)
            weld_start_coordinate_y.append(WYS)
            weld_end_coordinate_x.append(WXE)
            weld_end_coordinate_y.append(WYE)
        WELD_START_COORDINATE_X.append(weld_start_coordinate_x)
        WELD_START_COORDINATE_Y.append(weld_start_coordinate_y)
        WELD_END_COORDINATE_X.append(weld_end_coordinate_x)
        WELD_END_COORDINATE_Y.append(weld_end_coordinate_y)
        WELD_E2_START_ANGLE.append(weld_e2_start_angle)
        WELD_E2_END_ANGLE.append(weld_e2_end_angle)

    ############## Welding Among Coordinate ###############
    WELD_AMONG_COORDINATE_X = []                      # 3 Dimension of List, [layer][bead_count][point_group-2]
    WELD_AMONG_COORDINATE_Y = []                      # 3 Dimension of List, [layer][bead_count][point_group-2]
    WELD_E2_AMONG_ANGLE = []
    for i in range(WELD_REINFORCE_LAYER):
        weld_among_coordinate_x = []
        weld_among_coordinate_y = []
        weld_e2_among_angle = []
        for j in range(WELD_BEAD[i]):
            weld_among_x = []
            weld_among_y = []
            weld_among_e2 = []
            for k in range(WELD_POINT_GROUP[i][j]):
                calcul_e2 = round(-(k+1)*float(DELTA_THETA2[i][j]), 3)
                WXA = round(float(WELD_START_COORDINATE_X[i][j])*np.cos(calcul_e2*to_rad) - float(WELD_START_COORDINATE_Y[i][j])*np.sin(calcul_e2*to_rad),3)
                WYA = round(float(WELD_START_COORDINATE_X[i][j])*np.sin(calcul_e2*to_rad) + float(WELD_START_COORDINATE_Y[i][j])*np.cos(calcul_e2*to_rad),3)   
                weld_among_x.append(WXA)
                weld_among_y.append(WYA)
                weld_among_e2.append(f"{float(WELD_E2_START_ANGLE[i][j]) - calcul_e2:.3f}")
            weld_among_coordinate_x.append(weld_among_x)
            weld_among_coordinate_y.append(weld_among_y)
            weld_e2_among_angle.append(weld_among_e2)
        WELD_AMONG_COORDINATE_X.append(weld_among_coordinate_x)
        WELD_AMONG_COORDINATE_Y.append(weld_among_coordinate_y)
        WELD_E2_AMONG_ANGLE.append(weld_e2_among_angle)
    #############################################################################################

    ######################## Calculate Total Time ###########################
    sum_time = 0
    weld_sum_time = 0
    SIGMA_TIME = []                                     # 2 Dimension of List, [layer][bead_count]
    BEAD_EACH_TIME = []                                 # 2 Dimension of List, [layer][3*bead_count,+1,+2]
    WELD_SIGMA_TIME = []                                # 2 Dimension of List, [layer][bead_count]
    WELD_BEAD_EACH_TIME = []                            # 2 Dimension of List, [layer][3*bead_count,+1,+2]
    for i in range(1,TOTAL_LAYER + 1 + ADDITIONAL_LAYER): 
        sigma_time = []
        EACH_TIME = []
        for j in range(len(BEAD_RADIUS[i-1])):
            present_time = round((2*np.abs(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.pi - RADIAL_HATCHDIS)/TRAVEL_SPEED , 3)
            sum_time += present_time
            sigma_time.append(sum_time)
            each_time_h = math.trunc(present_time/60)
            each_time_m = ((present_time/60)-each_time_h)*60
            each_time_mm = math.trunc(each_time_m)
            each_time_s = math.ceil((each_time_m-math.trunc(each_time_m))*60)
            EACH_TIME.append(each_time_h)
            EACH_TIME.append(each_time_mm)
            EACH_TIME.append(each_time_s)
        SIGMA_TIME.append(sigma_time)
        BEAD_EACH_TIME.append(EACH_TIME)
        Fueltank_time = sum_time
    if WELD_REINFORCEMENT == 1:
        for i in range(WELD_REINFORCE_LAYER):
            weld_sigma_time = []
            WELD_EACH_TIME = []
            for j in range(WELD_BEAD[i]):
                weld_present_time = (WELD_START_X[i][j]*2*np.pi - RADIAL_HATCHDIS)/TRAVEL_SPEED
                sum_time += weld_present_time
                weld_sum_time += weld_present_time
                weld_sigma_time.append(weld_sum_time)
                weld_each_time_h = math.trunc(weld_present_time/60)
                weld_each_time_m = ((weld_present_time/60) - weld_each_time_h)*60
                weld_each_time_mm = math.trunc(weld_each_time_m)
                weld_each_time_s = math.ceil((weld_each_time_m-math.trunc(weld_each_time_m))*60)
                WELD_EACH_TIME.append(weld_each_time_h)
                WELD_EACH_TIME.append(weld_each_time_mm)
                WELD_EACH_TIME.append(weld_each_time_s)
            WELD_SIGMA_TIME.append(weld_sigma_time)
            WELD_BEAD_EACH_TIME.append(WELD_EACH_TIME)
    TOTAL_TIME = sum_time
    time_h = int(TOTAL_TIME/60)
    time_m = ((TOTAL_TIME/60) - time_h)*60
    time_mm = int(time_m)
    time_s = math.ceil((time_m - time_mm)*60)

    fuel_time_h = int(Fueltank_time/60)
    fuel_time_m = ((Fueltank_time/60) - fuel_time_h)*60
    fuel_time_mm = int(fuel_time_m)
    fuel_time_s = math.ceil((fuel_time_m - fuel_time_mm)*60)

    weld_time_h = int(weld_sum_time/60)
    weld_time_m = ((weld_sum_time/60) - weld_time_h)*60
    weld_time_mm = int(weld_time_m)
    weld_time_s = math.ceil((weld_time_m - weld_time_mm)*60)

    ############## Calculate Total Distance ##############
    sum_distance = 0
    weld_sum_distance = 0
    SIGMA_DISTANCE = []                                 # 2 Dimension of List, [layer][bead_count]
    BEAD_EACH_DISTANCE = []                             # 2 Dimension of List, [layer][3*bead_count,+1,+2]
    WELD_SIGMA_DISTANCE = []                            # 2 Dimension of List, [layer][bead_count]
    WELD_EACH_DISTANCE = []                             # 2 Dimension of List, [layer][3*bead_count,+1,+2]
    for i in range(1, TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        sigma_distance = []
        bead_each_distance = []
        for j in range(len(BEAD_RADIUS[i-1])):
            present_distance = round(2*np.abs(BEAD_INITIAL_COORDINATE_X[i-1][j])*np.pi - RADIAL_HATCHDIS, 3)
            sum_distance += present_distance
            sigma_distance.append(sum_distance)
            bead_each_distance.append(present_distance)
        SIGMA_DISTANCE.append(sigma_distance)
        BEAD_EACH_DISTANCE.append(bead_each_distance)
    if WELD_REINFORCEMENT == 1:
        for i in range(WELD_REINFORCE_LAYER):
            weld_sigma_distance = []
            weld_each_distance = []
            for j in range(WELD_BEAD[i]):
                weld_present_distance = round(2*WELD_START_X[i][j]*np.pi - RADIAL_HATCHDIS, 3)
                weld_sum_distance += weld_present_distance
                weld_sigma_distance.append(weld_sum_distance)
                weld_each_distance.append(weld_present_distance)
            WELD_SIGMA_DISTANCE.append(weld_sigma_distance)
            WELD_EACH_DISTANCE.append(weld_each_distance)
    TOTAL_DISTANCE = sum_distance + weld_sum_distance
    distance_columns = [f"BEAD {i+1}" for i in range(len(BEAD_RADIUS[0]))]
    weld_distance_columns = [f"BEAD {i+1}" for i in range(WELD_BEAD[0])]
    NEW_SIGMA_DISTANCE = make_row(SIGMA_DISTANCE)
    NEW_BEAD_EACH_DISTANCE = make_row(BEAD_EACH_DISTANCE)
    NEW_WELD_SIGMA_DISTANCE = make_row(WELD_SIGMA_DISTANCE)
    NEW_WELD_EACH_DISTANCE = make_row(WELD_EACH_DISTANCE)
    bead_each_distance_df = pd.DataFrame(data= NEW_BEAD_EACH_DISTANCE, columns= distance_columns)
    sigma_distance_df = pd.DataFrame(data= NEW_SIGMA_DISTANCE, columns= distance_columns)
    weld_bead_each_distance_df = pd.DataFrame(data= NEW_WELD_EACH_DISTANCE, columns= weld_distance_columns)
    weld_sigma_distance_df = pd.DataFrame(data= NEW_WELD_SIGMA_DISTANCE, columns= weld_distance_columns)
    
    ################ XZ Coordinate (report) #################
    XZ_COORDINATE = []
    for i in range(1, TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        xz_coordinate = []
        for j in range(len(BEAD_RADIUS[i-1])):
            xz_coordinate.append(BEAD_INITIAL_COORDINATE_X[i-1][j])
            xz_coordinate.append(BEAD_INITIAL_COORDINATE_Z[i-1][j])
        XZ_COORDINATE.append(xz_coordinate)
    xz_coordinate_columns = [f"{axis}{i+1}" for i in range(len(BEAD_RADIUS[0])) for axis in ("X", "Z")]
    NEW_XZ_COORDINATE = make_row(XZ_COORDINATE)
    xz_coordinate_df = pd.DataFrame(data= NEW_XZ_COORDINATE, columns= xz_coordinate_columns)

    ################ In/External Radius & Arc ################
    CURVATURE_LIST = []
    for i in range(1, TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        curvature_list = []
        for j in range(len(BEAD_RADIUS[i-1])):
            curvature = round(BEAD_RADIUS[i-1][j]*ta_list[i-1]*to_rad,3)
            curvature_list.append(curvature)
        CURVATURE_LIST.append(curvature_list)
    xy_radius = make_row(BEAD_INITIAL_COORDINATE_X)
    xz_curvature = make_row(CURVATURE_LIST)
    xy_radius_columns = [f"Radius Bead{i+1}" for i in range(len(BEAD_RADIUS[0]))]
    xz_curvature_columns = [f"Curvature Bead{i+1}" for i in range(len(BEAD_RADIUS[0]))]
    xy_radius_df = pd.DataFrame(data= xy_radius, columns = xy_radius_columns)
    xz_curvature_df = pd.DataFrame(data= xz_curvature, columns = xz_curvature_columns)

    ################ E2 Start & End Angle (Report) #################
    e2_start_list = make_row(REPORT_BEAD_E2_START_ANGLE)
    e2_end_list = make_row(REPORT_BEAD_E2_END_ANGLE)
    e2_columns = [f"Bead{i+1}" for i in range(len(BEAD_RADIUS[0]))]
    e2_start_list_df = pd.DataFrame(data= e2_start_list, columns = e2_columns)
    e2_end_list_df = pd.DataFrame(data= e2_end_list, columns = e2_columns)

    #################### Divide SRC File ######################
    FILE_LIST = []
    max_point_check = 54000
    point_check = 0
    TOTAL_BEAD_COUNT = 0
    for i in range(1, TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
        for j in range(len(BEAD_RADIUS[i-1])):
            point_check += POINT_GROUP[i-1][j]
            TOTAL_BEAD_COUNT += 1
            if point_check >= max_point_check:
                if i == TOTAL_LAYER + 1 and j == BEAD_COUNT:
                    pass
                else:
                    FILE_LIST.append(TOTAL_BEAD_COUNT)
                    point_check = 0
    
    ###################### Calculate Bead Volume ######################
    sum_volume = 0
    BEAD_VOLUME = []                              # 2 Dimension of List, [layer][bead_count]
    for i in range(1,TOTAL_LAYER+1): 
        volume_ta = Volume_E1 + (i-1)*Delta_E1
        VOLUME_R = R*np.sin(volume_ta*to_rad)
        volume_height = 2*np.pi*VOLUME_R - RADIAL_HATCHDIS
        volume = round(volume_height*BEAD_AREA*MATERIAL_DENSITY*0.000001, 3)
        sum_volume += volume
        BEAD_VOLUME.append(volume)

    ########################### Toolpath Report #################################
    if repeat_num == 1 and REPORT_TYPE == 1:
        toolpath_report_file_name = os.path.join(folder_path, f"{day.strftime('%Y%m%d')}_Toolpath Report.xlsx")
        with pd.ExcelWriter(toolpath_report_file_name) as writer:
            xz_coordinate_df.to_excel(writer, sheet_name= "XZ_Coordinate", index= False)
            ta_list_df.to_excel(writer, sheet_name= "E1 Angle", index= False)
            xy_radius_df.to_excel(writer, sheet_name= "Layer XYPlane Radius", index= False)
            xz_curvature_df.to_excel(writer, sheet_name= "Layer Curvature", index= False)
            if ADAPTIVE_SLICING == 1:
                ADAPTIVE_SLICING_RADIUS_df.to_excel(writer, sheet_name= "Interpolate & Center Radius", index= False)
            BEAD_RADIUS_df.to_excel(writer, sheet_name= "Bead Radius", index= False)
            e2_start_list_df.to_excel(writer, sheet_name= "E2 Start Anlge", index= False)
            e2_end_list_df.to_excel(writer, sheet_name= "E2 End Angle", index= False)
            bead_each_distance_df.to_excel(writer, sheet_name= "Bead Each Distance", index= False)
            sigma_distance_df.to_excel(writer, sheet_name= "Bead Sigma Distance", index= False)
            weld_bead_each_distance_df.to_excel(writer, sheet_name= "Weld Bead Each Distance", index= False)
            weld_sigma_distance_df.to_excel(writer, sheet_name= " Weld Bead Sigma Distance", index= False)

    ############################### Fueltank Lane ###############################################
    if repeat_num == 0:
        ################################## KUKA SUB CODE ############################################
        text_count = 1
        present_bead_count = 1
        point_sum = 0
        reinforcement_end = False
        for i in range(1,TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
            for j in range(len(BEAD_RADIUS[i-1])):
                if point_sum == 0:
                    file_path = os.path.join(folder_path, 'S' + day.strftime('%Y%m%d') + '_fueltank'+ str(text_count) + '.src')
                    sys.stdout = open(file_path, 'w')
                    f = open(file_path,'w+')
                    print("DEF S",day.strftime('%Y%m%d'),"_fueltank",str(text_count),"()",sep='')
                reversed_j = len(BEAD_RADIUS[i-1]) - 1 - j
                if TOOLPATH == 1:
                    print("\n" ";BEAD No.", j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                    if WORKING_SUPPORT_MODE == 1:
                        if len(BEAD_RADIUS[i-1]) > bead_limit:
                            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                        if len(BEAD_RADIUS[i-1]) == bead_limit and j == 0 and not reinforcement_end:
                            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")
                            reinforcement_end = True
                elif TOOLPATH == 2:
                    print("\n" ";BEAD No.", reversed_j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                    if WORKING_SUPPORT_MODE == 1:
                        if len(BEAD_RADIUS[i-1]) > bead_limit:
                            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][reversed_j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                        if len(BEAD_RADIUS[i-1]) == bead_limit and j == 0 and not reinforcement_end:
                            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")
                            reinforcement_end = True
                elif TOOLPATH == 3:
                    if i % 2 == 1:
                        print("\n" ";BEAD No.", j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        if WORKING_SUPPORT_MODE == 1:
                            if len(BEAD_RADIUS[i-1]) > bead_limit:
                                print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                            if len(BEAD_RADIUS[i-1]) == bead_limit and j == 0 and not reinforcement_end:
                                print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")
                                reinforcement_end = True
                    else:
                        print("\n" ";BEAD No.", reversed_j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        if WORKING_SUPPORT_MODE == 1:
                            if len(BEAD_RADIUS[i-1]) > bead_limit:
                                print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][reversed_j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                            if len(BEAD_RADIUS[i-1]) == bead_limit and j == 0 and not reinforcement_end:
                                print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")
                                reinforcement_end = True
                elif TOOLPATH == 4:
                    if i % 2 == 1:
                        print("\n" ";BEAD No.", reversed_j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        if WORKING_SUPPORT_MODE == 1:
                            if len(BEAD_RADIUS[i-1]) > bead_limit:
                                print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][reversed_j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                            if len(BEAD_RADIUS[i-1]) == bead_limit and j == 0 and not reinforcement_end:
                                print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")
                                reinforcement_end = True
                    else:
                        print("\n" ";BEAD No.", j+1, "\n;BEAD:",i,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        if WORKING_SUPPORT_MODE == 1:
                            if len(BEAD_RADIUS[i-1]) > bead_limit:
                                print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                            if len(BEAD_RADIUS[i-1]) == bead_limit and j == 0 and not reinforcement_end:
                                print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")
                                reinforcement_end = True
                elif TOOLPATH == 5:
                    if len(BEAD_RADIUS[i-1]) > bead_limit:
                        print("\n" ";BEAD No.", j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        if WORKING_SUPPORT_MODE == 1:
                            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                    else:
                        if i % 2 == 1:
                            print("\n" ";BEAD No.", j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        else:
                            print("\n" ";BEAD No.", reversed_j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                    if WORKING_SUPPORT_MODE == 1:
                        if len(BEAD_RADIUS[i-1]) == bead_limit and j == 0 and not reinforcement_end:
                            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")
                            reinforcement_end = True
                elif TOOLPATH == 6:
                    if REINFORCEMENT_TYPE == 2 or REINFORCEMENT_TYPE == 3:
                        if i > bead_limit:
                            print("\n" ";BEAD No.", int(CUSTOM_TOOLPATH[j])+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        else:
                            print("\n" ";BEAD No.", j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                    else:
                        print("\n" ";BEAD No.", int(CUSTOM_TOOLPATH[j])+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                    if WORKING_SUPPORT_MODE == 1:
                        print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                elif TOOLPATH == 7:
                    if REINFORCEMENT_TYPE == 2 or REINFORCEMENT_TYPE == 3:
                        if i > bead_limit:
                            if i % 2 == 1:
                                print("\n" ";BEAD No.", int(CUSTOM_TOOLPATH[j])+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                            else:
                                print("\n" ";BEAD No.", int(CCUSTOM_TOOLPATH[j])+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        else:
                            print("\n" ";BEAD No.", j+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                    else:
                        if i % 2 == 1:
                            print("\n" ";BEAD No.", int(CUSTOM_TOOLPATH[j])+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER)
                        else:
                            print("\n" ";BEAD No.", int(CCUSTOM_TOOLPATH[j])+1, "\n;BEAD:",present_bead_count,"/",TOTAL_BEAD_COUNT, "LAYER:", i,"/",TOTAL_LAYER + ADDITIONAL_LAYER) 
                    if WORKING_SUPPORT_MODE == 1:
                        print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][j]):.3f}",",Y 1.90,Z 84.33,A -90.000,B 0.000,C 90.000}")
                percent_time = round((float(SIGMA_TIME[i-1][j])/Fueltank_time*100),3)
                print(";Progress Time=",BEAD_EACH_TIME[i-1][3*j],"h",BEAD_EACH_TIME[i-1][3*j+1],"m",BEAD_EACH_TIME[i-1][3*j+2],"s /",
                    "Progress Percent=",percent_time,"% /", "Progress Diameter=",BEAD_INITIAL_COORDINATE_X[i-1][j],"mm /", "Progress Distance=",BEAD_EACH_DISTANCE[i-1][j],"mm")
                if present_bead_count == 1:
                        None
                else:
                    print("\n" "FEEDER_READY()" "\n" "LASER_READY()" "\n")
                # print("DO_USER_TRIGGER = TRUE")  20250117 Delete

                ######## Start Coordinate #########
                print("SLIN{X",float(BEAD_START_COORDINATE_X[i-1][j]),",Y",float(BEAD_START_COORDINATE_Y[i-1][j]),",Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),
                    ",A",f"{(90-float(BEAD_E2_START_ANGLE[i-1][j])):.3f}",",B 0.000,C",f"{float(E1_ANGLE[i-1]) - C_ANGLE[i-1]:.3f}",",S 6, T 26",",E1",f"{float(E1_ANGLE[i-1]) + WRONG_E1:.3f}",
                    ",E2",float(BEAD_E2_START_ANGLE[i-1][j]),"}")
                ####################################

                if TOOLPATH == 1:
                    print("LAYER_COUNT =",i,sep='')
                    print("BEAD_NUMBER =",j+1,sep='')
                elif TOOLPATH == 2:
                    print("LAYER_COUNT =",i,sep='')
                    print("BEAD_NUMBER =",reversed_j+1,sep='')
                elif TOOLPATH == 3:
                    if i % 2 == 1:
                        print("LAYER_COUNT =",i,sep='')
                        print("BEAD_NUMBER =",j+1,sep='')
                    else:
                        print("LAYER_COUNT =",i,sep='')
                        print("BEAD_NUMBER =",reversed_j+1,sep='')
                elif TOOLPATH == 4:
                    if i % 2 == 1:
                        print("LAYER_COUNT =",i,sep='')
                        print("BEAD_NUMBER =",reversed_j+1,sep='')
                    else:
                        print("LAYER_COUNT =",i,sep='')
                        print("BEAD_NUMBER =",j+1,sep='')
                elif TOOLPATH == 5:
                    print("LAYER_COUNT =",i,sep='')
                elif TOOLPATH == 6:
                    print("LAYER_COUNT =",i,sep='')
                    if REINFORCEMENT_TYPE == 2 or REINFORCEMENT_TYPE == 3:
                        if i > bead_limit:
                            print("BEAD_NUMBER =",int(CUSTOM_TOOLPATH[j])+1,sep='')
                        else:
                            print("BEAD_NUMBER =",j+1,sep='')
                    else:
                        print("BEAD_NUMBER =",int(CUSTOM_TOOLPATH[j])+1,sep='')
                elif TOOLPATH == 7:
                    print("LAYER_COUNT =",i,sep='')
                    if REINFORCEMENT_TYPE == 2 or REINFORCEMENT_TYPE == 3:
                        if i > bead_limit:
                            if i % 2 == 1:
                                print("BEAD_NUMBER =",int(CUSTOM_TOOLPATH[j])+1,sep='')
                            else:
                                print("BEAD_NUMBER =",int(CCUSTOM_TOOLPATH[j])+1,sep='')
                        else:
                            print("BEAD_NUMBER =",j+1,sep='')
                    else:
                        if i % 2 == 1:
                            print("BEAD_NUMBER =",int(CUSTOM_TOOLPATH[j])+1,sep='')
                        else:
                            print("BEAD_NUMBER =",int(CCUSTOM_TOOLPATH[j])+1,sep='')
                if j == 0:
                    print("WAIT FOR DI_CHECK_WD_ASSIGNED")
                    if TRAVEL_SPEED == AIR_TRAVEL_SPEED:
                        None
                    else: 
                        print("$VEL.CP=",AIR_SPEED)

                    if SCANNER_SENSOR == 1:
                        if present_bead_count != 1:
                            ######### 3D Scanner Coordinate ##########
                            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE + 250:.3f}",",Y -348.10,Z 83.82, A -90.000,B -0.000,C 90.000}")
                            print("SLIN{X",float(BEAD_START_COORDINATE_X[i-1][j]),",Y",float(BEAD_START_COORDINATE_Y[i-1][j]),",Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),
                                ",A",f"{(90-float(BEAD_E2_START_ANGLE[i-1][j])):.3f}",",B 0.000,C 0.000,S 6,T 26,E1 0.000,E2",float(BEAD_E2_START_ANGLE[i-1][j]),"}")
                            print("SLIN{X 0.000,Y 0.000,Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),",A",f"{(90-float(BEAD_E2_START_ANGLE[i-1][j])):.3f}",
                                ",B 0.000,C 0.000,S 6,T 26,E1 0.000,E2",float(BEAD_E2_START_ANGLE[i-1][j]),"}")
                            ############# Trigger #################
                            print("WAIT SEC 0.0")
                            print("TD_CAMERA()")

                    if DEFORMATION_SENSOR == 1:
                        ####### Deformation Sensor Coordinate #####
                        print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 77.97,Z 74.89,A -90.000,B -0.000,C 90.000}")
                        print("SLIN{X",float(BEAD_START_COORDINATE_X[i-1][j]),",Y",float(BEAD_START_COORDINATE_Y[i-1][j]),",Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),
                            ",A",f"{(90-float(BEAD_E2_START_ANGLE[i-1][j])):.3f}",",B 0.000,C",f"{float(E1_ANGLE[i-1]) - C_ANGLE[i-1]:.3f}",",S 6, T 26",",E1",f"{float(E1_ANGLE[i-1]) + WRONG_E1:.3f}",
                            ",E2",float(BEAD_E2_START_ANGLE[i-1][j]),"}")
                        ############# Trigger #################
                        print("WD_CHECK()")

                    if DEFORMATION_SENSOR == 1 or SCANNER_SENSOR == 1:
                        ######## Return Start Coordinate #########
                        print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")
                        # print("SLIN{X",float(BEAD_START_COORDINATE_X[i-1][j]),",Y",float(BEAD_START_COORDINATE_Y[i-1][j]),",Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),
                        #     ",A",f"{(90-float(BEAD_E2_START_ANGLE[i-1][j])):.3f}",",B 0.000,C 0.000,S 6,T 26,E1 0.000,E2",float(BEAD_E2_START_ANGLE[i-1][j]),"}")
                        print("SLIN{X",float(BEAD_START_COORDINATE_X[i-1][j]),",Y",float(BEAD_START_COORDINATE_Y[i-1][j]),",Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),
                            ",A",f"{(90-float(BEAD_E2_START_ANGLE[i-1][j])):.3f}",",B 0.000,C",f"{float(E1_ANGLE[i-1]) - C_ANGLE[i-1]:.3f}",",S 6, T 26",",E1",f"{float(E1_ANGLE[i-1]) + WRONG_E1:.3f}",
                            ",E2",float(BEAD_E2_START_ANGLE[i-1][j]),"}")
                        ###################################

                print("Bead_",present_bead_count,":",sep='')        
                print(";Code Statement")
                print("SWITCH WORK_CHECK")
                if point_sum == 0:
                        if present_bead_count == 1:
                            pass
                        else:
                            # print("    CASE 1")            # It can't open other file
                            # print("        CALL S",day.strftime('%Y%m%d'),"_fueltank",str(text_count-1),"_marker()",sep='')   
                            pass
                else:
                    print("    CASE 1")
                    print("        GOTO Bead_",(present_bead_count-1),sep='')
                if present_bead_count in FILE_LIST:
                        pass
                        # print("    CASE 2")                # It can't open other file
                        # print("        CALL S",day.strftime('%Y%m%d'),"_fueltank",str(text_count+1),"_marker()",sep='')
                else:        
                    if present_bead_count == TOTAL_BEAD_COUNT:
                        pass
                    else:
                        print("    CASE 2")
                        print("        GOTO Bead_",(present_bead_count+1),sep='')
                print("ENDSWITCH")
                if present_bead_count == 1:
                        print("HALT")
                else:
                    print("DO_OPTIONAL_STOP()")
                if j == 0:
                    print("LAYER_START = TRUE")
                print("$VEL.CP= TRAVEL_SPEED/60000" "\n")
                print("LOAD_SETPOINT = TRUE \nWAIT SEC 0.5 \nLOAD_SETPOINT = FALSE")
                print("TRIGGER WHEN DISTANCE = 0 DELAY = F_ON_MS DO DO_FD_START_FORWARD = TRUE" "\n" "TRIGGER WHEN DISTANCE = 0 DELAY = L_ON_MS DO DO_LASER_MODULATION = TRUE" "\n") # 20250117 Change

                ######### Among Coordinate ########
                for k in range(POINT_GROUP[i-1][j]):
                    print("SLIN{X",float(BEAD_AMONG_COORDINATE_X[i-1][j][k]),",Y",float(BEAD_AMONG_COORDINATE_Y[i-1][j][k]),",Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),
                        ",A",f"{(90-float(BEAD_E2_AMONG_ANGLE[i-1][j][k])):.3f}",",B 0.000,C",f"{float(E1_ANGLE[i-1]) - C_ANGLE[i-1]:.3f}",",S 6, T 26",",E1",f"{float(E1_ANGLE[i-1]) + WRONG_E1:.3f}",
                        ",E2",float(BEAD_E2_AMONG_ANGLE[i-1][j][k]),"}C_DIS")
                ###################################

                ######### End Coordinate ##########
                print("SLIN{X",float(BEAD_END_COORDINATE_X[i-1][j]),",Y",float(BEAD_END_COORDINATE_Y[i-1][j]),",Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),
                    ",A",f"{(90-float(BEAD_E2_END_ANGLE[i-1][j])):.3f}",",B 0.000,C",f"{float(E1_ANGLE[i-1]) - C_ANGLE[i-1]:.3f}",",S 6, T 26",",E1",f"{float(E1_ANGLE[i-1]) + WRONG_E1:.3f}",
                    ",E2",float(BEAD_E2_END_ANGLE[i-1][j]),"}C_DIS")
                ###################################

                print("END_TRIGGER()" "\n")
                if j == len(BEAD_RADIUS[i-1])-1:
                    print("LAYER_START = FALSE")
                if TRAVEL_SPEED == AIR_TRAVEL_SPEED:
                        None
                else: 
                    print("$VEL.CP=",AIR_SPEED)
                
                ######### Air Coordinate #########
                print("SLIN{X",float(BEAD_AIR_COORDINATE_X[i-1][j]),",Y",float(BEAD_AIR_COORDINATE_Y[i-1][j]),",Z",float(BEAD_AIR_COORDINATE_Z[i-1][j]),
                    ",A",f"{(90-float(BEAD_E2_END_ANGLE[i-1][j])):.3f}",",B 0.000,C",f"{float(E1_ANGLE[i-1]) - C_ANGLE[i-1]:.3f}",",S 6, T 26",",E1",f"{float(E1_ANGLE[i-1]) + WRONG_E1:.3f}",
                    ",E2",float(BEAD_E2_END_ANGLE[i-1][j]),"}C_DIS")
                ##################################

                point_sum += POINT_GROUP[i-1][j]
                if present_bead_count in FILE_LIST:
                    # print("DEF S",day.strftime('%Y%m%d'),"_fueltank",str(text_count),"_marker()",sep='')
                    # print("        GOTO Layer_",(present_bead_count),sep='')
                    print("END")
                    sys.stdout.close()
                    text_count += 1
                    point_sum = 0
                present_bead_count += 1
        if present_bead_count > FILE_LIST[-1] + 1:
            print("END")
        sys.stdout.close()

        ####################################  KUKA Main Test Code #######################################
        if TEST_SRC == 1:
            file_path = os.path.join(folder_path, 'M' + day.strftime('%Y%m%d') + '_fueltanktest' + '.src')
            sys.stdout = open(file_path, 'w')
            f = open(file_path,'w+')
            print("DEF MOON()")
            print("\n""\n"";############### PROCESS PARAMETER #################")
            print(";INNER RADIUS=",R,"mm", "\n" ";PIPE DIAMETER=",D,"mm", "\n\n" "TRAVEL_SPEED=",TRAVEL_SPEED,";mm/min","\n" "WIRE_FEEDRATE=",WIRE_FEEDRATE,";m/min","\n" 
                ";LASER POWER=",LASER_POWER,"W\n\n" ";TARGET THICKNESS=",BEAD_WIDTH,"mm\n" ";LAYER THICKNESS=",LAYER_THICKNESS ,"mm\n" ";BEAD WIDTH=",BEAD_WIDTH,"mm\n" 
                ";RADIAL OVERLAP(RADIAL HATCHDIS)=",RADIAL_OVERLAP,"%","(",np.abs(RADIAL_HATCHDIS),"mm)""\n"";AXIAL OVERLAP(AIXIAL HATCHDIS)=",AXIAL_OVERLAP,"%","(",np.abs(AXIAL_HATCHDIS),"mm)\n\n"
                ";EXPECT THICKNESS=",BEAD_EXPECT_WIDTH,"mm\n"";EXPECT WEIGHT=", f"{sum_volume:.3f} kg\n\n" ";TOTAL TIME =",time_h,"h",time_mm,"m",time_s,"s\n"";FUELTANK TOTAL TIME =",fuel_time_h,"h",fuel_time_mm,"m",fuel_time_s,"s\n"
                ";TOTAL BEAD=", TOTAL_BEAD_COUNT,"EA")
            if SPHERE_TYPE == 1:
                print(";TOTAL LAYER=", TOTAL_LAYER,"EA\n"";SPHERE TYPE= Hemisphere")
                if ADDITIONAL_LAYER != 0:
                    print(";TOTAL LAYER=", TOTAL_LAYER + ADDITIONAL_LAYER,"(",TOTAL_LAYER,"+",ADDITIONAL_LAYER,")","EA\n"";SPHERE TYPE= Hemisphere")
            else:
                print(";HALF LAYER=", HALF_TOTAL_LAYER,"layers ""/ TOTAL LAYER=", TOTAL_LAYER,"EA\n"";SPHERE TYPE= Sphere")
            CUSTOM_TOOLPATH = [x + 1 for x in CUSTOM_TOOLPATH]
            CCUSTOM_TOOLPATH = [x + 1 for x in CCUSTOM_TOOLPATH]
            Custom_Toolpath_Format = ";TOOLPATH = " + "-".join(map(str, CUSTOM_TOOLPATH))
            Ccustom_Toolpath_Format = ";TOOLPATH = " + "-".join(map(str, [*CUSTOM_TOOLPATH, *CCUSTOM_TOOLPATH]))
            if ADAPTIVE_SLICING == 1:
                print(";SLICING = Adaptive Slicing")
            if BEAD_COUNT >= 2:
                if TOOLPATH == 1:
                    print(";TOOLPATH = 1-2-1-2")
                elif TOOLPATH == 2:
                    print(";TOOLPATH = 2-1-2-1")
                elif TOOLPATH == 3:
                    print(";TOOLPATH = 1-2-2-1")
                elif TOOLPATH == 4:
                    print(";TOOLPATH = 2-1-1-2") 
                elif TOOLPATH == 5:
                    print(";TOOLPATH = 1-2-1-2(Ladder)")
                elif TOOLPATH == 6:
                    print(Custom_Toolpath_Format)
                elif TOOLPATH == 7:
                    print(Ccustom_Toolpath_Format)
            if REINFORCEMENT_TYPE == 1:
                print(";REINFORCEMENT TYPE= None")
            elif REINFORCEMENT_TYPE == 2:
                print(";REINFORCEMENT TYPE= In & External")
            elif REINFORCEMENT_TYPE == 3:
                print(";REINFORCEMENT TYPE= External")
            print("\n"";############ SUB PROCESS PARAMETER ###############")
            print("LASER_OFF()""\n""FEEDER_OFF()")
            print("ROBOT_SPEED= TRAVEL_SPEED/60000" "\n" "FEEDER_SPEED= WIRE_FEEDRATE*6553.5" "\n" "AO_LASER_POWER=",AO_LASER_POWER,"\n""\n")
            print(";FOLD ============== CONTROL PARAMETERS ===============")
            print("    WIRE_CONTROL = TRUE")
            print("    DEPOSITION_START = FALSE")
            print("    LAYER_START = FALSE")
            print("    GET_WORKING_DISTANCE = FALSE")
            print("    GET_3D_IMAGE = FALSE")
            print(";ENDFOLD (CONTROL PARAMETERS)")
            print("    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )")
            print("    INTERRUPT ON 3")
            print("    BAS (#INITMOV,0 )")
            # print("$BASE=EK({X 1113.87,Y -292.46,Z -397.81,A 160.30,B -0.34,C 0.05},#EASYS,{X 0.000,Y 0.000,Z 104.000,A 0.000,B 0.000,C 0.000})")                         # Coordinate when Laser Head go to Original Coordinate (202501~)
            print("$BASE=EK({X 1115.744,Y -291.584,Z -395.744,A 160.30,B -0.33,C 0.00},#EASYS,{X 0.000,Y 0.000,Z 104.000,A 0.000,B 0.000,C 0.000})") 
            print("$IPO_MODE=#BASE")
            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")                                                                   # Coordinate, Tool Position
            print("PTP $POS_ACT")
            print("$VEL.CP= ROBOT_SPEED")                           
            print("SLIN {X",f"{initial_X1:.3f}",",Y",f"{initial_Z1:.3f}",",Z 125.070,A",f"{initial_a:.3f}",",B 0.000,C",f"{initial_e1:.3f}",",S 6,T 26,E1",f"{initial_e1:.3f}",",E2",f"{-START_THETA:.3f}",",E3 0.000,E4 0.000,E5 0.000,E6 0.000} C_DIS")
            print("FEEDER_READY()" "\n" "LASER_READY()")
            print(" DEPOSITION_START = TRUE  ; NOTICE PROCESS START TO MONITORING SW" "\n" "LAYER_START = TRUE")
            # for i in range(1, TOTAL_LAYER + 1 + ADDITIONAL_LAYER):
            #     for j in range(len(BEAD_RADIUS[i-1])):
            #         print("PTP $POS_ACT")
            #         print(";LAYER COUNT =", i)
            #         if WORKING_SUPPORT_MODE == 1:
            #             print("$TOOL={X",f"{HEAD_WORKING_DISTANCE+float(BEAD_WORKING_DISTANCE[i-1][j]):.3f}",",Y -1.730,Z 83.820,A -90.000,B -0.000,C 90.000}")
            #         print("SLIN{X",float(BEAD_INITIAL_COORDINATE_X[i-1][j]),",Y 0.000,Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][j]),
            #             ",A 90.000,B 0.000,C",f"{float(E1_ANGLE[i-1]) - C_ANGLE[i-1]:.3f}",",S 6, T 26",",E1",f"{float(E1_ANGLE[i-1]) + WRONG_E1:.3f}",
            #             ",E2 0.000}")
            #     print("HALT")
            for i in range(1, TOTAL_LAYER + 2, 10):
                print("PTP $POS_ACT")
                print(";LAYER COUNT =", i)
                print("SLIN{X",float(BEAD_INITIAL_COORDINATE_X[i-1][0]),",Y 0.000,Z",float(BEAD_INITIAL_COORDINATE_Z[i-1][0]),
                    ",A 90.000,B 0.000,C",f"{float(E1_ANGLE[i-1]) - C_ANGLE[i-1]:.3f}",",S 6, T 26",",E1",f"{float(E1_ANGLE[i-1]) + WRONG_E1:.3f}",
                    ",E2 0.000}")
                print("HALT")
            print("END")
            sys.stdout.close()

        ####################################  KUKA Main Code #######################################
        file_path = os.path.join(folder_path, 'M' + day.strftime('%Y%m%d') + '_fueltank' + '.src' )
        sys.stdout = open(file_path, 'w')
        f = open(file_path,'w+')
        print("DEF MOON()")
        print("\n""\n"";############### PROCESS PARAMETER #################")
        print(";INNER RADIUS=",R,"mm", "\n" ";PIPE DIAMETER=",D,"mm", "\n\n" "TRAVEL_SPEED=",TRAVEL_SPEED,";mm/min","\n" "WIRE_FEEDRATE=",WIRE_FEEDRATE,";m/min","\n" 
            ";LASER POWER=",LASER_POWER,"W\n\n" ";TARGET THICKNESS=",BEAD_WIDTH,"mm\n" ";LAYER THICKNESS=",LAYER_THICKNESS ,"mm\n" ";BEAD WIDTH=",BEAD_WIDTH,"mm\n" 
            ";RADIAL OVERLAP(RADIAL HATCHDIS)=",RADIAL_OVERLAP,"%","(",np.abs(RADIAL_HATCHDIS),"mm)""\n"";AXIAL OVERLAP(AIXIAL HATCHDIS)=",AXIAL_OVERLAP,"%","(",np.abs(AXIAL_HATCHDIS),"mm)\n\n"
            ";EXPECT THICKNESS=",BEAD_EXPECT_WIDTH,"mm\n"";EXPECT WEIGHT=", f"{sum_volume:.3f} kg\n\n" ";TOTAL TIME(Fueltank + Welding part) =",time_h,"h",time_mm,"m",time_s,"s\n" 
            ";FUELTANK TOTAL TIME =",fuel_time_h,"h",fuel_time_mm,"m",fuel_time_s,"s\n" ";TOTAL BEAD=", TOTAL_BEAD_COUNT,"EA")
        if SPHERE_TYPE == 1:
            print(";TOTAL LAYER=", TOTAL_LAYER,"EA\n"";SPHERE TYPE= Hemisphere")
            if ADDITIONAL_LAYER != 0:
                print(";TOTAL LAYER=", TOTAL_LAYER + ADDITIONAL_LAYER,"(",TOTAL_LAYER,"+",ADDITIONAL_LAYER,")","EA\n"";SPHERE TYPE= Hemisphere")
        else:
            print(";HALF LAYER=", HALF_TOTAL_LAYER,"layers ""/ TOTAL LAYER=", TOTAL_LAYER,"EA\n"";SPHERE TYPE= Sphere")
        Custom_Toolpath_Format = ";TOOLPATH = " + "-".join(map(str, CUSTOM_TOOLPATH))
        Ccustom_Toolpath_Format = ";TOOLPATH = " + "-".join(map(str, [*CUSTOM_TOOLPATH, *CCUSTOM_TOOLPATH]))
        if ADAPTIVE_SLICING == 1:
            print(";SLICING = Adaptive Slicing")
        if BEAD_COUNT >= 2:
            if TOOLPATH == 1:
                print(";TOOLPATH = 1-2-1-2")
            elif TOOLPATH == 2:
                print(";TOOLPATH = 2-1-2-1")
            elif TOOLPATH == 3:
                print(";TOOLPATH = 1-2-2-1")
            elif TOOLPATH == 4:
                print(";TOOLPATH = 2-1-1-2") 
            elif TOOLPATH == 5:
                print(";TOOLPATH = 1-2-1-2(Ladder)")
            elif TOOLPATH == 6:
                print(Custom_Toolpath_Format)
            elif TOOLPATH == 7:
                print(Ccustom_Toolpath_Format)
        if REINFORCEMENT_TYPE == 1:
            print(";REINFORCEMENT TYPE = None")
        elif REINFORCEMENT_TYPE == 2:
            print(";REINFORCEMENT TYPE = In & External")
        elif REINFORCEMENT_TYPE == 3:
            print(";REINFORCEMENT TYPE = External")
        print("\n"";############ SUB PROCESS PARAMETER ###############")
        print("LASER_OFF()""\n""FEEDER_OFF()")
        print("ROBOT_SPEED= TRAVEL_SPEED/60000" "\n" "FEEDER_SPEED= WIRE_FEEDRATE*6553.5" "\n" "AO_LASER_POWER=",AO_LASER_POWER,"\n""\n")
        print(";FOLD ============== CONTROL PARAMETERS ===============")
        print("    WIRE_CONTROL = TRUE")
        print("    DEPOSITION_START = FALSE")
        print("    LAYER_START = FALSE")
        print("    GET_WORKING_DISTANCE = FALSE")
        print("    GET_3D_IMAGE = FALSE")
        print(";ENDFOLD (CONTROL PARAMETERS)")
        print("    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )")
        print("    INTERRUPT ON 3")
        print("    BAS (#INITMOV,0 )")
        # print("$BASE=EK({X 1113.87,Y -292.46,Z -397.81,A 160.30,B -0.34,C 0.05},#EASYS,{X 0.000,Y 0.000,Z 104.000,A 0.000,B 0.000,C 0.000})")                         # Coordinate when Laser Head go to Original Coordinate (202501~)
        print("$BASE=EK({X 1115.744,Y -291.584,Z -395.744,A 160.30,B -0.33,C 0.00},#EASYS,{X 0.000,Y 0.000,Z 104.000,A 0.000,B 0.000,C 0.000})") 
        print("$IPO_MODE=#BASE")
        print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")                                                                   # Coordinate, Tool Position
        print("PTP $POS_ACT")
        print("$VEL.CP= ROBOT_SPEED")                           
        print("SLIN {X",f"{initial_X1:.3f}",",Y",f"{initial_Z1:.3f}",",Z 125.070,A",f"{initial_a:.3f}",",B 0.000,C",f"{initial_e1:.3f}",",S 6,T 26,E1",f"{initial_e1:.3f}",",E2",f"{-START_THETA:.3f}",",E3 0.000,E4 0.000,E5 0.000,E6 0.000} C_DIS")
        print("FEEDER_READY()" "\n" "LASER_READY()")
        print(" DEPOSITION_START = TRUE  ; NOTICE PROCESS START TO MONITORING SW" "\n" "LAYER_START = TRUE")
        for i in range(1,text_count+1):
            print("S",day.strftime('%Y%m%d'),"_fueltank",i,"()",sep='') 
        print("END")
        sys.stdout.close()

        ################################ Welding Reinforcement Test Code #########################
        if WELD_REINFORCEMENT == 1 and WELD_TEST_SRC == 1:
            present_bead_count = 1
            file_path = os.path.join(folder_path, 'M' + day.strftime('%Y%m%d') + '_weldingtest' + '.src')
            sys.stdout = open(file_path, 'w')
            f = open(file_path,'w+')
            print("DEF MOON()")
            print("\n""\n"";############### PROCESS PARAMETER #################")
            print(";INNER RADIUS=",R,"mm", "\n" ";PIPE DIAMETER=",D,"mm", "\n\n" "TRAVEL_SPEED=",TRAVEL_SPEED,";mm/min","\n" "WIRE_FEEDRATE=",WIRE_FEEDRATE,";m/min","\n" 
                ";LASER POWER=",LASER_POWER,"W\n\n" ";TARGET THICKNESS=",BEAD_WIDTH,"mm\n" ";LAYER THICKNESS=",LAYER_THICKNESS ,"mm\n" ";BEAD WIDTH=",BEAD_WIDTH,"mm\n" 
                ";RADIAL OVERLAP(RADIAL HATCHDIS)=",RADIAL_OVERLAP,"%","(",np.abs(RADIAL_HATCHDIS),"mm)""\n"";AXIAL OVERLAP(AIXIAL HATCHDIS)=",AXIAL_OVERLAP,"%","(",np.abs(AXIAL_HATCHDIS),"mm)\n\n"
                ";WELD REINFORCEMENT DISTANCE=",WELD_REINFORCEMENT_DIS,"mm\n",";TOTAL BEAD=",WELD_TOTAL_BEAD_COUNT,"EA")
            print("\n"";############ SUB PROCESS PARAMETER ###############")
            print("LASER_OFF()""\n""FEEDER_OFF()")
            print("ROBOT_SPEED= TRAVEL_SPEED/60000" "\n" "FEEDER_SPEED= WIRE_FEEDRATE*6553.5" "\n" "AO_LASER_POWER=",AO_LASER_POWER,"\n""\n")
            print(";FOLD ============== CONTROL PARAMETERS ===============")
            print("    WIRE_CONTROL = TRUE")
            print("    DEPOSITION_START = FALSE")
            print("    LAYER_START = FALSE")
            print("    GET_WORKING_DISTANCE = FALSE")
            print("    GET_3D_IMAGE = FALSE")
            print(";ENDFOLD (CONTROL PARAMETERS)")
            print("    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )")
            print("    INTERRUPT ON 3")
            print("    BAS (#INITMOV,0 )")
            # print("$BASE=EK({X 1113.87,Y -292.46,Z -397.81,A 160.30,B -0.34,C 0.05},#EASYS,{X 0.000,Y 0.000,Z 104.000,A 0.000,B 0.000,C 0.000})")                         # Coordinate when Laser Head go to Original Coordinate (202501~)
            print("$BASE=EK({X 1115.744,Y -291.584,Z -395.744,A 160.30,B -0.33,C 0.00},#EASYS,{X 0.000,Y 0.000,Z 104.000,A 0.000,B 0.000,C 0.000})") 
            print("$IPO_MODE=#BASE")
            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")                                                                   # Coordinate, Tool Position
            print("$VEL.CP= ROBOT_SPEED")                           

            for i in range(WELD_REINFORCE_LAYER):
                for j in range(WELD_BEAD[i]):
                    ######## Start Coordinate #########
                    print("SLIN{X",float(WELD_START_X[i][j]),",Y 0.000",",Z",float(WELD_START_Z[i][j]),
                        ",A 90.000",",B 0.000,C",float(WELD_E1_LIST[i][j]),",S 6, T 26",",E1",f"{float(WELD_E1_LIST[i][j]) + WRONG_E1:.3f}",
                        ",E2 0.000}")
                    ####################################
            print("END")
            sys.stdout.close()

        ################################ Welding Reinforcement Code #########################
        if WELD_REINFORCEMENT == 1:
            present_bead_count = 1
            file_path = os.path.join(folder_path, 'M' + day.strftime('%Y%m%d') + '_welding' + '.src')
            sys.stdout = open(file_path, 'w')
            f = open(file_path,'w+')
            print("DEF MOON()")
            print("\n""\n"";############### PROCESS PARAMETER #################")
            print(";INNER RADIUS=",R,"mm", "\n" ";PIPE DIAMETER=",D,"mm", "\n\n" "TRAVEL_SPEED=",TRAVEL_SPEED,";mm/min","\n" "WIRE_FEEDRATE=",WIRE_FEEDRATE,";m/min","\n" 
                ";LASER POWER=",LASER_POWER,"W\n\n" ";TARGET THICKNESS=",BEAD_WIDTH,"mm\n" ";LAYER THICKNESS=",LAYER_THICKNESS ,"mm\n" ";BEAD WIDTH=",BEAD_WIDTH,"mm\n" 
                ";RADIAL OVERLAP(RADIAL HATCHDIS)=",RADIAL_OVERLAP,"%","(",np.abs(RADIAL_HATCHDIS),"mm)""\n"";AXIAL OVERLAP(AIXIAL HATCHDIS)=",AXIAL_OVERLAP,"%","(",np.abs(AXIAL_HATCHDIS),"mm)\n\n"
                ";WELD REINFORCEMENT DISTANCE=",WELD_REINFORCEMENT_DIS,"mm\n",";TOTAL BEAD=",WELD_TOTAL_BEAD_COUNT,"EA")
            print(";TOTAL TIME =",weld_time_h,"h",weld_time_mm,"m",weld_time_s,"s\n")
            print("\n"";############ SUB PROCESS PARAMETER ###############")
            print("LASER_OFF()""\n""FEEDER_OFF()")
            print("ROBOT_SPEED= TRAVEL_SPEED/60000" "\n" "FEEDER_SPEED= WIRE_FEEDRATE*6553.5" "\n" "AO_LASER_POWER=",AO_LASER_POWER,"\n""\n")
            print(";FOLD ============== CONTROL PARAMETERS ===============")
            print("    WIRE_CONTROL = TRUE")
            print("    DEPOSITION_START = FALSE")
            print("    LAYER_START = FALSE")
            print("    GET_WORKING_DISTANCE = FALSE")
            print("    GET_3D_IMAGE = FALSE")
            print(";ENDFOLD (CONTROL PARAMETERS)")
            print("    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )")
            print("    INTERRUPT ON 3")
            print("    BAS (#INITMOV,0 )")
            # print("$BASE=EK({X 1113.87,Y -292.46,Z -397.81,A 160.30,B -0.34,C 0.05},#EASYS,{X 0.000,Y 0.000,Z 104.000,A 0.000,B 0.000,C 0.000})")                         # Coordinate when Laser Head go to Original Coordinate (202501~)
            print("$BASE=EK({X 1115.744,Y -291.584,Z -395.744,A 160.30,B -0.33,C 0.00},#EASYS,{X 0.000,Y 0.000,Z 104.000,A 0.000,B 0.000,C 0.000})") 
            print("$IPO_MODE=#BASE")
            print("$TOOL={X",f"{HEAD_WORKING_DISTANCE:.3f}",",Y 1.90,Z 84.33,A -90.00,B 0.00,C 90.00}")                                                                   # Coordinate, Tool Position
            print("PTP $POS_ACT")
            print("$VEL.CP= ROBOT_SPEED")  
            print("SLIN{X",f"{1.3*float(WELD_START_COORDINATE_X[0][0]):.3f}",",Y",f"{1.3*float(WELD_START_COORDINATE_Y[0][0]):.3f}",",Z",float(WELD_START_Z[0][0]),
                        ",A",f"{(90-float(WELD_E2_START_ANGLE[0][0])):.3f}",",B 0.000,C",float(WELD_E1_LIST[0][0]),",S 6, T 26",",E1",float(WELD_E1_LIST[0][0]),
                        ",E2",float(WELD_E2_START_ANGLE[0][0]),"}")                         
            print("FEEDER_READY()" "\n" "LASER_READY()")
            print("DEPOSITION_START = TRUE  ; NOTICE PROCESS START TO MONITORING SW" "\n" "LAYER_START = TRUE")

            for i in range(WELD_REINFORCE_LAYER):
                for j in range(WELD_BEAD[i]):
                    weld_percent_time = round(WELD_SIGMA_TIME[i][j]/weld_sum_time*100,3)
                    print("\n" ";BEAD No.", j+1, "\n;BEAD:",present_bead_count,"/",WELD_TOTAL_BEAD_COUNT, "LAYER:", i+1,"/",WELD_REINFORCE_LAYER)
                    print(";Progress Time=",WELD_BEAD_EACH_TIME[i][3*j],"h",WELD_BEAD_EACH_TIME[i][3*j+1],"m",WELD_BEAD_EACH_TIME[i][3*j+2],"s /", 
                          "Progress Percent=",weld_percent_time,"% /", "Progress Diameter=",WELD_START_X[i][j],"mm /", "Progress Distance=",WELD_EACH_DISTANCE[i][j],"mm")
                    
                    print("\n" "FEEDER_READY()" "\n" "LASER_READY()" "\n")
                    # print("DO_USER_TRIGGER = TRUE")  # 20250117 Delete

                    ######## Start Coordinate #########
                    print("SLIN{X",float(WELD_START_COORDINATE_X[i][j]),",Y",float(WELD_START_COORDINATE_Y[i][j]),",Z",float(WELD_START_Z[i][j]),
                        ",A",f"{(90-float(WELD_E2_START_ANGLE[i][j])):.3f}",",B 0.000,C",float(WELD_E1_LIST[i][j]),",S 6, T 26",",E1",f"{float(WELD_E1_LIST[i][j]) + WRONG_E1:.3f}",
                        ",E2",float(WELD_E2_START_ANGLE[i][j]),"}")
                    ####################################

                    print("LAYER_COUNT =",i+1,sep='')
                    print("BEAD_NUMBER =",j+1,sep='')
                    # if j == 0:
                    #     print("WD_CHECK()")          # Collision
                    if present_bead_count == 1:
                        print("HALT")
                    else:
                        print("DO_OPTIONAL_STOP()")
                    if j == 0:
                        print("LAYER_START = TRUE")  
                    print("$VEL.CP= TRAVEL_SPEED/60000" "\n")
                    print("LOAD_SETPOINT = TRUE\nWAIT SEC 0.5\nLOAD_SETPOINT = FALSE")
                    print("TRIGGER WHEN DISTANCE = 0 DELAY = F_ON_MS DO DO_FD_START_FORWARD = TRUE" "\n" "TRIGGER WHEN DISTANCE = 0 DELAY = L_ON_MS DO DO_LASER_MODULATION = TRUE" "\n")

                    ######### Among Coordinate ########
                    for k in range(WELD_POINT_GROUP[i][j]):
                        print("SLIN{X",float(WELD_AMONG_COORDINATE_X[i][j][k]),",Y",float(WELD_AMONG_COORDINATE_Y[i][j][k]),",Z",float(WELD_START_Z[i][j]),
                            ",A",f"{(90-float(WELD_E2_AMONG_ANGLE[i][j][k])):.3f}",",B 0.000,C",float(WELD_E1_LIST[i][j]),",S 6, T 26",",E1",f"{float(WELD_E1_LIST[i][j]) + WRONG_E1:.3f}",
                            ",E2",float(WELD_E2_AMONG_ANGLE[i][j][k]),"}C_DIS")
                    ###################################

                    ######### End Coordinate ##########
                    print("SLIN{X",float(WELD_END_COORDINATE_X[i][j]),",Y",float(WELD_END_COORDINATE_Y[i][j]),",Z",float(WELD_START_Z[i][j]),
                        ",A",f"{(90-float(WELD_E2_END_ANGLE[i][j])):.3f}",",B 0.000,C",float(WELD_E1_LIST[i][j]),",S 6, T 26",",E1",f"{float(WELD_E1_LIST[i][j]) + WRONG_E1:.3f}",
                        ",E2",float(WELD_E2_END_ANGLE[i][j]),"}C_DIS")
                    ###################################

                    print("END_TRIGGER()" "\n")
                    if j == WELD_BEAD[i]-1:
                        print("LAYER_START = FALSE")
                    if TRAVEL_SPEED == AIR_TRAVEL_SPEED:
                            None
                    else: 
                        print("$VEL.CP=",AIR_SPEED)
                    
                    ######### Air Coordinate #########
                    print("SLIN{X",f"{1.1*float(WELD_END_COORDINATE_X[i][j]):.3f}",",Y",f"{1.1*float(WELD_END_COORDINATE_Y[i][j]):.3f}",",Z",float(WELD_START_Z[i][j]),
                        ",A",f"{(90-float(WELD_E2_END_ANGLE[i][j])):.3f}",",B 0.000,C",float(WELD_E1_LIST[i][j]),",S 6, T 26",",E1",f"{float(WELD_E1_LIST[i][j]) + WRONG_E1:.3f}",
                        ",E2",float(WELD_E2_END_ANGLE[i][j]),"}C_DIS")
                    ###################################     
                    present_bead_count += 1         
            print("END")
            sys.stdout.close()

################################## Graph Viewer ####################################
GRAPH_COORDINATE = []
XZ_GRAPH_COORDINATE = []
COMP_XZ_GRAPH_COORDINATE = []
comp_x0 = 0
comp_z0 = -R
for i in range(1, TOTAL_LAYER+1):
    xz_graph_coordinate = []
    comp_xz_graph_coordinate = []
    comp_ta = Start_E1 + (i-1)*Delta_E1
    comp_x = round(comp_x0*np.cos(comp_ta*to_rad) - comp_z0*np.sin(comp_ta*to_rad),3)
    comp_z = round(-(comp_z0 - (comp_x0*np.sin(comp_ta*to_rad) + comp_z0*np.cos(comp_ta*to_rad))) + JIG_DIS,3)
    comp_xz_graph_coordinate.append(comp_x)
    comp_xz_graph_coordinate.append(comp_z)
    COMP_XZ_GRAPH_COORDINATE.append(comp_xz_graph_coordinate) 
    for j in range(len(BEAD_RADIUS[i-1])):
        graph_coordinate_start = [float(BEAD_START_COORDINATE_X[i-1][j]), float(BEAD_START_COORDINATE_Y[i-1][j]), float(BEAD_INITIAL_COORDINATE_Z[i-1][j])]
        GRAPH_COORDINATE.append(graph_coordinate_start)
        if SIMULATION_TYPE == 1:
            for k in range(POINT_GROUP[i-1][j]):
                graph_coor_among = [BEAD_AMONG_COORDINATE_X[i-1][j][k],BEAD_AMONG_COORDINATE_Y[i-1][j][k], BEAD_INITIAL_COORDINATE_Z[i-1][j]]
                GRAPH_COORDINATE.append(graph_coor_among)
        graph_coordinate_end = [float(BEAD_END_COORDINATE_X[i-1][j]), float(BEAD_END_COORDINATE_Y[i-1][j]), float(BEAD_INITIAL_COORDINATE_Z[i-1][j])]
        GRAPH_COORDINATE.append(graph_coordinate_end)
    xz_graph_coordinate.append(float(BEAD_INITIAL_COORDINATE_X[i-1][0]))
    xz_graph_coordinate.append(float(BEAD_INITIAL_COORDINATE_Z[i-1][0]))
    XZ_GRAPH_COORDINATE.append(xz_graph_coordinate)

xz_x_coords_1 = [point[0] for point in COMP_XZ_GRAPH_COORDINATE] 
xz_x_coords_2 = [point[0] for point in XZ_GRAPH_COORDINATE] 
xz_z_coords_1 = [point[1] for point in COMP_XZ_GRAPH_COORDINATE] 
xz_z_coords_2 = [point[1] for point in XZ_GRAPH_COORDINATE]  

x_coords = [coord[0] for coord in GRAPH_COORDINATE]
y_coords = [coord[1] for coord in GRAPH_COORDINATE]
z_coords = [coord[2] for coord in GRAPH_COORDINATE]

# ######## XZ Graph Comparison ##########
# plt.figure(figsize=(8, 6))
# plt.scatter(xz_x_coords_1, xz_z_coords_1, color='blue', marker='o', s=1, label='Original Slicing')  
# # plt.plot(xz_x_coords_1, xz_z_coords_1, color='blue', linestyle='-', label='Connecting Line')
# plt.scatter(xz_x_coords_2, xz_z_coords_2, color='red', marker='o', s=1, label='Adaptive Slicing')  
# # plt.plot(xz_x_coords_2, xz_z_coords_2, color='red', linestyle='-', label='Connecting Line')
# plt.xlabel('X Coordinate')
# plt.ylabel('Z Coordinate')
# plt.legend()
# plt.grid(True)
# plt.show()

# ########## XYZ Graph Import #########
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_coords, y_coords, z_coords, color= 'blue')
# for i in range(len(GRAPH_COORDINATE)):
#     if i % 2 == 0:
#         ax.scatter(GRAPH_COORDINATE[i][0], GRAPH_COORDINATE[i][1], GRAPH_COORDINATE[i][2], color='blue')
#     else:
#         ax.scatter(GRAPH_COORDINATE[i][0], GRAPH_COORDINATE[i][1], GRAPH_COORDINATE[i][2], color='red')
for i in range(len(GRAPH_COORDINATE)-1):
    ax.plot([GRAPH_COORDINATE[i][0], GRAPH_COORDINATE[i+1][0]], [GRAPH_COORDINATE[i][1], GRAPH_COORDINATE[i+1][1]], [GRAPH_COORDINATE[i][2], GRAPH_COORDINATE[i+1][2]], color='red')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

######################### Simulation #############################
if SIMULATION_TYPE == 1:
    animation_speed = 1.0
    frame_skip = 1
    def create_cube(center, width=BEAD_WIDTH, height=LAYER_THICKNESS):
        if len(center) != 3:
            raise ValueError(f"Expected 3 values for center, got {len(center)}: {center}")
        x, y, z = center
        r = [-width / 2, width / 2]
        h = [0, height]
        vertices = np.array([[x + i, y + j, z + k] for i in r for j in r for k in h])
        faces = [[vertices[j] for j in [0, 1, 3, 2]], # bottom face
                [vertices[j] for j in [4, 5, 7, 6]],  # top face
                [vertices[j] for j in [0, 1, 5, 4]],  # front face
                [vertices[j] for j in [2, 3, 7, 6]],  # back face
                [vertices[j] for j in [0, 2, 6, 4]],  # left face
                [vertices[j] for j in [1, 3, 7, 5]]]  # right face
        return faces

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    def draw_block(center):
        x, y, z = float(center[0]), float(center[1]), float(center[2])
        faces = create_cube([x, y, z], BEAD_WIDTH, LAYER_THICKNESS)
        cube = Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.6)
        ax.add_collection3d(cube)

    def update(val):
        ax.cla()  
        ax.set_xlim(min([float(coord[0]) for coord in GRAPH_COORDINATE]) - 10, max([float(coord[0]) for coord in GRAPH_COORDINATE]) + 10)
        ax.set_ylim(min([float(coord[1]) for coord in GRAPH_COORDINATE]) - 10, max([float(coord[1]) for coord in GRAPH_COORDINATE]) + 10)
        ax.set_zlim(min([float(coord[2]) for coord in GRAPH_COORDINATE]) - 10, max([float(coord[2]) for coord in GRAPH_COORDINATE]) + 10)
        ax.set_box_aspect([1, 1, 2])

        frame = int(slider.val)
        for i in range(frame):
            center = [float(coord) for coord in GRAPH_COORDINATE[i]]
            draw_block(center)
        fig.canvas.draw_idle()

    ax_slider = plt.axes([0.25, 0.02, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_slider, 'Time', 1, len(GRAPH_COORDINATE), valinit=1, valstep=1)
    slider.on_changed(update)

    is_playing = False

    def set_frame_skip(skip_value):
        global frame_skip
        frame_skip = skip_value

    def play(event):
        global is_playing
        is_playing = True
        while is_playing and slider.val < len(GRAPH_COORDINATE):
            slider.set_val(slider.val + frame_skip)  
            time.sleep(0.5 / animation_speed)  
            fig.canvas.flush_events()

    def stop(event):
        global is_playing
        is_playing = False

    def set_speed(speed):
        global animation_speed
        animation_speed = speed

    ax_play = plt.axes([0.7, 0.9, 0.1, 0.04])
    btn_play = Button(ax_play, 'Play')
    btn_play.on_clicked(play)

    ax_stop = plt.axes([0.81, 0.9, 0.1, 0.04])
    btn_stop = Button(ax_stop, 'Stop')
    btn_stop.on_clicked(stop)

    ax_speed1 = plt.axes([0.1, 0.9, 0.08, 0.04])
    btn_speed1 = Button(ax_speed1, '1x')
    btn_speed1.on_clicked(lambda event: set_speed(1))

    ax_speed2 = plt.axes([0.2, 0.9, 0.08, 0.04])
    btn_speed2 = Button(ax_speed2, '3x')
    btn_speed2.on_clicked(lambda event: set_speed(3))

    ax_speed3 = plt.axes([0.3, 0.9, 0.08, 0.04])
    btn_speed3 = Button(ax_speed3, '5x')
    btn_speed3.on_clicked(lambda event: set_speed(5))

    ax_speed4 = plt.axes([0.4, 0.9, 0.08, 0.04])
    btn_speed4 = Button(ax_speed4, '7x')
    btn_speed4.on_clicked(lambda event: set_speed(7))

    ax_speed5 = plt.axes([0.5, 0.9, 0.08, 0.04])
    btn_speed5 = Button(ax_speed5, '9x')
    btn_speed5.on_clicked(lambda event: set_speed(9))

    ax_skip1 = plt.axes([0.1, 0.85, 0.08, 0.04])
    btn_skip1 = Button(ax_skip1, '1*Frame')
    btn_skip1.on_clicked(lambda event: set_frame_skip(1)) 

    ax_skip2 = plt.axes([0.2, 0.85, 0.08, 0.04])
    btn_skip2 = Button(ax_skip2, '5*Frame')
    btn_skip2.on_clicked(lambda event: set_frame_skip(50)) 

    ax_skip3 = plt.axes([0.3, 0.85, 0.08, 0.04])
    btn_skip3 = Button(ax_skip3, '10*Frame')
    btn_skip3.on_clicked(lambda event: set_frame_skip(100)) 

    ax_skip4 = plt.axes([0.4, 0.85, 0.08, 0.04])
    btn_skip4 = Button(ax_skip4, '15*Frame')
    btn_skip4.on_clicked(lambda event: set_frame_skip(150)) 

    ax_skip5 = plt.axes([0.5, 0.85, 0.08, 0.04])
    btn_skip5 = Button(ax_skip5, '20*Frame')
    btn_skip5.on_clicked(lambda event: set_frame_skip(200))
    update(0.01)
    plt.show()