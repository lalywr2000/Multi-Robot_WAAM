import numpy as np
from gcodeparser import GcodeParser
from pyrobopath.toolpath import Contour, Toolpath, visualize_toolpath, visualize_toolpath_projection
from pyrobopath.toolpath.preprocessing import LayerRangeStep
from pyrobopath.process import AgentModel, create_dependency_graph_by_z
from pyrobopath.collision_detection import FCLRobotBBCollisionModel
from pyrobopath.toolpath_scheduling import MultiAgentToolpathPlanner,PlanningOptions, animate_multi_agent_toolpath_full
from pyrobopath.toolpath.preprocessing import *


MATERIAL = 1

GCODE_MODE = False
# True: gcode toolpath   False: manual toolpath
SCHEDULE_MODE = True
# True: scheduling       False: visualizing toolpath
PREPROCESSING_MODE = True
# True: preprocessing    False: skip preprocessing

GCODE_PATH  = "./gcode/square.gcode"
MANUAL_PATH = "./manual/test_1.txt"

ROBOT_COUNT        = 3      # int
ROBOT_BASEFRAME_R  = 350.0  # float
ROBOT_HOMEPOS_R    = 250.0  # float

MAX_CONTOUR_LENGTH = 300.0  # float


#-------------------- Toolpath --------------------


toolpath = None

if GCODE_MODE:
    with open(GCODE_PATH, "r") as f:
        gcode = f.read()
    parsed_gcode = GcodeParser(gcode)
    toolpath = Toolpath.from_gcode(parsed_gcode.lines)
    LayerRangeStep(0, 1).apply(toolpath)  # Extract the first layer

else:
    path = []
    contour = []
    with open(MANUAL_PATH, "r") as f:
        for line in f:
            if not line.isspace():
                point = np.array(list(map(float, line.split())))
                path.append(point)
            else:
                contour.append(Contour(path, tool=MATERIAL))
                path = []
    toolpath = Toolpath(contour)


#------------------ Agent Models ------------------


agent_models = None

if SCHEDULE_MODE:
    baseframe1 = np.array([ROBOT_BASEFRAME_R * np.cos(2 * np.pi * 1 / ROBOT_COUNT + np.pi / 6),  # robot1 x_pos
                           ROBOT_BASEFRAME_R * np.sin(2 * np.pi * 1 / ROBOT_COUNT + np.pi / 6),  # robot1 y_pos
                           0.0])                                                                 # robot1 z_pos
    homepos1   = np.array([ROBOT_HOMEPOS_R * np.cos(2 * np.pi * 1 / ROBOT_COUNT + np.pi / 6),
                           ROBOT_HOMEPOS_R * np.sin(2 * np.pi * 1 / ROBOT_COUNT + np.pi / 6),
                           0.0])

    baseframe2 = np.array([ROBOT_BASEFRAME_R * np.cos(2 * np.pi * 3 / ROBOT_COUNT + np.pi / 6),  # robot2 x_pos
                           ROBOT_BASEFRAME_R * np.sin(2 * np.pi * 3 / ROBOT_COUNT + np.pi / 6),  # robot2 y_pos
                           0.0])                                                                 # robot2 z_pos
    homepos2   = np.array([ROBOT_HOMEPOS_R * np.cos(2 * np.pi * 3 / ROBOT_COUNT + np.pi / 6),
                           ROBOT_HOMEPOS_R * np.sin(2 * np.pi * 3 / ROBOT_COUNT + np.pi / 6),
                           0.0])
    
    baseframe3 = np.array([ROBOT_BASEFRAME_R * np.cos(2 * np.pi * 2 / ROBOT_COUNT + np.pi / 6),  # robot3 x_pos
                           ROBOT_BASEFRAME_R * np.sin(2 * np.pi * 2 / ROBOT_COUNT + np.pi / 6),  # robot3 y_pos
                           0.0])                                                                 # robot3 z_pos
    homepos3   = np.array([ROBOT_HOMEPOS_R * np.cos(2 * np.pi * 2 / ROBOT_COUNT + np.pi / 6),
                           ROBOT_HOMEPOS_R * np.sin(2 * np.pi * 2 / ROBOT_COUNT + np.pi / 6),
                           0.0])

    boundingbox = (150.0, 40.0, 100.0)

    agent1 = AgentModel(
        base_frame_position = baseframe1,
        home_position       = homepos1,
        capabilities        = [0,1],  # robot1 material
        velocity            = 10.0,
        travel_velocity     = 50.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe1),
    )
    agent2 = AgentModel(
        base_frame_position = baseframe2,
        home_position       = homepos2,
        capabilities        = [0,1],  # robot2 material
        velocity            = 10.0,
        travel_velocity     = 50.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe2),
    )
    agent3 = AgentModel(
        base_frame_position = baseframe3,
        home_position       = homepos3,
        capabilities        = [0,1],  # robot3 material
        velocity            = 10.0,
        travel_velocity     = 50.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe3),
    )
    agent_models = {"robot1": agent1, "robot2": agent2, "robot3": agent3}


#------------------ Preprocessing -----------------


if PREPROCESSING_MODE:
    preprocessor = ToolpathPreprocessor()
    preprocessor.add_step(MaxContourLengthStep(MAX_CONTOUR_LENGTH))
    preprocessor.process(toolpath)


#------------------- Scheduling -------------------


if SCHEDULE_MODE:
    graph = create_dependency_graph_by_z(toolpath)

    planner = MultiAgentToolpathPlanner(agent_models)
    options = PlanningOptions(
        retract_height=5.0,
        collision_offset=1.0,
        collision_gap_threshold=1.0,
    )

    print()
    print("========== Start Scheduling ==========")
    print("|")

    schedule = planner.plan(toolpath, graph, options)

    print(f"| - Total duration: {schedule.duration()}")
    for agent, sched in schedule.schedules.items():
        duration = 0.0
        for event in sched._events:
            duration += event.duration
        print(f"| - {agent} execution time: {duration}")

    print("|")
    print("========== Finish Scheduling =========")
    print()


#------------------- Visualizing ------------------


if SCHEDULE_MODE:
    animate_multi_agent_toolpath_full(
        toolpath, schedule, agent_models, limits=((-550, 550), (-430, 250))
    )

else:
    visualize_toolpath_projection(toolpath)
    visualize_toolpath(toolpath, backend="matplotlib", color_method="tool")

