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
PREPROCESSING_MODE = False
# True: preprocessing    False: skip preprocessing

GCODE_PATH  = "./gcode/square.gcode"
MANUAL_PATH = "./manual/test_1.txt"

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
    baseframe1 = np.array([-350.0, 0.0, 0.0])
    homepos1   = np.array([-250.0, 0.0, 0.0])

    baseframe2 = np.array([350.0, 0.0, 0.0])
    homepos2   = np.array([250.0, 0.0, 0.0])

    boundingbox = (200.0, 50.0, 300.0)

    agent1 = AgentModel(
        base_frame_position = baseframe1,
        home_position       = homepos1,
        capabilities        = [0,1],
        velocity            = 50.0,
        travel_velocity     = 50.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe1),
    )
    agent2 = AgentModel(
        base_frame_position = baseframe2,
        home_position       = homepos2,
        capabilities        = [0,1],
        velocity            = 50.0,
        travel_velocity     = 50.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe2),
    )
    agent_models = {"robot1": agent1, "robot2": agent2}


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
        toolpath, schedule, agent_models, limits=((-550, 550), (-250, 250))
    )

else:
    visualize_toolpath_projection(toolpath)
    visualize_toolpath(toolpath, backend="matplotlib", color_method="tool")

