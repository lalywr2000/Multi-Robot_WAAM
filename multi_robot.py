import numpy as np
from gcodeparser import GcodeParser
from pyrobopath.toolpath import Contour, Toolpath, visualize_toolpath, visualize_toolpath_projection
from pyrobopath.process import AgentModel, create_dependency_graph_by_z
from pyrobopath.collision_detection import FCLRobotBBCollisionModel
from pyrobopath.toolpath_scheduling import MultiAgentToolpathPlanner,PlanningOptions, animate_multi_agent_toolpath_full


CHUNK_1 = 1
CHUNK_2 = 2

GCODE_MODE    = False  # True: gcode toolpath   False: manual toolpath
SCHEDULE_MODE = True   # True: scheduling       False: visualize toolpath

GCODE_PATH = "./gcode/???.gcode"


#-------------------- Toolpath --------------------


toolpath = None

if GCODE_MODE:
    with open(GCODE_PATH, "r") as f:
        gcode = f.read()
    parsed_gcode = GcodeParser(gcode)
    toolpath = Toolpath.from_gcode(parsed_gcode.lines)

else:
    points1 = [[-150.0, -150.0, 0.0],
               [-150.0, 150.0, 0.0],
               [150.0, 150.0, 0.0],
               [150.0, -150.0, 0.0],]

    points2 = [[-100.0, -100.0, 0.0],
               [-100.0, 100.0, 0.0],
               [100.0, 100.0, 0.0],
               [100.0, -100.0, 0.0],]

    path1 = [np.array(point) for point in points1]
    path2 = [np.array(point) for point in points2]
    contour1 = Contour(path1, tool=CHUNK_1)
    contour2 = Contour(path2, tool=CHUNK_2)
    toolpath = Toolpath([contour1, contour2])


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
        capabilities        = [CHUNK_1],
        velocity            = 50.0,
        travel_velocity     = 50.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe1),
    )
    agent2 = AgentModel(
        base_frame_position = baseframe2,
        home_position       = homepos2,
        capabilities        = [CHUNK_2],
        velocity            = 50.0,
        travel_velocity     = 50.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe2),
    )
    agent_models = {"robot1": agent1, "robot2": agent2}


#------------------- Scheduling -------------------


if SCHEDULE_MODE:
    graph = create_dependency_graph_by_z(toolpath)

    planner = MultiAgentToolpathPlanner(agent_models)
    options = PlanningOptions(
        retract_height=5.0,
        collision_offset=1.0,
        collision_gap_threshold=1.0,
    )

    print("========== Start Scheduling ==========")

    schedule = planner.plan(toolpath, graph, options)

    print(f"| - Total duration: {schedule.duration()}")
    for agent, sched in schedule.schedules.items():
        duration = 0.0
        for event in sched._events:
            duration += event.duration
        print(f"| - {agent} execution time: {duration}")

    print("========== Finish Scheduling =========")


#------------------- Visualize --------------------


if SCHEDULE_MODE:
    animate_multi_agent_toolpath_full(
        toolpath, schedule, agent_models, limits=((-550, 550), (-250, 250))
    )

else:
    visualize_toolpath(toolpath, backend="matplotlib", color_method="tool")
    visualize_toolpath_projection(toolpath)
