
########################################### utility


import numpy as np
from enum import Enum
from gcodeparser import GcodeParser

from pyrobopath.toolpath import Toolpath, Contour
from pyrobopath.toolpath_scheduling import MultiAgentToolpathSchedule


# ========================== helper functions ==========================
def toolpath_from_gcode(filepath):
    """Parse gcode file to internal toolpath representation."""
    with open(filepath, "r") as f:
        gcode = f.read()
    parsed_gcode = GcodeParser(gcode)

    toolpath = Toolpath.from_gcode(parsed_gcode.lines)
    return toolpath


def print_schedule_info(schedule: MultiAgentToolpathSchedule):
    print(f"Schedule duration: {schedule.duration()}")
    print(f"Total Events: {schedule.n_events()}")

    for agent, sched in schedule.schedules.items():
        duration = 0.0
        for e in sched._events:
            duration += e.duration
        print(f"{agent} execution time: {duration}")
    print()


# ======================== simple toolpath ==========================
class Materials(Enum):
    MATERIAL_A = 1
    MATERIAL_B = 2
    MATERIAL_C = 3
    MATERIAL_D = 4


def raster_rect(p, h, spacing, n):
    pi = np.array(p)
    raster = [pi.copy()]
    dir = 1.0
    for _ in range(n):
        pi[1] = pi[1] + (h * dir)
        raster.append(pi.copy())
        pi[0] = pi[0] + spacing
        dir *= -1
        raster.append(pi.copy())
    pi[1] = pi[1] + (h * dir)
    raster.append(pi.copy())
    return raster


def rotate_pathZ(path, about, rad):
    about = np.array(about)
    new_path = [p - about for p in path]
    s, c = np.sin(rad), np.cos(rad)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    new_path = [(R @ p) + about for p in new_path]
    return new_path


def create_example_toolpath():
    # Layer 1
    path1 = raster_rect([-150.0, -150.0, 0.0], 300, 20, 7)
    path2 = raster_rect([10.0, -150.0, 0.0], 300, 20, 7)

    # Layer 2
    path3 = raster_rect([-150.0, -150.0, 1.0], 140, 20, 7)
    path4 = raster_rect([-150.0, 150.0, 1.0], 300, 20, 7)
    path4 = rotate_pathZ(path4, [-150, 150, 0], -np.pi / 2)
    path5 = raster_rect([10.0, -150.0, 1.0], 140, 20, 7)

    # Layer 3
    path6 = raster_rect([-150.0, 150.0, 2.0], -200, 20, 5)
    path7 = raster_rect([-30.0, 150.0, 2.0], -200, 20, 5)
    path8 = raster_rect([-230.0, -150.0, 2.0], 220, 20, 4)
    path8 = rotate_pathZ(path8, [-150.0, -150.0, 2.0], -np.pi / 2)
    path9 = raster_rect([90.0, -150.0, 2.0], 300, 20, 3)

    c1 = Contour(path1, tool=Materials.MATERIAL_A)
    c2 = Contour(path2, tool=Materials.MATERIAL_B)
    c3 = Contour(path3, tool=Materials.MATERIAL_B)
    c4 = Contour(path4, tool=Materials.MATERIAL_A)
    c5 = Contour(path5, tool=Materials.MATERIAL_A)
    c6 = Contour(path6, tool=Materials.MATERIAL_B)
    c7 = Contour(path7, tool=Materials.MATERIAL_A)
    c8 = Contour(path8, tool=Materials.MATERIAL_A)
    c9 = Contour(path9, tool=Materials.MATERIAL_B)

    return Toolpath([c1, c2, c3, c4, c5, c6, c7, c8, c9])






#########################################################3 toolpath example

from pyrobopath.toolpath import visualize_toolpath, visualize_toolpath_projection
from pyrobopath.toolpath.path.transform import Rotation
from pyrobopath.toolpath.preprocessing import *

from utilities import Materials, toolpath_from_gcode


# =========================== visualization ============================
def toolpath_visualization_example():
    filepath = "../test/test_gcode/multi_tool_square.gcode"
    toolpath = toolpath_from_gcode(filepath)
    visualize_toolpath(toolpath, backend="matplotlib", color_method="tool")
    visualize_toolpath(toolpath, backend="pyqtgraph", color_method="cycle")


def toolpath_layer_visualization():
    filepath = "../test/test_gcode/multi_tool_square.gcode"
    toolpath = toolpath_from_gcode(filepath)
    visualize_toolpath_projection(toolpath)


# =========================== preprocessing ============================
def toolpath_preprocessing_example():
    filepath = "../test/test_gcode/multi_tool_square.gcode"
    toolpath = toolpath_from_gcode(filepath)

    # visualize_toolpath before modifications
    visualize_toolpath(toolpath, backend="matplotlib", color_method="cycle", show=False)

    # preprocessing (see other preprocessing steps in pyrobopath documentation)
    preprocessor = ToolpathPreprocessor()
    preprocessor.add_step(TranslateStep([100, 0, 0]))
    preprocessor.add_step(RotateStep(Rotation.Rz(np.pi / 4)))
    preprocessor.add_step(ScalingStep(0.001))
    preprocessor.add_step(MaxContourLengthStep(1.0))
    preprocessor.add_step(LayerRangeStep(5, 10))
    preprocessor.add_step(
        SubstituteToolStep({0: Materials.MATERIAL_A, 1: Materials.MATERIAL_B})
    )

    preprocessor.process(toolpath)

    visualize_toolpath(toolpath, color_method="cycle", backend="matplotlib")


if __name__ == "__main__":
    toolpath_visualization_example()
    toolpath_layer_visualization()
    toolpath_preprocessing_example()



################################### simple scheduling example #################
import numpy as np

from pyrobopath.process import AgentModel, create_dependency_graph_by_z
from pyrobopath.collision_detection import FCLRobotBBCollisionModel
from pyrobopath.toolpath_scheduling import (
    MultiAgentToolpathPlanner,
    PlanningOptions,
    animate_multi_agent_toolpath_full,
)

from utilities import Materials, print_schedule_info, create_example_toolpath


def two_robot_agent_models():
    bf1 = np.array([-350.0, 0.0, 0.0])
    bf2 = np.array([350.0, 0.0, 0.0])

    # create agent collision models
    agent1 = AgentModel(
        base_frame_position=bf1,
        home_position=np.array([-250.0, 0.0, 0.0]),
        capabilities=[Materials.MATERIAL_A],
        velocity=50.0,
        travel_velocity=50.0,
        collision_model=FCLRobotBBCollisionModel((200.0, 50.0, 300.0), bf1),
    )
    agent2 = AgentModel(
        base_frame_position=bf2,
        home_position=np.array([250.0, 0.0, 0.0]),
        capabilities=[Materials.MATERIAL_B],
        velocity=50.0,
        travel_velocity=50.0,
        collision_model=FCLRobotBBCollisionModel((200.0, 50.0, 300.0), bf2),
    )
    agent_models = {"robot1": agent1, "robot2": agent2}
    return agent_models


def two_robot_simple_example():
    toolpath = create_example_toolpath()
    dg = create_dependency_graph_by_z(toolpath)

    # create planner
    agent_models = two_robot_agent_models()
    planner = MultiAgentToolpathPlanner(agent_models)
    options = PlanningOptions(
        retract_height=5.0,
        collision_offset=1.0,
        collision_gap_threshold=1.0,
    )

    print(f"{(80 * '#')}\nScheduling Simple Two Materal Toolpath:\n{(80 * '#')}\n")
    sched = planner.plan(toolpath, dg, options)
    print(f"\n{(80 * '#')}\nFound Toolpath Plan!\n{(80 * '#')}\n")
    print_schedule_info(sched)

    animate_multi_agent_toolpath_full(
        toolpath, sched, agent_models, limits=((-550, 550), (-250, 250))
    )


if __name__ == "__main__":
    two_robot_simple_example()