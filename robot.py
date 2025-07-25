import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from gcodeparser import GcodeParser
from pyrobopath.toolpath import Contour, Toolpath, visualize_toolpath, visualize_toolpath_projection
from pyrobopath.toolpath.preprocessing import LayerRangeStep
from pyrobopath.process import AgentModel, DependencyGraph, create_dependency_graph_by_z
from pyrobopath.collision_detection import FCLRobotBBCollisionModel
from pyrobopath.toolpath_scheduling import ContourEvent, MoveEvent, MultiAgentToolpathSchedule, PlanningOptions, events_cause_collision, animate_multi_agent_toolpath_full
from pyrobopath.toolpath.preprocessing import *


MATERIAL = 0

GCODE_MODE = False
# True: gcode toolpath   False: manual toolpath
PREPROCESSING_MODE = False
# True: preprocessing    False: skip preprocessing
SCHEDULE_MODE = True
# True: scheduling       False: visualizing toolpath

ALGORITHM_MODE = 1
# 0: sequential
# 1: distance priority

GCODE_PATH  = "./gcode/multi_tool_square.gcode"
MANUAL_PATH = "./manual/cylinder.txt"

ROBOT_COUNT        = 3      # int
ROBOT_BASEFRAME_R  = 350.0  # float
ROBOT_HOMEPOS_R    = 250.0  # float

MAX_CONTOUR_LENGTH = 150.0  # float


class SchedulingContext:
    def __init__(self, agent_models: Dict[str, AgentModel], options: PlanningOptions):
        self.agent_models = agent_models
        self.options = options
        self.reset()

    def reset(self):
        self.start_times = dict.fromkeys(self.agent_models.keys(), 0.0)
        self.positions = dict()
        for agent in self.agent_models:
            self.positions[agent] = self.agent_models[agent].home_position

    def get_agents_with_start_time(self, time):
        min_time_agents = [a for (a, t) in self.start_times.items() if t == time]
        return min_time_agents

    def get_unique_start_times(self):
        return sorted(set(self.start_times.values()))

    def set_agent_start_time(self, agent, time):
        self.start_times[agent] = time

    def get_current_position(self, agent):
        return self.positions[agent]
    

class TaskManager:
    def __init__(self, toolpath: Toolpath, dg: DependencyGraph):
        self.contours = toolpath.contours
        self.dg = dg

        self.frontier = set()
        self.completed_tasks = set()
        self.in_progress: Dict[str, float] = dict()

    def add_inprogress(self, id, t_end):
        self.in_progress[id] = t_end

    def mark_inprogress_complete(self, time):
        complete = [k for (k, v) in self.in_progress.items() if time >= v]
        self.completed_tasks.update(complete)
        for c in complete:
            self.dg.mark_complete(c)
            self.in_progress.pop(c)

    def has_frontier(self):
        return bool(self.frontier)

    def get_available_tasks(self, *args):
        available = [n for n in self.frontier if self.dg.can_start(n)]
        return available


def build_event_chain(
    t_start, p_start, contour: Contour, agent, context: SchedulingContext
):
    p_approach = contour.path[0].copy()
    p_approach[2] += context.options.retract_height
    path_travel = [p_start, p_approach, contour.path[0]]

    if (p_start == p_approach).all():
        path_travel.pop(0)

    # travel event
    e_travel = MoveEvent(
        t_start, path_travel, context.agent_models[agent].travel_velocity
    )

    # contour event
    e_contour = ContourEvent(
        e_travel.end, contour, context.agent_models[agent].velocity
    )

    p_depart = contour.path[-1].copy()
    p_depart[2] += context.options.retract_height

    # depart events
    e_depart = MoveEvent(
        e_contour.end,
        [contour.path[-1], p_depart],
        context.agent_models[agent].travel_velocity,
    )

    # depart events
    e_home = MoveEvent(
        e_depart.end,
        [p_depart, context.agent_models[agent].home_position],
        context.agent_models[agent].travel_velocity,
    )

    return [e_travel, e_contour, e_depart, e_home]


def slice_home_event(home_event: MoveEvent, end_time: float):
    new_traj = home_event.traj.slice(home_event.start, end_time)
    path = [p.data for p in new_traj.points]
    return MoveEvent(home_event.start, path, home_event.velocity)


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


schedule = None

if SCHEDULE_MODE:
    graph = create_dependency_graph_by_z(toolpath)
    options = PlanningOptions(
        retract_height=0.0,
        collision_offset=1.0,
        collision_gap_threshold=1.0,
    )

    print()
    print("========== Start Scheduling ==========")

    required_tools = set(toolpath.tools())
    provided_tools = set(
        [cap for a in agent_models.values() for cap in a.capabilities]
    )
    if not required_tools.issubset(provided_tools):
        raise ValueError("Agents cannot provide all required capabilities")

    schedule = MultiAgentToolpathSchedule()
    schedule.add_agents(agent_models.keys())

    context = SchedulingContext(agent_models, options)
    taskmanager = TaskManager(toolpath, graph)
    taskmanager.frontier.update(graph.roots())

    while taskmanager.has_frontier():
        sorted_times = context.get_unique_start_times()
        time = sorted_times[0]
        min_time_agents = context.get_agents_with_start_time(time)
        taskmanager.mark_inprogress_complete(time)

        for agent in min_time_agents:
            tools = agent_models[agent].capabilities
            available = taskmanager.get_available_tasks()
            available = [c for c in available if taskmanager.contours[c].tool in tools]

            if not available:
                if len(sorted_times) > 1:
                    context.set_agent_start_time(agent, sorted_times[1])
                continue

            ##################################################
            ############## SEQUENTIAL ALGORITHM ##############

            if ALGORITHM_MODE == 0:
                key = lambda node: graph._graph.out_degree(node)
                nodes = sorted(available, key=key, reverse=True)

            ##################################################
            ########## DISTANCE PRIORITY ALGORITHM ###########

            if ALGORITHM_MODE == 1:
                agent_position = context.get_current_position(agent)

                key = lambda node: np.linalg.norm(agent_position - taskmanager.contours[node].path[0]) + \
                                   np.linalg.norm(agent_position - taskmanager.contours[node].path[-1])
                nodes = sorted(available, key=key)

                for node in nodes:
                    start_point_dist = np.linalg.norm(agent_position - taskmanager.contours[node].path[0])
                    finish_point_dist = np.linalg.norm(agent_position - taskmanager.contours[node].path[-1])
                    
                    if start_point_dist > finish_point_dist:
                        taskmanager.contours[node].path = taskmanager.contours[node].path[::-1]

            ##################################################

            all_collide_flag = True
            for node in nodes:
                contour = taskmanager.contours[node]
                p_start = schedule[agent].get_state(
                    time, agent_models[agent].home_position
                )

                events = build_event_chain(time, p_start, contour, agent, context)
                if events_cause_collision(
                    events,
                    agent,
                    schedule,
                    agent_models,
                    options.collision_gap_threshold,
                ):
                    continue

                # slice home event if overlap
                if schedule[agent].end_time() > events[0].start:
                    prev_home_event = schedule[agent]._events.pop()
                    if prev_home_event.start != events[0].start:
                        sliced_home = slice_home_event(prev_home_event, events[0].start)
                        schedule.add_event(sliced_home, agent)

                schedule.add_events(events, agent)
                taskmanager.add_inprogress(node, events[1].end)
                taskmanager.frontier.remove(node)
                taskmanager.frontier.update(graph._graph.successors(node))
                context.positions[agent] = events[2].data[-1]
                context.set_agent_start_time(agent, events[2].end)

                all_collide_flag = False
                break

            if all_collide_flag:
                context.set_agent_start_time(agent, time + options.collision_offset)

    print("|")
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

    x, y, z = [], [], []
    for time in range(int(schedule.start_time()), int(schedule.end_time()) + 1):
        for robot, homepos in zip(["robot1", "robot2", "robot3"], [homepos1, homepos2, homepos3]):
            pos = schedule[robot].get_state(time, default=homepos)
            x.append(pos[0])
            y.append(pos[1])
            z.append(pos[2])

    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.2)

    sc = ax.scatter([x[:3]], [y[:3]], [z[:3]], c='r', s=30)

    ax.set_title("WAAM Simulation")
    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_zlim(-1, 10)

    ax_slider = plt.axes([0.2, 0.1, 0.6, 0.03])  # [left, bottom, width, height]
    time_slider = Slider(ax_slider,
                         'Time',
                         int(schedule.start_time()),
                         int(schedule.end_time()),
                         valinit=int(schedule.start_time()),
                         valstep=1)

    def update(val):
        time = time_slider.val
        sc._offsets3d = (x[:3*(time+1)], y[:3*(time+1)], z[:3*(time+1)])
        fig.canvas.draw_idle()

    time_slider.on_changed(update)
    plt.show()

else:
    visualize_toolpath_projection(toolpath)
    visualize_toolpath(toolpath, backend="matplotlib", color_method="tool")

