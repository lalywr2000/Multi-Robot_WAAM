import numpy as np

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from gcodeparser import GcodeParser

from pyrobopath.toolpath_scheduling import ContourEvent, MoveEvent, MultiAgentToolpathSchedule, PlanningOptions, events_cause_collision, animate_multi_agent_toolpath_full
from pyrobopath.toolpath import Contour, Toolpath, visualize_toolpath
from pyrobopath.toolpath.preprocessing import *
from pyrobopath.process import AgentModel, DependencyGraph, create_dependency_graph_by_z
from pyrobopath.collision_detection import FCLRobotBBCollisionModel


GCODE_MODE = False
# True: input gcode path   # False: input manual path
PREPROCESSING_MODE = True
# True: preprocessing      # False: skip preprocessing
SCHEDULING_MODE = True
# True: scheduling         # False: visualizing

ALGORITHM_MODE = 1
# 0: sequential
# 1: distance priority

GCODE_PATH  = "./path/gcode/square.gcode"
MANUAL_PATH = "./path/manual/puzzle.txt"

ROBOT_REACHABLE_R  = 2500.0  # [mm]
SUBSTRATE_SIZE     = 500.0   # [mm]
MAX_CONTOUR_LENGTH = 300.0   # [mm]
LAYER_THICKNESS    = 3.0     # [mm]
DWELLING_TIME      = 0.0


#------ Base Implementation from Pyrobopath -------
#--- https://github.com/alexarbogast/pyrobopath ---


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
    LayerRangeStep(0, 1).apply(toolpath)  # extracting first layer

else:
    path = []
    contour = []
    with open(MANUAL_PATH, "r") as f:
        for line in f:
            if not line.isspace():
                point = np.array(list(map(float, line.split())))
                path.append(point)
            else:
                contour.append(Contour(path))
                path = []
    toolpath = Toolpath(contour)


#------------------ Agent Models ------------------


agent_models = None

if SCHEDULING_MODE:
    baseframe1 = np.array([-1250, 1150, 0])  # robot1 pos
    homepos1   = np.array([-300, 300, 600])

    baseframe2 = np.array([1250, 1150, 0])   # robot2 pos
    homepos2   = np.array([300, 300, 600])
    
    baseframe3 = np.array([0, -1750, 0])     # robot3 pos
    homepos3   = np.array([0, -425, 600])

    boundingbox = (2000.0, 300.0, 500.0)

    agent1 = AgentModel(
        base_frame_position = baseframe1,
        home_position       = homepos1,
        capabilities        = [0,1],  # robot1 material
        velocity            = 1.0,
        travel_velocity     = 3.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe1),
    )
    agent2 = AgentModel(
        base_frame_position = baseframe2,
        home_position       = homepos2,
        capabilities        = [0,1],  # robot2 material
        velocity            = 1.0,
        travel_velocity     = 3.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe2),
    )
    agent3 = AgentModel(
        base_frame_position = baseframe3,
        home_position       = homepos3,
        capabilities        = [0,1],  # robot3 material
        velocity            = 1.0,
        travel_velocity     = 3.0,
        collision_model     = FCLRobotBBCollisionModel(boundingbox, baseframe3),
    )
    agent_models = {"robot1": agent1, "robot2": agent2, "robot3": agent3}


#----------------- Preprocessing ------------------


if PREPROCESSING_MODE:
    preprocessor = ToolpathPreprocessor()
    preprocessor.add_step(MaxContourLengthStep(MAX_CONTOUR_LENGTH))
    preprocessor.process(toolpath)


#------------------- Scheduling -------------------


schedule = None

total_contour_cnt = len(toolpath.contours)
scheduled_contour_cnt = 0

layer_height = 0.0
dwelling_timestamp = []

if SCHEDULING_MODE:
    graph = create_dependency_graph_by_z(toolpath)
    options = PlanningOptions(
        retract_height=0.0,
        collision_offset=10.0,  # scheduling time step
    )

    print()
    print("========== Start Scheduling ==========")
    print("|")

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

            # dwelling event if layer completed
            if available and layer_height != taskmanager.contours[available[0]].path[0][-1]:
                for robot in ["robot1", "robot2", "robot3"]:
                    last_target = schedule[robot]._events[-1].data[-1]
                    home_pos = agent_models[robot].home_position
                    current_pos = schedule[robot].get_state(time)

                    if np.allclose(last_target, home_pos):
                        schedule[robot]._events.pop()

                    dwelling_event = MoveEvent(
                        schedule[robot]._events[-1].end,
                        [current_pos, home_pos],
                        agent_models[robot].travel_velocity,
                    )

                    schedule.add_event(dwelling_event, robot)
                    context.positions[robot] = home_pos
                    context.set_agent_start_time(robot, dwelling_event.end)
                
                for robot in ["robot1", "robot2", "robot3"]:
                    context.set_agent_start_time(robot, max(schedule["robot1"].end_time(),
                                                            schedule["robot2"].end_time(),
                                                            schedule["robot3"].end_time()) + DWELLING_TIME)
                
                dwelling_timestamp.append(max(schedule["robot1"].end_time(),
                                              schedule["robot2"].end_time(),
                                              schedule["robot3"].end_time()) + DWELLING_TIME)

                layer_height += LAYER_THICKNESS
                break

            reachable = []

            if agent == "robot1":
                for contour in available:
                    for point in taskmanager.contours[contour].path:
                        if not np.linalg.norm(baseframe1 - point) < ROBOT_REACHABLE_R:
                            break
                    else:
                        reachable.append(contour)
            
            if agent == "robot2":
                for contour in available:
                    for point in taskmanager.contours[contour].path:
                        if not np.linalg.norm(baseframe2 - point) < ROBOT_REACHABLE_R:
                            break
                    else:
                        reachable.append(contour)

            if agent == "robot3":
                for contour in available:
                    for point in taskmanager.contours[contour].path:
                        if not np.linalg.norm(baseframe3 - point) < ROBOT_REACHABLE_R:
                            break
                    else:
                        reachable.append(contour)

            available = reachable[:]
            ------------------------------------------
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

                scheduled_contour_cnt += 1
                print(f"| Progress: {int(scheduled_contour_cnt / total_contour_cnt * 100)}%")

                all_collide_flag = False
                break

            if all_collide_flag:
                context.set_agent_start_time(agent, time + options.collision_offset)

    print("|")
    print(f"| - Total duration: {schedule.duration()}")
    print("|")
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
        toolpath, schedule, agent_models, limits=((-1500, 1500), (-1850, 1250))
    )

    visualize_toolpath(toolpath, backend="matplotlib", color_method="tool")

    target1_x, target1_y, target1_z = [], [], []
    target2_x, target2_y, target2_z = [], [], []
    target3_x, target3_y, target3_z = [], [], []

    for time in range(int(schedule.start_time()), int(schedule.end_time()) + 1):
        for robot, homepos in zip(["robot1", "robot2", "robot3"], [homepos1, homepos2, homepos3]):
            pos = schedule[robot].get_state(time, default=homepos)

            if robot == "robot1":
                target1_x.append(pos[0])
                target1_y.append(pos[1])
                target1_z.append(pos[2])
            if robot == "robot2":
                target2_x.append(pos[0])
                target2_y.append(pos[1])
                target2_z.append(pos[2])
            if robot == "robot3":
                target3_x.append(pos[0])
                target3_y.append(pos[1])
                target3_z.append(pos[2])

    deposition1 = [False for _ in range(int(schedule.start_time()), int(schedule.end_time()) + 1)]
    deposition2 = [False for _ in range(int(schedule.start_time()), int(schedule.end_time()) + 1)]
    deposition3 = [False for _ in range(int(schedule.start_time()), int(schedule.end_time()) + 1)]

    for robot in ["robot1", "robot2", "robot3"]:
        for event in schedule.schedules[robot]._events:
            if isinstance(event, ContourEvent):
                if robot == "robot1":
                    for time in range(int(event.start)+1, int(event.end)+1):
                        deposition1[time] = True
                if robot == "robot2":
                    for time in range(int(event.start)+1, int(event.end)+1):
                        deposition2[time] = True
                if robot == "robot3":
                    for time in range(int(event.start)+1, int(event.end)+1):
                        deposition3[time] = True



    with open("ROB1_target.txt", "w") as f1:
        idx = 0
        num = 1
        for x, y, z, d in zip(target1_x, target1_y, target1_z, deposition1):
            if idx % 2 == 0:
                f1.write(f"CONST robtarget Target_{num}:=[[{x+250}, {y+250}, {z+3}],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
                num += 1
            idx += 1

    with open("ROB1_move.txt", "w") as f2:
        num = 1
        collect_time_idx = 0
        prepos = None

        for i, d in enumerate(deposition1):
            if collect_time_idx < len(collect_time) and i == int(collect_time[collect_time_idx]) + 1:
                f2.write(f"WaitSyncTask sync1,all_tasks;\n")
                f2.write(f"SyncMoveOn sync2,all_tasks;\n")
                f2.write(f"mhome;\n")
                f2.write(f"SyncMoveOff sync2;\n")
                collect_time_idx += 1

            if i % 2 == 1:
                continue

            curpos = np.array([target1_x[i], target1_y[i], target1_z[i]])

            if np.allclose(curpos, agent_models['robot1'].home_position):
                if prepos is None or not np.allclose(prepos, agent_models['robot1'].home_position):
                    f2.write(f"MoveL Target_{num},v300,fine,Weldgun_1\WObj:=Workobject_1;\n")

                if prepos is not None and np.allclose(prepos, agent_models['robot1'].home_position):
                    f2.write(f"WaitTime \\InPos,0.01;\n")    
            else:
                if d:
                    f2.write(f"MoveL Target_{num},v100,z0,Weldgun_1\WObj:=Workobject_1;\n")
                else:
                    f2.write(f"MoveL Target_{num},v300,z0,Weldgun_1\WObj:=Workobject_1;\n")
            num += 1
            prepos = curpos

    with open("ROB2_target.txt", "w") as f3:
        idx = 0
        num = 1
        for x, y, z, d in zip(target2_x, target2_y, target2_z, deposition2):
            if idx % 2 == 0:
                f3.write(f"CONST robtarget Target_{num}:=[[{x+250}, {y+250}, {z+3}],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
                num += 1
            idx += 1
    
    with open("ROB2_move.txt", "w") as f4:
        num = 1
        collect_time_idx = 0
        prepos = None

        for i, d in enumerate(deposition2):
            if collect_time_idx < len(collect_time) and i == int(collect_time[collect_time_idx]) + 1:
                f4.write(f"WaitSyncTask sync1,all_tasks;\n")
                f4.write(f"SyncMoveOn sync2,all_tasks;\n")
                f4.write(f"mhome;\n")
                f4.write(f"SyncMoveOff sync2;\n")
                collect_time_idx += 1

            if i % 2 == 1:
                continue

            curpos = np.array([target2_x[i], target2_y[i], target2_z[i]])

            if np.allclose(curpos, agent_models['robot2'].home_position):
                if prepos is None or not np.allclose(prepos, agent_models['robot2'].home_position):
                    f4.write(f"MoveL Target_{num},v300,fine,Weldgun_2\WObj:=Workobject_2;\n")

                if prepos is not None and np.allclose(prepos, agent_models['robot2'].home_position):
                    f4.write(f"WaitTime \\InPos,0.01;\n")    
            else:
                if d:
                    f4.write(f"MoveL Target_{num},v100,z0,Weldgun_2\WObj:=Workobject_2;\n")
                else:
                    f4.write(f"MoveL Target_{num},v300,z0,Weldgun_2\WObj:=Workobject_2;\n")
            num += 1
            prepos = curpos

    with open("ROB3_target.txt", "w") as f5:
        idx = 0
        num = 1
        for x, y, z, d in zip(target3_x, target3_y, target3_z, deposition3):
            if idx % 2 == 0:
                f5.write(f"CONST robtarget Target_{num}:=[[{x+250}, {y+250}, {z+3}],[0,1,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n")
                num += 1
            idx += 1
    
    with open("ROB3_move.txt", "w") as f6:
        num = 1
        collect_time_idx = 0
        prepos = None

        for i, d in enumerate(deposition3):
            if collect_time_idx < len(collect_time) and i == int(collect_time[collect_time_idx]) + 1:
                f6.write(f"WaitSyncTask sync1,all_tasks;\n")
                f6.write(f"SyncMoveOn sync2,all_tasks;\n")
                f6.write(f"mhome;\n")
                f6.write(f"SyncMoveOff sync2;\n")
                collect_time_idx += 1

            if i % 2 == 1:
                continue

            curpos = np.array([target3_x[i], target3_y[i], target3_z[i]])

            if np.allclose(curpos, agent_models['robot3'].home_position):
                if prepos is None or not np.allclose(prepos, agent_models['robot3'].home_position):
                    f6.write(f"MoveL Target_{num},v300,fine,Weldgun_3\WObj:=Workobject_3;\n")

                if prepos is not None and np.allclose(prepos, agent_models['robot3'].home_position):
                    f6.write(f"WaitTime \\InPos,0.01;\n")    
            else:
                if d:
                    f6.write(f"MoveL Target_{num},v100,z0,Weldgun_3\WObj:=Workobject_3;\n")
                else:
                    f6.write(f"MoveL Target_{num},v300,z0,Weldgun_3\WObj:=Workobject_3;\n")
            num += 1
            prepos = curpos



    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.2)

    sub_x = [SUBSTRATE_SIZE * -0.5, SUBSTRATE_SIZE * 0.5]  # substrate_x
    sub_y = [SUBSTRATE_SIZE * -0.5, SUBSTRATE_SIZE * 0.5]  # substrate_y

    vertices = [[[sub_x[0], sub_y[0], 0.0],
                 [sub_x[1], sub_y[0], 0.0],
                 [sub_x[1], sub_y[1], 0.0],
                 [sub_x[0], sub_y[1], 0.0]]]

    substrate = Poly3DCollection(vertices, facecolor='gray', linewidths=1.0, edgecolor='black', alpha=0.1)
    ax.add_collection3d(substrate)

    segments1, segments2, segments3 = [], [], []
    colors1, colors2, colors3 = [], [], []
    linewidths1, linewidths2, linewidths3 = [], [], []

    segments1.append([[target1_x[0], target1_y[0], target1_z[0]], [target1_x[0], target1_y[0], target1_z[0]]])
    segments2.append([[target2_x[0], target2_y[0], target2_z[0]], [target2_x[0], target2_y[0], target2_z[0]]])
    segments3.append([[target3_x[0], target3_y[0], target3_z[0]], [target3_x[0], target3_y[0], target3_z[0]]])
    colors1.append('red'   if deposition1[0] and deposition1[0] else 'gray')
    colors2.append('green' if deposition2[0] and deposition2[0] else 'gray')
    colors3.append('blue'  if deposition3[0] and deposition3[0] else 'gray')
    linewidths1.append(3 if deposition1[0] and deposition1[0] else 1)
    linewidths2.append(3 if deposition2[0] and deposition2[0] else 1)
    linewidths3.append(3 if deposition3[0] and deposition3[0] else 1)

    line_collection1 = Line3DCollection(segments1, colors=colors1, linewidths=linewidths1)
    line_collection2 = Line3DCollection(segments2, colors=colors2, linewidths=linewidths2)
    line_collection3 = Line3DCollection(segments3, colors=colors3, linewidths=linewidths3)

    ax.add_collection3d(line_collection1)
    ax.add_collection3d(line_collection2)
    ax.add_collection3d(line_collection3)


    def create_cylinder(center_x, center_y, center_z, color, r=5, h=50, resolution=20):
        x = r * np.cos(np.linspace(0, 2 * np.pi, resolution))
        y = r * np.sin(np.linspace(0, 2 * np.pi, resolution))
        z_bottom = np.full_like(x, 0)
        z_top = np.full_like(x, h)

        verts = []
        for i in range(resolution - 1):
            verts.append([
                [center_x + x[i],   center_y + y[i],   center_z + z_bottom[i]],
                [center_x + x[i+1], center_y + y[i+1], center_z + z_bottom[i+1]],
                [center_x + x[i+1], center_y + y[i+1], center_z + z_top[i+1]],
                [center_x + x[i],   center_y + y[i],   center_z + z_top[i]],
            ])

        verts.append([[center_x + x[i], center_y + y[i], center_z + z_bottom[i]] for i in range(resolution)])
        verts.append([[center_x + x[i], center_y + y[i], center_z + z_top[i]] for i in range(resolution)])

        return Poly3DCollection(verts, facecolors=color, linewidths=0.5, alpha=0.2)
    

    cylinder1 = create_cylinder(target1_x[0], target1_y[0], target1_z[0], 'red')
    cylinder2 = create_cylinder(target2_x[0], target2_y[0], target2_z[0], 'green')
    cylinder3 = create_cylinder(target3_x[0], target3_y[0], target3_z[0], 'blue')

    ax.add_collection3d(cylinder1)
    ax.add_collection3d(cylinder2)
    ax.add_collection3d(cylinder3)

    ax.set_title("WAAM Simulation")
    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_zlim(-50, 550)

    ax_slider = plt.axes([0.2, 0.13, 0.6, 0.03])  # [left, bottom, width, height]
    time_slider = Slider(ax_slider,
                         'Time',
                         int(schedule.start_time()),
                         int(schedule.end_time()),
                         valinit=int(schedule.start_time()),
                         valstep=1)
    
    ax_button = plt.axes([0.4, 0.05, 0.2, 0.05])
    button = Button(ax_button, '▶ Play')
    animating = False

    legend_elements = [
        Line2D([0], [0], color='red',   lw=3, label='robot1'),
        Line2D([0], [0], color='green', lw=3, label='robot2'),
        Line2D([0], [0], color='blue',  lw=3, label='robot3'),
    ]
    ax.legend(handles=legend_elements, loc='upper right')


    def update(val):
        time = time_slider.val
        pretime = len(segments1)

        if pretime < time:
            for i in range(pretime, time):
                segments1.append([[target1_x[i], target1_y[i], target1_z[i]], [target1_x[i+1], target1_y[i+1], target1_z[i+1]]])
                segments2.append([[target2_x[i], target2_y[i], target2_z[i]], [target2_x[i+1], target2_y[i+1], target2_z[i+1]]])
                segments3.append([[target3_x[i], target3_y[i], target3_z[i]], [target3_x[i+1], target3_y[i+1], target3_z[i+1]]])
                colors1.append('red'   if deposition1[i] and deposition1[i+1] else 'gray')
                colors2.append('green' if deposition2[i] and deposition2[i+1] else 'gray')
                colors3.append('blue'  if deposition3[i] and deposition3[i+1] else 'gray')
                linewidths1.append(3 if deposition1[i] and deposition1[i+1] else 0.5)
                linewidths2.append(3 if deposition2[i] and deposition2[i+1] else 0.5)
                linewidths3.append(3 if deposition3[i] and deposition3[i+1] else 0.5)
        else:
            for i in range(time, pretime):
                segments1.pop()
                segments2.pop()
                segments3.pop()
                colors1.pop()
                colors2.pop()
                colors3.pop()
                linewidths1.pop()
                linewidths2.pop()
                linewidths3.pop()

        line_collection1.set_segments(segments1)
        line_collection2.set_segments(segments2)
        line_collection3.set_segments(segments3)
        line_collection1.set_color(colors1)
        line_collection2.set_color(colors2)
        line_collection3.set_color(colors3)
        line_collection1.set_linewidth(linewidths1)
        line_collection2.set_linewidth(linewidths2)
        line_collection3.set_linewidth(linewidths3)

        global cylinder1, cylinder2, cylinder3
        cylinder1.remove()
        cylinder2.remove()
        cylinder3.remove()

        cylinder1 = create_cylinder(target1_x[time], target1_y[time], target1_z[time], 'red')
        cylinder2 = create_cylinder(target2_x[time], target2_y[time], target2_z[time], 'green')
        cylinder3 = create_cylinder(target3_x[time], target3_y[time], target3_z[time], 'blue')

        ax.add_collection3d(cylinder1)
        ax.add_collection3d(cylinder2)
        ax.add_collection3d(cylinder3)

        fig.canvas.draw_idle()


    time_slider.on_changed(update)


    def advance_slider(timer_event):
        if not animating:
            return
        
        current = time_slider.val
        max_val = time_slider.valmax
        next_val = current + 10 if current < max_val else time_slider.valmin
        time_slider.set_val(next_val)


    timer = fig.canvas.new_timer(interval=100)
    timer.add_callback(advance_slider, None)
    timer.start()


    def toggle_animation(event):
        global animating
        animating = not animating
        button.label.set_text('■ Stop' if animating else '▶ Play')
        fig.canvas.draw_idle()


    button.on_clicked(toggle_animation)

    plt.show()

else:
    visualize_toolpath(toolpath, backend="matplotlib", color_method="tool")

