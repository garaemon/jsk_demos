#!/usr/bin/env python
# coding: UTF-8

import sys
import csv
import time
import numpy as np
from scipy import signal, interpolate
import matplotlib as mpl
import matplotlib.pyplot as plt
from drc_task_common.optimization_container import QualityTable, OptimizationContainer
plt.ion()                       # enable interactive mode
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--type', default="valve_inplace", type=str)
parser.add_argument('--p0', default=1.0, type=float)
parser.add_argument('--m0', default=1.0, type=float)
parser.add_argument('--e0', default=4.0, type=float)
parser.add_argument('--p1', default=0.5, type=float)
parser.add_argument('--p-tracking', default=0.0, type=float)
parser.add_argument('--m1', default=0.5, type=float)
parser.add_argument('--e1', default=1.0, type=float)
parser.add_argument('time', type=float)
parser.add_argument('--wait', action="store_true")
parser.add_argument('--quiet', action="store_true")
parser.add_argument('--incremental', action="store_true")
parser.add_argument('--distance', type=float)
args = parser.parse_args()
deadline_time = args.time

if args.type not in ["valve_inplace", "door_inplace", "valve_walk", "door_walk", "valve_walk2", "door_walk2"]:
    raise Exception("Unknown type: %s" % (args.type))

container = OptimizationContainer()
k = 1.0                         # for step cost
if args.type == "valve_inplace":
    initial_dx = 10
    initial_collision = 10
    initial_traj = 20
    initial_speed_factor = 5
    p_table = QualityTable("q_{p0}", "package://drc_task_common/profile_data/recognition/valve_detection.csv", args.p0)
    m_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_average_q_mono.csv", args.m0)
    m_all_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_average_q.csv", args.m0)
    e_table = QualityTable("q_{e0}", "package://drc_task_common/profile_data/execution/valve/jaxon_red_valve_zmp_ee_normalized.csv", 1)
    container.register_quality_table(p_table, p_table.lookup_value('dx', initial_dx, 'time'))
    container.register_quality_table(m_table, m_all_table.lookup_value2('collision-num', 'trajectory-num',
                                                                        initial_collision, initial_traj,
                                                                        'time'))
    container.register_quality_table(e_table, e_table.lookup_value('speed-factor', initial_speed_factor, 'time'))
elif args.type == "door_inplace":
    initial_dx = 10
    initial_collision = 10
    initial_traj = 20
    initial_speed_factor = 5
    p_table = QualityTable("q_{p0}", "package://drc_task_common/profile_data/recognition/door_detection.csv", args.p0)
    m_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_door_ik_average_q_mono.csv", args.m0)
    m_all_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_door_ik_average_q.csv", args.m0)
    e_table = QualityTable("q_{e0}", "package://drc_task_common/profile_data/execution/door/jaxon_red_door_zmp_ee_normalized.csv", 1)
    container.register_quality_table(p_table, p_table.lookup_value('dx', initial_dx, 'time'))
    container.register_quality_table(m_table, m_all_table.lookup_value2('collision-num', 'trajectory-num',
                                                                        initial_collision, initial_traj,
                                                                        'time'))
    container.register_quality_table(e_table, e_table.lookup_value('speed-factor', initial_speed_factor, 'time'))
elif args.type == "valve_walk":
    initial_dx = 10
    initial_collision = 10
    initial_traj = 20
    initial_speed_factor = 5
    steps = args.distance / 0.2
    distance_factor = 1.0 / (1.0 + k * steps * steps)
    print "distance_factor:", distance_factor
    p0_table = QualityTable("q_{p0}", "package://drc_task_common/profile_data/recognition/valve_detection.csv", args.p0 * distance_factor)
    m0_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_stand2_average_mono.csv", args.m0 * distance_factor)
    m0_all_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_stand2_average.csv", args.m0  * distance_factor)
    e_table = QualityTable("q_{e0}", "package://drc_task_common/profile_data/execution/valve/jaxon_red_valve_zmp_ee_normalized.csv", 1)
    container.register_quality_table(p0_table, p0_table.lookup_value('dx', initial_dx, 'time'))
    container.register_quality_table(m0_table, m0_all_table.lookup_value2('collision-num', 'trajectory-num',
                                                                         initial_collision, initial_traj,
                                                                         'time'))
    container.register_quality_table(e_table, e_table.lookup_value('speed-factor', initial_speed_factor, 'time'))
elif args.type == "valve_walk2":
    initial_dx = 10
    initial_collision = 10
    initial_traj = 20
    initial_speed_factor = 5
    steps = args.distance / 0.2
    distance_factor = 1.0 / (1.0 + k * steps * steps)
    print "distance_factor:", distance_factor
    p0_table = QualityTable("q_{p0}", "package://drc_task_common/profile_data/recognition/valve_detection.csv", args.p0 * distance_factor)
    p1_table = QualityTable("q_{p1}", "package://drc_task_common/profile_data/recognition/valve_detection.csv", 1.0)
    m0_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_stand2_average_mono.csv", args.m0 * distance_factor)
    m1_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_average_q_mono.csv", args.m1)
    m1_all_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_average_q.csv", args.m1)
    m0_all_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_stand2_average.csv", args.m0  * distance_factor)
    e_table = QualityTable("q_{e0}", "package://drc_task_common/profile_data/execution/valve/jaxon_red_valve_zmp_ee_normalized.csv", 1)
    container.register_quality_table(p0_table, p0_table.lookup_value('dx', initial_dx, 'time'))
    container.register_quality_table(p1_table, p1_table.lookup_value('dx', initial_dx, 'time'))
    container.register_quality_table(m0_table, m0_all_table.lookup_value2('collision-num', 'trajectory-num',
                                                                         initial_collision, initial_traj,
                                                                         'time'))
    container.register_quality_table(m1_table, m1_all_table.lookup_value2('collision-num', 'trajectory-num',
                                                                         initial_collision, initial_traj,
                                                                         'time'))
    container.register_quality_table(e_table, e_table.lookup_value('speed-factor', initial_speed_factor, 'time'))
elif args.type == "door_walk2":
    initial_dx = 10
    initial_collision = 10
    initial_traj = 20
    initial_speed_factor = 5
    steps = args.distance / 0.2
    distance_factor = 1.0 / (1.0 + k * steps * steps)
    print "distance_factor:", distance_factor
    p0_table = QualityTable("q_{p0}", "package://drc_task_common/profile_data/recognition/epsilon_plane_sigma.csv", args.p0 * distance_factor)
    p1_table = QualityTable("q_{p1}", "package://drc_task_common/profile_data/recognition/epsilon_plane_sigma.csv", 1.0)
    m0_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_door_ik_average_q_mono.csv", args.m0 * distance_factor)
    m1_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_door_ik_average_q_mono.csv", args.m1)
    m0_all_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_door_ik_stand2_average.csv", args.m0 * distance_factor)
    m1_all_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_door_ik_average_q.csv", args.m1)
    e_table = QualityTable("q_{e0}", "package://drc_task_common/profile_data/execution/door/jaxon_red_door_zmp_ee_normalized.csv", 1)
    container.register_quality_table(p0_table, p0_table.lookup_value('dx', initial_dx, 'time'))
    container.register_quality_table(p1_table, p1_table.lookup_value('dx', initial_dx, 'time'))
    container.register_quality_table(m0_table, m0_all_table.lookup_value2('collision-num', 'trajectory-num',
                                                                         initial_collision, initial_traj,
                                                                         'time'))
    container.register_quality_table(m1_table, m1_all_table.lookup_value2('collision-num', 'trajectory-num',
                                                                         initial_collision, initial_traj,
                                                                         'time'))
    container.register_quality_table(e_table, e_table.lookup_value('speed-factor', initial_speed_factor, 'time'))
elif args.type == "door_walk":
    initial_dx = 10
    initial_collision = 10
    initial_traj = 20
    initial_speed_factor = 5
    steps = args.distance / 0.2
    distance_factor = 1.0 / (1.0 + k * steps * steps)
    print "distance_factor:", distance_factor
    p0_table = QualityTable("q_{p0}", "package://drc_task_common/profile_data/recognition/epsilon_plane_sigma.csv", args.p0 * distance_factor)
    m0_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_door_ik_average_q_mono.csv", args.m0 * distance_factor)
    m0_all_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_door_ik_stand2_average.csv", args.m0 * distance_factor)
    e_table = QualityTable("q_{e0}", "package://drc_task_common/profile_data/execution/door/jaxon_red_door_zmp_ee_normalized.csv", 1)
    container.register_quality_table(p0_table, p0_table.lookup_value('dx', initial_dx, 'time'))
    container.register_quality_table(m0_table, m0_all_table.lookup_value2('collision-num', 'trajectory-num',
                                                                         initial_collision, initial_traj,
                                                                         'time'))
    container.register_quality_table(e_table, e_table.lookup_value('speed-factor', initial_speed_factor, 'time'))

    
# global variables
# door
# perception_csv = "epsilon_plane_sigma.csv"
# planning_csv = "ik_sigma_epsilon.csv"
# execution_csv = "zmp-door_epsilon_sigma.csv"
# all_planning_csv = "ik_sigma_epsilon_all.csv"
# tracking_csv = "tracking_sigma_epsilon.csv"
# initial_dx = 10.0               # 1cm downsample is default
# initial_track_d = 0.005
# initial_collision = 5
# initial_traj = 12
# initial_execution_sigma = 20.0198

ax = plt.figure().add_subplot(111)
ax.set_xlabel('$t$')
ax.set_ylabel('$q$')

# check direction



# p0_table = QualityTable("q_{p0}", "epsilon_plane_sigma.csv", args.p0)
# m0_table = QualityTable("q_{m0}", "ik_sigma_epsilon.csv", args.m0)
# m0_all_table = QualityTable("q_{m0}", "ik_sigma_epsilon_all.csv", args.m0)
# p1_table = QualityTable("q_{p1}", "epsilon_plane_sigma.csv", args.p1)
# m1_table = QualityTable("q_{m1}", "ik_sigma_epsilon.csv", args.m1)
# e0_table = QualityTable("q_{e0}", "zmp-door_epsilon_sigma.csv", args.e0)
# e1_table = QualityTable("q_{e1}", "zmp-door_epsilon_sigma.csv", args.e1)
# p_tracking_table = QualityTable("q_{p_{{\rm tracking}}}", tracking_csv, args.p_tracking)

# if args.p0 != 0.0:    
#     container.register_quality_table(p0_table, p0_table.lookup_value('dx', initial_dx, 'sigma'))
# if args.m0 != 0.0:
#     container.register_quality_table(m0_table, m0_all_table.lookup_value2('collision', 'trajectory',
#                                                                           initial_collision, initial_traj,
#                                                                           'sigma'))
# if args.e0 != 0.0:
#     container.register_quality_table(e0_table, initial_execution_sigma)
# if args.p1 != 0.0:
#     container.register_quality_table(p1_table, p1_table.lookup_value('dx', initial_dx, 'sigma'))
# if args.m1 != 0.0:
#     container.register_quality_table(m1_table, m0_all_table.lookup_value2('collision', 'trajectory',
#                                                                           initial_collision, initial_traj,
#                                                                           'sigma'))
# if args.e1 != 0.0:
#     container.register_quality_table(e1_table, initial_execution_sigma)
# if args.p_tracking != 0.0:
#     container.register_quality_table(p_tracking_table, p_tracking_table.lookup_value('d', initial_track_d, 'sigma'))
# print "Initial sigma:", container.current_sigma()
if container.current_time() < deadline_time:
    # print "Increase time"
    container.setDirectionPositve()
else:
    # print "Decrease time"
    container.setDirectionNegative()
if args.wait:
    raw_input()

while not container.is_converged(deadline_time):
    if not container.proc():
        container.draw(ax)
        break
    container.draw(ax)
    if args.incremental:
        container.printOverview2(deadline_time)
    if args.wait:
        raw_input()
if not args.incremental:
    if args.quiet:
        container.printOverview2(deadline_time)
    else:
        container.printOverview(deadline_time)
        raw_input()
