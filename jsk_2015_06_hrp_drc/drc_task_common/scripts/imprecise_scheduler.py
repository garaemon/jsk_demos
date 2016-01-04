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
parser.add_argument('--e0', default=0.0, type=float)
parser.add_argument('--p1', default=1.0, type=float)
parser.add_argument('--p-tracking', default=0.0, type=float)
parser.add_argument('--m1', default=1.0, type=float)
parser.add_argument('--e1', default=1.0, type=float)
parser.add_argument('sigma', type=float)
parser.add_argument('--wait', action="store_true")
parser.add_argument('--quiet', action="store_true")
parser.add_argument('--incremental', action="store_true")
args = parser.parse_args()
deadline_sigma = args.sigma

if args.type not in ["valve_inplace"]:
    raise Exception("Unknown type: %s" % (args.type))

container = OptimizationContainer()
if args.type == "valve_inplace":
    initial_dx = 10
    initial_collision = 10
    initial_traj = 20
    initial_speed_factor = 5
    p_table = QualityTable("q_{p0}", "package://drc_task_common/profile_data/recognition/epsilon_plane_sigma.csv", args.p0)
    m_table = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_average_q_mono.csv", args.m0)
    m_table_all = QualityTable("q_{m0}", "package://drc_task_common/profile_data/motion/jaxon_valve_ik_average_q.csv", args.m0)
    e_table = QualityTable("q_{e0}", "package://drc_task_common/profile_data/execution/valve/jaxon_red_valve_zmp_ee.csv", args.e0)
    container.register_quality_table(p_table, p_table.lookup_value('dx', initial_dx, 'sigma'))
    container.register_quality_table(m_table, m_all_table.lookup_value2('collision-num', 'trajectory-num',
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
if container.current_sigma() < deadline_sigma:
    # print "Increase sigma"
    container.setDirectionPositve()
else:
    # print "Decrease sigma"
    container.setDirectionNegative()
if args.wait:
    raw_input()

while not container.is_converged(deadline_sigma):
    if not container.proc():
        container.draw(ax)
        break
    container.draw(ax)
    if args.incremental:
        container.printOverview2(deadline_sigma)
    if args.wait:
        raw_input()
if not args.incremental:
    if args.quiet:
        container.printOverview2(deadline_sigma)
    else:
        container.printOverview(deadline_sigma)
        raw_input()
