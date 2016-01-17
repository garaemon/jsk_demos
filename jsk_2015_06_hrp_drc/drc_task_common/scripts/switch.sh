#!/bin/bsah


for d in 0 0.1 0.3 0.5 1.0 1.5
do
    rosrun drc_task_common imprecise_scheduler.py --k 1.0 --type valve_walk 0 --distance $d --incremental --no-gui > /tmp/valve_walk_$d.csv
    rosrun drc_task_common imprecise_scheduler.py --k 1.0 --type valve_walk2 0 --distance $d --incremental --no-gui > /tmp/valve_walk2_$d.csv
done
pltcli /tmp/valve_walk*csv
