#! /bin/bash

function kill_roslaunch() {
    echo sending SIGKILL to roslaunch...
    sudo pkill -f ros
    return 0
}

trap "echo signal trapped.;kill_roslaunch; exit" 1 2 3 15

SIMULATION_LAUNCH=`rospack find drcsim_gazebo`/launch/vrc_task_1.launch
CMD_NAMESPACE=/drc_vehicle
USE_HANDLE="true"

# parse arguments
if [ $# -ge 3 ]; then
    SIMULATION_LAUNCH=$1
    CMD_NAMESPACE=$2
    USE_HANDLE=$3
fi

source ${HOME}/ros/hydro/devel/setup.sh
roslaunch gazebo_drive_simulator gazebo_drive_simulator.launch SIMULATION_LAUNCH:=${SIMULATION_LAUNCH} CMD_NAMESPACE:=${CMD_NAMESPACE} USE_HANDLE:=${USE_HANDLE} &
sleep 40
rostopic pub --once /drc_world/robot_exit_car geometry_msgs/Pose '{}' # exit from wall
sleep 20
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{position: {y: -0.6, z: -0.2}}' # '{position: {y: -0.6}}'
 # ride to drc_vehicle  # if you change initial position, you have to change gazebo_drive_simulator/launch/gazebo_drive_simulator.launch
sleep 20
rostopic pub --once /atlas/control_mode std_msgs/String "User" # Modify UserMode
rosrun gazebo_drive_simulator traj_yaml.py `rospack find gazebo_drive_simulator`/config/atlas_sitting_pose.yaml sitting-arm-up # sitting
rostopic pub --once ${CMD_NAMESPACE}/hand_brake/cmd std_msgs/Float64 '{ data : 0 }' # disable hand brake
roslaunch gazebo_drive_simulator multisense_sl_relay.launch & # for PointCloud2

roslaunch drive_recognition extract_obstacle_cloud.launch USE_DRC:="true" USE_VRC:="false" &
roslaunch drive_recognition obstacle_detection.launch &
rosrun drive_recognition CalculateVelocityFromOdometry.py &
roslaunch drive_recognition cheat_goal_direction.launch &
roslaunch gazebo_drive_simulator polaris_interactive_marker.launch &


while true
do
    sleep 1 # wait termination
done
