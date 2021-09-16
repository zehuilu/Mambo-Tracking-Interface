#!/bin/bash

# Start Mambo Mocap Qualisys for {Online Planner, Mambo tracking controller},
# and Mambo tracking controller

# Usage:
# $ ./launch_mambo.sh mambo_id run_mambo_flag

mambo_id=$1
run_mambo_flag=$2

if [ -z "$mambo_id" ]
then
    echo "mambo_id = 1"
else
    echo "mambo_id = ${mambo_id}"
fi

# mambo_01 Mocap Qualisys for Online Planner
gnome-terminal -- bash -c "python3 scripts_aimslab/run_mocap_qualisys_for_planner.py $mambo_id"

# mambo_01 Mocap Qualisys for Mambo tracking controller
gnome-terminal -- bash -c "python3 scripts_aimslab/run_mocap_qualisys.py $mambo_id"

# wait 1 second
sleep 1

# if the second argument run_mambo_flag doesn't exist or equals to true, run Mambo tracking controller
# else, do nothing
if [ -z "$run_mambo_flag" ]
then
    echo "run_mambo_flag = true"
    gnome-terminal -- bash -c "python3 scripts_aimslab/run_mambo.py $mambo_id"
elif [ $run_mambo_flag == "true" ]
then
    echo "run_mambo_flag = ${run_mambo_flag}"
    gnome-terminal -- bash -c "python3 scripts_aimslab/run_mambo.py $mambo_id"
else
    echo "run_mambo_flag = false"
fi
