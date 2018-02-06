#!/bin/bash
echo "runs: $1"
echo "start_random: $2"
echo "end_random: $3"
rosrun predictive_control trajectory_exporter.py --folder /home/bfb-ws/mpc_ws/src/predictive_control/output_data/ --base_frame arm_base_link --eef_frame arm_7_link --runs $1 --start_random $2 --end_random $3
