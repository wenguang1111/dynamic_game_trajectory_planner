#!/bin/bash

cd build
cmake ..
make
./dynamic_game_trajectory_planner
cd ..
python3 plot_trajectories.py
