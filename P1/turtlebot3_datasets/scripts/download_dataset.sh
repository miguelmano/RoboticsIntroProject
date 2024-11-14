#!/bin/bash
path_to_pkg=`rospack find turtlebot3_datasets`/data

# Download and extract the first tar.gz (existing dataset)
# gdown -O $path_to_pkg/ir_labs.tar.gz https://drive.google.com/uc?id=1-QdzXIoh0ltdDNGuffLqzsPZLnLwSjAj && tar -xvf $path_to_pkg/ir_labs.tar.gz -C $path_to_pkg

# Download and extract the new tar.gz (your new rosbag)
gdown -O $path_to_pkg/real_robot_rosbag.tar.gz https://drive.google.com/uc?id=1sDR_OelXr9GHtt_8MoH_BPHkH27IakAl && tar -xvf $path_to_pkg/real_robot_rosbag.tar.gz -C $path_to_pkg

