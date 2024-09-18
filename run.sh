#!/bin/bash

if [ ! -f "bin/rog_map" ]; then
    echo "Error: bin/rog_map not found. Please recomplie the code."
    exit 1
fi

read -p "Enter the input path: " input_path
read -p "Enter the start sequence: " start
read -p "Enter the end sequence: " end
read -p "Enter the poses file (e.g., keyframe_poses.csv): " poses
bin/rog_map --input_path "$input_path" --start "$start" --end "$end" --poses "$poses" --resolution 0.1 --p_hit 0.70 --p_miss 0.30 --p_min 0.12 --p_max 0.97 --p_occ 0.80 --p_free 0.30
