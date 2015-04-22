#!/bin/bash
file=$1_$(date +%Y-%m-%d-%H-%M-%S)
rosparam dump /path_follower ${file}.txt
rosbag record -O ${file}.bag /topics
