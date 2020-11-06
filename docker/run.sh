#!/bin/bash
docker run --net=host -it --rm \
           -v $(realpath ..):/root/catkin_ws/src/loam_velodyne \
           -w /root/catkin_ws/src/loam_velodyne \
           $@ \
           loam
