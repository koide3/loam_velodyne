# LOAM

Original repository: https://github.com/laboshinl/loam_velodyne

## Build
```bash
cd loam/docker
./build.sh
```

## Run

### On host:
```bash
roscore
```

```bash
rosparam set use_sim_time true
rviz -d loam/rviz_cfg/loam_velodyne.rviz
```

```bash
rosbag play --clock nsh_indoor_outdoor.bag
```
nsh_indoor_outdoor.bag: https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view


### On docker image:
```bash
cd loam/docker
./run.sh

roslaunch loam_velodyne loam_velodyne.launch
```

![loam](https://user-images.githubusercontent.com/31344317/98347880-5da2df80-205b-11eb-8aae-abfd8fc67f70.gif)

