#!/bin/zsh

dst=$(rospack find clcbs_gazebo)
rosrun clcbs_setup setup_heightmap.py -i "$1" -o "$dst/models/heightmap/map/map.png"
rosrun clcbs_setup setup_mapserver.py -i "$1" -o "$dst/models/heightmap/map.yaml"
rosrun clcbs_setup setup_gazebo.py -i "$1" -o "$dst/models/heightmap/model.sdf"
rosrun clcbs_setup setup_launch.py -i "$1" -o "$dst/launch/clcbs_config.launch"