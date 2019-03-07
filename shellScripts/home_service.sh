#!/bin/sh
catkin_dir=/home/workspace/catkin_ws
catkin_src_dir=$catkin_dir/src

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_src_dir/world/myWorld.world" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$catkin_src_dir/world/myMap_new.yaml" &
sleep 5
xterm -e "rosrun rviz rviz -d $catkin_src_dir/rvizConfig/home_service_robot.rviz" &
sleep 5
xterm -e "source $catkin_dir/devel/setup.bash; rosrun add_markers add_markers_node"  &
sleep 2
xterm -e "source $catkin_dir/devel/setup.bash; rosrun pick_objects pick_objects_node"