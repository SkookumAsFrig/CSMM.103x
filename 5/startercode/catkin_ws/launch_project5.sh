source devel/setup.bash
roslaunch motion_planning mp.launch &
rosrun motion_planning marker_control.py &
rosrun motion_planning motion_planning.py &

