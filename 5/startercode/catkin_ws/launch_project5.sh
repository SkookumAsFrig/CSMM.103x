echo -e 'run the following commands one by one in bash:\nsource devel/setup.bash\nroslaunch motion_planning mp.launch &\nrosrun motion_planning marker_control.py &\nrosrun motion_planning motion_planning.py &\nrosrun rviz rviz\n\nuse kill $(jobs -p) to exit background jobs'

