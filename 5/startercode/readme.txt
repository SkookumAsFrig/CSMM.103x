https://courses.edx.org/courses/course-v1:ColumbiaX+CSMM.103x+1T2017/
How do run this project in my own Ubuntu machine?
Launch Project 5, then in Vocareum click Actions>Download Starter code. This will download all the files you need to make the project run locally in your computer.
Install the needed ROS package(s). Run the following lines on your terminal:
sudo apt-get update
sudo apt-get install python-wstool ros-kinetic-moveit*

Replace kinetic with the ROS version that you are running on your local machine.
IGNORE all the files other than catkin_ws and lwr_defs folders. Put the catkin_ws and lwr_defs in your home directory.
The downloaded files are structured as a catkin workspace. You can either use this structure directly (as downloaded) and build the workspace using the "catkin_make" command or use whatever catkin workspace you already had, and just copy the packages inside your own src folder and run the catkin_make command. If you are having troubles with this, you should review the first ROS tutorial "Installing and configuring your ROS Environment".
Once you have a catkin workspace with the packages inside the src folder, you are ready to work on your project without having to make any changes in any of the files. Navigate to the catkin workspace folder and build the workspace using the command "catkin_make".
NOTE: You can source both your ROS distribution and your catkin workspace automatically everytime you open up a terminal automatically by editing the ~/.bashrc file in your home directory. For example if your ROS distribution is Indigo, and your catkin workspace is called "project5_ws" (and is located in your home directory) then you can add the following at the end of your .bashrc file:

source /opt/ros/kinetic/setup.bash
echo "ROS Kinetic was sourced"
source ~/project5_ws/devel/setup.bash
echo "Project5 workspace was sourced"

This way every time you open up a terminal, you will already have your workspace sourced, such that ROS will have knowledge of the packages there.
Before moving forward, if you haven't followed the instructions on step 6, you will need to source ROS and the catkin workspace every time you open a new terminal. To run the project, first open up a terminal and run "roslaunch motion_planning mp.launch". In the second terminal, run "rosrun motion_planning marker_control.py". Note that you do NOT have to run "roscore" as roslaunch includes all the necessary packages.
On another 2 separate terminals, run "rosrun motion_planning motion_planning.py" (this is what you need to edit to complete the project), and as always "rosrun rviz rviz" to visualize the robot.
On rviz, you will need to add a RobotModel, InteractiveMarker, and Marker. When you add the InteractiveMarker, click on "InteractiveMarker" to expand it, and select /control_markers/update as the update topic. You shouldn't need to do anything for the RobotModel and the Marker.

Commands have been compiled into shell script in catkin_ws directory. Do source launch_project5.sh, and rosrun rviz rviz. Change Gloabal Fixed Frame in Rviz to world_link, add RobotModel, InteractiveMarker, and Marker. Right click on InteractiveMarker to run.

