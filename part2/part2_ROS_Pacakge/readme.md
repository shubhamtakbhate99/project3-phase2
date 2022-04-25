Download package "search" whereever you want in catkin workspace
catkin build
source the workspace
source devel/setup.bash
rosrun search my.launch


#We are by default giving the goal to be 900,900
#You can change the goal in the program if required.(Line #163 of the turtle.py in search/src)

#The codes will work with python3 
#ROS noetic is prefered for ROS 
# go to search/src and type 

$chmod +x turtle.py
