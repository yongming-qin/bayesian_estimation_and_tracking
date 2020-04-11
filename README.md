# bayesian_estimation_and_tracking

In a ROS environment:
put these file in /src
`catkin_make`
source the devel setup file

## simulation of a turtlebot and a human walker
```export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch turtlebot_gazebo human_tracking.launch```

## human movement
`rosrun deterministic_models human_movement`
TODO: change the name