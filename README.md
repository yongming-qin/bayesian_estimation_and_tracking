# bayesian_estimation_and_tracking

In a ROS environment:
put these file in /src
`catkin_make`
source the devel setup file

## simulation of a turtlebot and a human walker
```export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch turtlebot3_gazebo human_tracking.launch```

## human movement
`rosrun deterministic_models human_movement`
TODO: change the name

rosrun image_analysis color_detection _topic_image:=/camera/color/image_raw

rosrun pc_analysis balloon_position _topic_pointcloud:=/camera/depth/points _bool_noise:=true _noise_variance:=0.5


```
sudo apt install ros-melodic-slam-gmapping

```