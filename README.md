
This is a monocular visual odometry project using the KITTI dataset.

The project is created with docker and ROS2 jazzy.


### Run
```SHELL
docker compose up --build

# In another terminal
docker exec -it vio bash
colcon build
. install/local_setup.bash
ros2 run vio visual_odometry

# In another terminal
docker exec -it vio bash
rviz2
# Remember to add the two posearray topics and set them to different colors
# The two topics represent the ground truth data and visual odometry data from the monocular camera
```


