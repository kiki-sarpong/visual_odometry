
This is a monocular visual odometry project using the KITTI dataset.

The project is created with docker and ROS2 jazzy.


## Run
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
## Results
Results for monocular visual odometry on KITTI dataset 00 && 03.</br>
Generated odometry works well but subject to heavy drift over time.</br>
**Adding Bundle Adjustment can be used to minimize reprojection errors**

Red->ground truth.</br>
Green->generated odometry.
![image](https://github.com/kiki-sarpong/visual_odometry/blob/main/images/capture_kitti_03.PNG?raw=true)
![image](https://github.com/kiki-sarpong/visual_odometry/blob/main/images/kitti_capture_00.PNG?raw=true)

