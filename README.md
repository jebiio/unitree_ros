# 빌드
* 환경 : ubuntu 20.04, gazebo11, ros1 noetic

```bash
sudo apt-get install ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
mkdir ~/catkin_ws/src -p
cd ~/catkin_ws/src
git clone -b subakio https://github.com/jebiio/unitree_ros.git
cd ~/catkin_ws
catkin build
```
# 실행
## 모델
* a1
* aliengo
* b1
* go1
* laikago
* z1

## 환경설정
```bash
cd ~/catkin_ws
. devel/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor/models
```

## Rviz에서 모델 확인하기
```bash
roslaunch laikago_description laikago_rviz.launch
#roslaunch laikago_description <model_name>_rviz.launch
```

## gazebo simulation
```bash
roslaunch unitree_gazebo my_normal.launch
#roslaunch unitree_gazebo my_normal.launch rname:=<robot_name> wname:=<world_name>
```

## teleop
```bash
rosrun unitree_controller unitree_teleop
```

## navigation
* gammping
```bash
roslaunch unitree_navigation start_mapping.launch
```
* localization
```bash
roslaunch unitree_navigation start_localization.launch
```
* navigation
```bash
roslaunch unitree_navigation start_navigation.launch
```
