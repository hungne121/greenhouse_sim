
# Greenhouse Simulation (Pepper Robot)

## Requirements
- ROS Noetic
- pepper_description

## Install dependencies
```bash
sudo apt install ros-noetic-pepper-description
```

## System with Robot full interaction
```bash
roslaunch greenhouse_sim greenhouse_hri_system
```
## Run human model
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/human/cmd_vel
```
## Bring-up Robot and World
```bash
roslaunch greenhouse_sim greenhouse_bringup.launch
```
## Slam Gmapping
```bash
roslaunch greenhouse_sim greenhosue_gmapping.launch
```
## Run navigation
```bash
roslaunch greenhouse_sim greenhosue_navigation.launch
```
## Video Mo Phong
Video/go_to_one_plant.mp4
