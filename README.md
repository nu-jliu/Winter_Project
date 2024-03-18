# `Sawback` Pick and Place
 - **Author**: Allen Liu
 - MSR 2024

## Package Description
 - `object_detection` -
 - `arm_control`
 - `motion_control`
 - `vision_control`
 - `picker_interfaces`

 ## Package Setup

The following section detailed how to setup the packages requires for running the software.

### Ridgeback Packages
Install the ridgeback simulator on the PC via
```
sudo apt install ros-noetic-ridgeback-desktop ros-noetic-ridgeback-simulator
```

### Rethink Packeges

To install the `Rethink` woskspace, we need to build it from its source
```
mkdir -p rethink_ws/src
cd rethink_ws
vcs import --input https://raw.githubusercontent.com/nu-jliu/Winter_Project/main/rethink.repos
catkin_make
```

### Realsense Packages
Same for the `Realsense` workspace
```
mkdir -p realsense_ws/src
cd realsense_ws
vcs import --input https://raw.githubusercontent.com/nu-jliu/Winter_Project/main/realsense.repos
catkin_make
```

### Interbotix Packages
We also need to build the `Interbotix` workspace from its source
```
mkdir -p interbotix_ws/src
cd inertbotix_ws
vcs import --input https://raw.githubusercontent.com/nu-jliu/Winter_Project/main/interbotix.repos
catkin_make
```

## Setup ROS Environment
The the boot, the `base.launch` and `laser_slam.launch` in `nuridgeback_robot` would automatically launched in the software development. But it is launched under the `ROS MASTER` on `ridgeback` computer, but we would like to have everything run on `sawyer`'s master. So we need to make all ROS nodes running with `ROS_MASTER_URI=http://192.168.131.40:11311`. Run following command after sawyer competes boot

```
sudo systemctl stop ridgeback.service
consawyer
delrefs
roslaunch nuridgeback_robot base.launch
roslaunch nuridgeback_robot laser_slam.launch
```

## Deployment

To deploy all packages onto the ridgeback computer, run

```
rsync -av --delete --exclude '.git' --exclude '*.rviz' /home/allen/catkin_ws/src/ administrator@ridgeback:/home/administrator/allen_arm_ws/src
rsync -av --delete --exclude '.git' --exclude 'sawyer_moveit' --exclude '*.rviz' /home/allen/catkin_ws/src/ administrator@ridgeback:/home/administrator/allen_base_ws/src
rsync -av --delete --exclude '.git' /home/allen/rethink_ws/src/ administrator@ridgeback:/home/administrator/rethink_ws/src
rsync -av --delete --exclude '.git' --exclude '*.deb' /home/allen/realsense_ws/src/ administrator@ridgeback:/home/administrator/realsense_ws/src
rsync -av --delete --exclude '.git' /home/allen/interbotix_ws/src/ administrator@ridgeback:/home/administrator/interbotix_ws/src
```
