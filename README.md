# `Sawback` Pick and Place

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

### Rethink Workspace

To install the `Rethink` woskspace, we need to build it from its source
```
mkdir -p rethink_ws/src
cd rethink_ws
vcs import --input 
```