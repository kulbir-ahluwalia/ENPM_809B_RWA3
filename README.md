# ENPM_809B_RWA3

## Build Instructions for rwa 3
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.zsh
cd src/
#copy group4_rwa3 package here
cd ..
catkin_make
```

## Run the launch file ariac_manager.launch
```
cd catkin_ws
catkin_make --only-pkg-with-deps group4_rwa3
cd 809b
source ~/catkin_ws/devel/setup.zsh
#roslaunch group4_rwa3 ariac_manager.launch
roslaunch ariac_manager ariac_manager.launch
```

## Launching UR10 arm 1

```
cd 809b
source ~/809b/devel/setup.zsh
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
```

## Running main node
```
source ~/809b/devel/setup.zsh
source ~/catkin_ws/devel/setup.zsh
rosrun ariac_manager main_node
```

