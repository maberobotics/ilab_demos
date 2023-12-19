# ilab_biman_demo
This package contains the setup for the bimanual demo featuring the UR and Franka robots.

## Usage 

On the control PC of the Franka robot run 
```shell
$ ros2 launch ilab_franka_bringup ilab_franka.launch.py merge_joint_states:=false namespace:=franka/ use_fake_hardware:=false start_rviz:=false use_planning:=false
```

On the control PC of the UR robot run 
```shell
$ ros2 launch ilab_ur_bringup ilab_ur.launch.py merge_joint_states:=false namespace:=ur/ use_fake_hardware:=false start_rviz:=false use_planning:=false
```

On the demo PC (ex UR control PC) run 
```shell
$ ros2 launch ilab_biman_demo ilab_biman_demo.launch.py 
```

Now Moveit2 is started and configured and you can plan both robots using the Moveit2 RViz gui interface.

To run the bimanual pick and place run

```shell
$ ros2 launch ilab_biman_demo ilab_biman_demo_pick_and_place.launch.py
```
This node can freely be adapted for demo purposes. In its current version the movements are:
1. Close UR gripper on an object, in its current pose
2. Lift UR 20cm
3. Close Franka gripper
4. Open UR gripper
5. Lift Franka 10cm
6. Lower Franka 10cm
7. Close UR gripper
8. Open Franka gripper
9. Lower UR 20cm
10. Open UR gripper
These steps are looped untill node sigterm. 