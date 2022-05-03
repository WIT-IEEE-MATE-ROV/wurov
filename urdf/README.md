To check for URDF syntax:
```
    xacro nugv.urdf.xacro > tmp.urdf && check_urdf tmp.urdf && rm tmp.urdf
```

Run URDF model on RVIZ:
```
    roslaunch wurov rviz_nugv.launch
```
