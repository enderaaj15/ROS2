# raaj_dynamic_tf2_cpp

The structure of the package for this assignment is shown below:

```
└── raaj_dynamic_tf2_cpp/
    ├── src/
    │   ├── windturbine_tower_base_tf2_broadcaster.cpp
    │   ├── tower_mid_static_frame_broadcaster.cpp
    │   ├── nacelle_dynamic_frame_broadcaster.cpp
    │   ├── hub_dynamic_frame_broadcaster.cpp
    │   ├── blades_static_frame_broadcaster.cpp
    │   └── turbine_rotating2.cpp
    ├── launch/
    │   ├── windturbine_base_tf2.launch.py
    │   ├── windturbine_fixed_frame_tf2.launch.py
    │   ├── windturbine_dynamic_frames_tf2.launch.py
    │   └── turbine_rotating_tf2.launch.py
    ├── rviz/
    │   └── windturbine_dynamic.rviz
    ├── include/
    ├── CMakeLists.txt
    ├── package.xml
    ├── LICENSE
    └── README.md
```

## 🎯️ Function 

This assigment will demonstrate tf2 application for broadcasting static and dynamic frames. To demonstrate this, I designed a horizontal-axis wind turbine. The specifications [Design Specifications of the Wind Turbine](#design-specifications-of-the-wind-turbine) and frames [Constructing Frames ](#constructing-frames) of the turbine are detailed in the sections linked. 

In the launch folder, the **windturbine_dynamic_frames_tf2.launch.py** file launches all the frames nodes and RViz2. This is achieved through a hierarchy of linked files:

  → **windturbine_dynamic_frames_tf2.launch.py** (contains dynamic frames executables)  
  → links to **windturbine_fixed_frame_tf2.launch.py** (contains static frames)  
  → links to **windturbine_base_tf2.launch.py** (contains the tower_base executable and the RViz executable)
    
Additionally, a separate node was created to include all frames in 1 file **(turbine_rotating2.cpp)**. For this, just launch **turbine_rotating_tf2.launch.py** and it shows all frames in RViz2.

------------------------------------------------------------------------

## 📦️📥️ Compiling and Sourcing 

1.  Compile the dynamic_tf2 package:

    ```
    colcon build --packages-select raaj_dynamic_tf2_cpp
    ```

2.  Source your workspace:

    ```
    source ~/.bashrc
    ```
    
------------------------------------------------------------------------

## 🚀 Launching the Nodes

### (A) Separate nodes for each frame

To launch the wind turbine's static and dynamic nodes:

    ros2 launch raaj_dynamic_tf2_cpp windturbine_dynamic_frames_tf2.launch.py
    
Expected output:
    
This will launch all the wind turbine frames and RViz2 to visualize 
them. You will be able to see a structure of a wind turbine with a 
rotating hub (which rotates the blades) and a yawing nacelle 
(which yaws the hub).
    
(Can use ros2 topic echo /tf and ros2 topic echo/tf_static for this since separate nodes, and static uses static_transform_broadcaster while dynamic uses transform_broadcaster)

    
### (B) All frames in 1 node

I also created a node with the same frames as (A), but in a single file.
The output is the same. Launch it using:

    ros2 launch raaj_dynamic_tf2_cpp turbine_rotating_tf2.launch.py 
    
[Only use ros2 topic echo /tf will show output since its all in 1 node
and uses transform_broadcaster]

ros2 run rqt_tf_tree rqt_tf_tree
    
   
    
    
    
------------------------------------------------------------------------

## 🌬️💨️ Design Specifications of the Wind Turbine


Inspiration for choosing the Horizontal-axis wind turbine is because for my masters
research, it involves design optimization for enhancing performance of an H-Darrieus 
wind turbine. So why not make frames for a wind turbine right? xD
Also, I aim to demonstrates simple dynamic frames that rotates around the x-axis (hub) 
and yaw about z-axis (nacelle). 

For the wind turbine measurement, I referred to the GW140-3.4MW turbine found in this
link: https://www.goldwindamericas.com/sites/default/files/GW%20140-3.4%20MW_EN.pdf
    
The length of the nacelle is not provided, so I just assumed distance of the hub from 
the center of the wind turbine tower.
   
The important measurements for this assignment is shown in the table below:
    
| Part | Dimension |
| :---: | :--- |
| Hub height | 100m |
| Rotor diameter | 140m |
| Distance from hub to center of tower | 4m |

------------------------------------------------------------------------

## 🖼️ Constructing Frames 

###  (i) Table of Frames Specification 

| Parent - Child | Type | Translation | Rotation |
| :---: | :--- | :--- | :--- |
| world - tower_base | Static | (0, 0, 0) | (0,0,0) |
| tower_base - tower_mid | Static | (0, 0, 2.5) | (0,0,0) |
| tower_mid - nacelle | Dynamic | (0, 0, 2.5) | (0.0, 0.0, yaw) |
| nacelle - hub | Dynamic | (0.2, 0, 0) | (hub_angle, 0.0, 0.0) |
| hub - blades | :--- | Static | (0.5, 0.0, radius) | (0,0,0) |
|              | :--- |        | (0.5,  radius*sqrt(3)/2.0, -radius/2.0) | (0,0,0) |
|              | :--- |        | (0.5, -radius*sqrt(3)/2.0, -radius/2.0) | (0,0,0) |
    
###  (ii) Equations for the yaw, hub_angle, and radius

Nacelle Yaw: yaw = 0.5 * sin(t/5.0); (Yaw about z-axis)
    
Hub Rotation: hub_angle = 2.0 * t; (radians/sec) (Rotate around x-axis)

Blades Placement: radius = 3.5; (3 blades, 120 degrees apart)
    
------------------------------------------------------------------------

### 🔮 Next Steps (to try personally)

-   Make complex frames with complex movements
