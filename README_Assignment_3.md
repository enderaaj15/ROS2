# raaj_tyregrip_base
# raaj_tyregrip_plugins

These packages will show the coefficient of grip/friction and calculate 
the simple traction force of a car based on the type of tyres, weather, 
and speed. This is done using plugins. The Weather and Tyre type are 
the 2 classes of plugins, with each having 3 plugins. The structure of 
the packages for this assignment are shown below:

```
├── raaj_tyregrip_base/
│   ├── src/
│   │   └── traction_node.cpp
│   ├── include/
│   │   └── raaj_tyregrip_base/
│   │       └── traction.hpp
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── config/
│       ├── uhp_snow.yaml
│       └── winter_snow.yaml
└── raaj_tyregrip_plugins/
    ├── include/
    │   └── raaj_tyregrip_plugins/
    │       ├── raaj_tyregrip_plugins.hpp
    │       └── visibility_control.h
    ├── src/
    │   └── raaj_tyregrip_plugins.cpp
    ├── CMakeLists.txt
    ├── plugins.xml
    └── package.xml
```

-   **Executable:** `traction_node`
-   **Weather Plugins:** `"raaj_tyregrip_plugins::DryWeather"`  
			 `"raaj_tyregrip_plugins::WetWeather"`  
			 `"raaj_tyregrip_plugins::SnowWeather"`  
-   **Tyre Plugins:** `"raaj_tyregrip_plugins::UHPTyre"`  
		      `"raaj_tyregrip_plugins::EcoTyre"`  
		      `"raaj_tyregrip_plugins::WinterTyre"`  

------------------------------------------------------------------------

## 🚀 Running the Code

1.  Compile the package:

    ```
    colcon build --packages-select raaj_tyregrip_base raaj_tyregrip_plugins
    ```

2.  Source your workspace:

    ```
    source ~/.bashrc
    ```

3.  Run the node:

    ```
    ros2 run raaj_tyregrip_base traction_node
    ```

    It will display output based on the **initial plugins & parameters** 
    set in the node, traction_node.cpp.

------------------------------------------------------------------------

## ⚙️ Changing Plugins in Real Time

### (i) From the Terminal

While `traction_node` is running in **Terminal 1**, open a new
**Terminal (2)** and set parameters:

1.  Check available parameters:

    ```
    ros2 param list /traction_node
    ```

2.  Check current parameters:

    ```
    ros2 param get /traction_node <parameter_name>
    ```
    
3.  Swap plugins using param set (with example)

    ```
    ros2 param set /traction_node tyre_plugin raaj_tyregrip_plugins::EcoTyre
    ros2 param set /traction_node weather_plugin raaj_tyregrip_plugins::WetWeather
    ```

------------------------------------------------------------------------

### (ii) Using a YAML File

1. Created yaml files, `uhp_snow.yaml` and `winter_snow.yaml` in a config 
   folder: **src/raaj_tyregrip_base/config**.
   
2. Load the file using: 

   ```
    ros2 run raaj_tyregrip_base traction_node --ros-args --params-file ~/ros2_ws_assignment1/src/raaj_tyregrip_base/config/uhp_snow.yaml
    ```
    
**Alternatively**

While `traction_node` is running in **Terminal 1**, open a new
**Terminal (2)** and:

1. Save current parameters using command:

For example:

    ```
    ros2 param dump /traction_node > ~/ros2_ws_assignment1/src/raaj_tyregrip_base/config/tyre_weather.yaml
    ```

2. Edit the .yaml file in the config folder, then reload in 
   **Terminal 2** using the command:

    ```
    ros2 param load /traction_node ~/ros2_ws_assignment1/src/raaj_tyregrip_base/config/tyre_weather.yaml
    ```

------------------------------------------------------------------------

### 🔮 Next Steps (to try personally)

-   add launch file
