# raajbmw_parammonitor_cpp
# raaj_bmwparameters_cpp

Structure of the packages for this assignment:

```
├── raajbmw_parammonitor_cpp/
│   ├── src/
│   │   └── bmwparameters_event_handler.cpp
│   ├── include/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── LICENSE
│   └── README.md
└── raaj_bmwparameters_cpp/
    ├── src/
    │   └── bmw_parameters.cpp
    ├── config/
    │   └── comp_bmw.yaml
    ├── include/
    ├── CMakeLists.txt
    ├── package.xml
    └── LICENSE
```

## 🎯️ Function of the Parameter Monitoring Node
This is an extension from my assignment 2 (raaj_bmwparameters_py)
where I rewrote the code in C++ and made some adjustments to it.

The node created for this assignment, **bmw_parameters_monitoring**
is used to monitor any parameter changes to the **buycar_param_node**

**bmw_parameters_monitoring**
Whenever, parameters are set succesfully, this node will receive updates
and log it to show which parameter is changed and to what value. It will 
also show the parameter type (integer/string)

-   **Package name:** `raajbmw_parammonitor_cpp`  
-   **Source code:** `bmwparameters_event_handler.cpp`  
-   **Node name:** `bmw_parameters_monitoring`  
-   **Executable:** `bmwparam_monitor`  
 
**buycar_param_node**
This node calculates and publishes monthly loan installment based on 
car model, loan years, bank (with interest rates set in the code) 
and downpayment which are parameters that can be set.

The car color is just an additional parameter to list available car 
colors for fun. Setting budget will give you list of car models 
that are available within your budget.

-   **Package name:** `raaj_bmwparameters_cpp`  
-   **Source code:** `bmw_parameters.cpp`  
-   **Node name:** `buycar_param_node`  
-   **Executable:** `buycar_param_node`  
-   **Config file:** `comp_bmw.yaml`  

Parameters:\
`budget`, `car_model`, `car_color`, `loan_years`, `bank`, `downpayment`

------------------------------------------------------------------------

## 🚀 Running the Code

1.  Compile the packages:

    ```
    colcon build --packages-select raajbmw_parammonitor_cpp
    colcon build --packages-select raaj_bmwparameters_cpp
    ```

2.  Source your workspace:

    ```
    source ~/.bashrc
    ```

3.  Running the nodes:
    
    ***Terminal 1:***
    ```
    ros2 run raaj_bmwparameters_cpp buycar_param_node
    ```
    
    🔊️`buycar_param_node` will show the monthly repayment based on initial 
    parameters set in the code.
       
    ***Terminal 2:***
    ```
    ros2 run raajbmw_parammonitor_cpp bmwparam_monitor
    ```
   
    🔊️`bmw_parameters_monitoring` will show this:
    Monitoring BMW parameters from buycar_param_node


    To see the `bmw_parameters_monitoring` log the changes when 
    parameters are set, set parameter by referring to the sections below.

------------------------------------------------------------------------

## ⚙️ Changing Parameters

### (i) From the Terminal

In **Terminal 3** set the parameters:
  
1.  Set parameters value (with example):

    ```
    ros2 param set /buycar_server budget 300000
    ros2 param set /buycar_server car_model "BMW 320i Sport"
    ros2 param set /buycar_server car_color blue
    ros2 param set /buycar_server loan_years 6
    ros2 param set /buycar_server bank maybank
    ros2 param set /buycar_server downpayment 40000
    ```

2.  Can also load parameters using .yaml file:

    ```
    ros2 param load /buycar_param_node ~/ros2_ws_assignment1/src/raaj_bmwparameters_cpp/config/comp_bmw.yaml
    ```
    
    Try setting downpayment > car price (e.g. downpayment 700000 car_model "BMW X2")
    Try setting budget < car price (e.g. budget 130000 car_model "BMW X6")
   
    and see the output 😁️
   
------------------------------------------------------------------------

### (iii) Monitoring Parameters

1. There are a total of 6 parameters when set successfully, the 
   `bmw_parameters_monitoring` will show an update.


    

------------------------------------------------------------------------

### 🔮 Next Steps (to try personally)

-   Monitor plugin changes when loading plugins
