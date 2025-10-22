# raaj_bmwparameters_py
## 🎯️ Function of the Parameter Node
This package simulates a BMW car showroom where you can set parameters
such as budget, car model, color, loan years, bank, and downpayment.\
The node calculates the **monthly loan installment** based on your car
model, loan years, bank (with interest rates set in the code) and 
downpayment selections. The car color is just an additional parameter 
to list available car colors. The budget will give you list of car
models within your budget.

-   **Package name:** `raaj_bmwparameters_py`  
-   **Node name:** `buycar_param_node`  
-   **Executable:** `bmw_showroom`  
-   **Source code:** `buybmw_node.py`  
-   **Launch file:** `bmw_parameters_launch.py`  
-   **Config file:** `bmwparameters.yaml`  

Parameters:
`budget`, `car_model`, `car_color`, `loan_years`, `bank`, `downpayment`

------------------------------------------------------------------------

## 🚀 Running the Code

1.  Compile the package:

    ```
    colcon build --packages-select raaj_bmwparameters_py
    ```

2.  Source your workspace:

    ```
    source ~/.bashrc
    ```

3.  Run the node:

    ```
    ros2 run raaj_bmwparameters_py bmw_showroom
    ```

    It will display output based on the **initial parameters** set in
    the code.

------------------------------------------------------------------------

## ⚙️ Changing Parameters

### (i) From the Console

While `buycar_param_node` is running in **Terminal 1**, open a new
**Terminal (2)** and set parameters:

1.  Check available parameters:

    ```
    ros2 param list /buycar_param_node
    ```

2.  Check current parameters:

    ```
    ros2 param get /buycar_param_node <parameter_name>
    ```
    
3.  Set parameters value (with example):

    ```
    ros2 param set /buycar_param_node budget 300000
    ros2 param set /buycar_param_node car_model "BMW 320i Sport"
    ros2 param set /buycar_param_node car_color blue
    ros2 param set /buycar_param_node loan_years 6
    ros2 param set /buycar_param_node bank maybank
    ros2 param set /buycar_param_node downpayment 40000
    ```

------------------------------------------------------------------------

### (ii) Using **rqt_reconfigure**

While `buycar_param_node` is running in **Terminal 1**, open a new
**Terminal (2)** and run rqt:

1.  Run the parameter reconfigure GUI:

    ```
    ros2 run rqt_reconfigure rqt_reconfigure
    ```
    
2.  In rqt:
   
    Select `buycar_param_node`, change parameters, and hit **Enter** each
    time.

------------------------------------------------------------------------

### (iii) Using a YAML File

While `buycar_param_node` is running in **Terminal 1**, open a new
**Terminal (2)** and:

1. Edit `bmwparameters.yaml` in the config folder, then load it in 
   **Terminal 2** using the command:

    ```
    ros2 param load /buycar_param_node ~/`ros2_ws_assignment1`/src/raaj_bmwparameters_py/config/bmwparameters.yaml
    ```
    
    **note** `ros2_ws_assignment1` > change to your workspace directory

2. To save the parameters, use command:

    ```
    ros2 param dump /buycar_param_node > ~/`ros2_ws_assignment1`/src/raaj_bmwparameters_py/config/bmwparameters.yaml
    ```
    
    **note** `ros2_ws_assignment1` > change to your workspace directory

------------------------------------------------------------------------

## 📦 Launching with a Launch File

Simply run in **Terminal 1**:

```
ros2 launch raaj_bmwparameters_py bmw_parameters_launch.py
```

Then in **Terminal 2**, repeat the changing parameters steps of
(i), (ii), and (iii) as shown above. **Note** that node name now is
`custom_buycar_param_node`

------------------------------------------------------------------------

## 📝 Notes (Post Presentation)

-   Currently, the budget range is `120,000 – 600,000`. But logically, a
    **higher budget** should not be an issue. Only a **minimum limit** of
    RM120,000 is necessary.

Before (with upper and lower budget limits):

```
    if not (120_000 <= value <= 600_000):
	self.get_logger().error(
	    f'Budget RM{value:,} is out of range (RM120,000 - RM600,000). Rejecting parameter.'
	)
```

Can update to:

```
    if value < 120_000:
	self.get_logger().error(
	    f'Budget RM{value:,} is below the minimum (RM120,000). Rejecting parameter.'
	) 
```

### 🔮 Next Steps (to try personally)

-   Instead of always publishing the last valid parameters into 
    calculating the monthly repayment, consider
    **rejecting invalid inputs immediately** with a hard stop.
