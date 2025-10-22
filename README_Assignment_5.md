# comp_raaj_bmwshowroom_srvcli

## 🎯️ Function of the Composable Nodes
This is an extension from my assignment 2 (raaj_bmwparameters_py)
where I rewrote the code in C++ and convert it into composable node 
(**buycar_server**).
Also added another composable node (**buycar_client**) to interact 
with the buycar_server 

The **buycar_server** calculates and publishes monthly loan 
installment based on car model, loan years, 
bank (with interest rates set in the code) and downpayment which are
parameters that can be set.

The **buycar_client** is subscribed to the buycar_server, so when
buycar_server calculates and publishes the monthly repayment,
buycar_client will hear it.

The car color is just an additional parameter to list available car 
colors for fun. 
Setting budget will give you list of car models available within your 
budget.

```
└── comp_raaj_bmwshowroom_srvcli/
    ├── src/
    │   ├── bmw_server.cpp
    │   └── bmw_client.cpp
    ├── include/
    ├── config/
    │   └── comp_bmw.yaml
    ├── CMakeLists.txt
    ├── package.xml
    ├── LICENSE
    └── README.md
```

-   **Package name:** `comp_raaj_bmwshowroom_srvcli`\
-   **Node names:** `buycar_server`\
                    `buycar_client`
-   **Component names:** `comp_raaj_bmwshowroom_srvcli::BuyCarServer`\
                         `comp_raaj_bmwshowroom_srvcli::BuyCarClient`
-   **Executables:** `car_server`\
                     `car_client`
-   **Source codes:** `buybmw_server.cpp`\
                      `buybmw_client.cpp`
-   **Config file:** `comp_bmw.yaml`

Parameters:\
`budget`, `car_model`, `car_color`, `loan_years`, `bank`, `downpayment`

------------------------------------------------------------------------

## 🚀 Running the Code

1.  Compile the package:

    ``` bash
    colcon build --packages-select comp_raaj_bmwshowroom_srvcli
    ```

2.  Source your workspace:

    ``` bash
    source ~/.bashrc
    ```

3.  Run the node:
 
    **Standalone (EXECUTABLE)**
    
        ***Terminal 1:***
    ``` bash
    ros2 run comp_raaj_bmwshowroom_srvcli car_server
    ```
        ***Terminal 2:***
    ``` bash
    ros2 run comp_raaj_bmwshowroom_srvcli car_client
    ```
    
    **Composable (PLUGIN)**
    
        ***Terminal 1:***
        
    ``` bash
    ros2 run rclcpp_components component_container
    ```
        ***Terminal 2:***
        
    ``` bash
    ros2 component load /ComponentManager comp_raaj_bmwshowroom_srvcli comp_raaj_bmwshowroom_srvcli::BuyCarServer
    ```
    wait for this component to load, then

    ``` bash
    ros2 component load /ComponentManager comp_raaj_bmwshowroom_srvcli comp_raaj_bmwshowroom_srvcli::BuyCarClient
    ```
    
    🔊️buycar_server shows the monthly repayment based on initial parameters 
      and buycar_client listen to it.
    
    
    It will display output based on the **initial parameters** set in
    the code. Can change parameter, refer to sections below.

------------------------------------------------------------------------

## ⚙️ Changing Parameters (For Composable)

### (i) From the Terminal

In the same Terminal we loaded the components or better yet, new Terminal:
So in **Terminal (2)** or **Terminal 3** set parameters:
  
1.  Set parameters value (with example):

    ``` bash
    ros2 param set /buycar_server budget 300000
    ros2 param set /buycar_server car_model "BMW 320i Sport"
    ros2 param set /buycar_server car_color blue
    ros2 param set /buycar_server loan_years 6
    ros2 param set /buycar_server bank maybank
    ros2 param set /buycar_server downpayment 40000
    ```
2.  Can also load the Server component with the parameters straight away
    (with example):

    ``` bash
    ros2 component load /ComponentManager comp_raaj_bmwshowroom_srvcli comp_raaj_bmwshowroom_srvcli::BuyCarServer -p loan_years:=9 -p bank:=maybank -p car_color:=black -p downpayment:=45000 -p budget:=450000 -p car_model:="BMW 2 Series Gran Coupe"
    ```
    
3. Try setting downpayment > car price (e.g. downpayment 700000 car_model "BMW X2")
   Try setting budget < car price (e.g. budget 130000 car_model "BMW X6")
   
   and see the output 😁️
------------------------------------------------------------------------

### (iii) Using a YAML File

1. Load it using the command:

    ``` bash
    ros2 param load /buycar_server ~/ros2_ws_assignment1/src/comp_raaj_bmwshowroom_srvcli/config/comp_bmw.yaml
    ```
    
    **note** `ros2_ws_assignment1` > change to your workspace directory

------------------------------------------------------------------------

### 🔮 Next Steps (to try personally)

-   Wanted to make it server-client where client input parameters, but
    had to change last minute. So will try again. Also will try plugins
