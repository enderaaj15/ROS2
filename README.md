# raaj_launch_assignments
# raajaction_musicplayer_py

The structure of the packages for this assignment are shown below:

```
├── raaj_launch_assignments/
│   ├── launch/
│   │   └── action_mainmusicplayer.launch.py
│   ├── src/
│   ├── include/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── LICENSE
│   └── README.md
└── raajaction_musicplayer_py/
    ├── raajaction_musicplayer_py/
    │   ├── music_player_server.py
    │   └── music_player_client.py
    ├── launch/
    │	└──action_musicplayer.launch.py
    ├── resource/
    ├── test/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── LICENSE
    └── README.md
```
## 🎯️ Function of the Parameter Monitoring Node
This assigment will demonstrate a ROS 2 launch application. Specifically, 
a previous assignment will be launch:
**(Assignment 4, package name:raajaction_musicplayer_py)**

A launch package, **raaj_launch_assignments** is created and in it, a launch
folder containing the main launch file, **action_mainmusicplayer.launch.py**.
Launching this file will run the **music_player_server** node based on the
arguments set in the launch file. The content of this launch file redirects
itsway to the module launch file in the **raajaction_musicplayer_py launch folder**.

For the main launch file, **action_mainmusicplayer.launch.py** to work, the module launch 
file, **action_musicplayer.launch.py** was also created in the launch folder located in the 
**raajaction_musicplayer_py** package. Alternatively, the module file can also be launched
to run the **music_player_server** node. 

The output from launching both the main and module launch files will be similar
to Assignment 4, where client sends goals to the server to play a song. The skip
function/argument is also included, more details below (part????).

------------------------------------------------------------------------

## 📦️📥️ Compiling and Sourcing 

1.  Compile the launch and musicplayer packages:

    ```
    colcon build --packages-select raaj_launch_assignments raajaction_musicplayer_py
    ```

2.  Source your workspace:

    ```
    source ~/.bashrc
    ```
    
------------------------------------------------------------------------

## 🚀 Launching the Nodes
### (A) Launching Main Launch File

    Launch the server using main launch file:

    ```
    ros2 launch raaj_launch_assignments action_mainmusicplayer.launch.py
    ```
    
    This will play the song "Hey Jude - The Beatles" as it satisfies the 
    goal condition 'True' (to unlock client) and executes the send_goal 
    to the server. Also, it skips the "Bohemian Rhapsody" as the skip 
    default value is 'True' in the main launch file:
    
    | Arguments | Description | Default Value |
    | :---: | :--- | :--- |
    | song_name | Song - Artist | "Hey Jude - The Beatles" |
    | skip | Skip current song | True |
    | goal | Unlocks Client | True |
    
    
### (B) Launching Module Launch File  

    Launch the server using module launch file:

    ```
    ros2 launch raajaction_musicplayer_py action_musicplayer.launch.py
    ```
    
    This will play the song "Bohemian Rhapsody" as it 
    satisfies the goal condition 'True' (to unlock client) and 
    executes the send_goal to the server. Eventhough the song_name 
    default in this file is "Helena - My Chemical Romance", the skip
    value in this module launch file is 'False', so it does not skip 
    the "Bohemian Rhapsody" song which is the initial send_goal set in
    the music_server_client:
    
    | Arguments | Description | Default Value |
    | :---: | :--- | :--- |
    | song_name | Song - Artist | "Helena - My Chemical Romance" |
    | skip | Skip current song | False |
    | goal | Unlocks Client | True |
    
------------------------------------------------------------------------

## 🎸️🎵️ Setting the Arguments for Module Launch File


### ❌️ (i) skip

   To skip "Bohemian Rhapsody":

    ```
    ros2 launch raajaction_musicplayer_py action_musicplayer.launch.py skip:=True
    ```
   

### 🥅️ (ii) goal

   Try False goal:

    ```
    ros2 launch raajaction_musicplayer_py action_musicplayer.launch.py goal:=False
    ```
    

### 💿️🎵️ (iii) song_name

    To play any other song, send_goal + must skip:=True

    ```
    ros2 launch raajaction_musicplayer_py action_musicplayer.launch.py song_name:='"Blinding Lights - The Weeknd"' skip:=True
    ```

------------------------------------------------------------------------
### Table of Parameters/Arguments

    | Arguments | Description | Default Value |
    | :---: | :--- | :--- |
    | song_name | Song - Artist | "Helena - My Chemical Romance" |
    | skip | Skip current song | False |
    | goal | Unlocks Client | True |
    
------------------------------------------------------------------------

### 🔮 Next Steps (to try personally)

-   add plugins for lyrics for some songs? so that with certain song 
    request, it will show the actual lyrics.
