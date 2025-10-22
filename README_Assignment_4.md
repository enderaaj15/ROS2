# raajaction_musicplayer_py
# raaj_action_interfaces

These packages demonstrate a ROS 2 action system for music playback. 
A song request (song_name) can be sent to the server to start playing 
a track. If another song is already playing, the new request can either 
skip the current song or be rejected, depending on the (skip_current) 
value.

While the song is playing, feedback continuously reports the progress 
percentage (progress_percent). Once playback is complete, the result 
message (result_message) confirms that the song has finished.

PlaySong.action was created in raaj_action_interfaces. The structure 
of the packages for this assignment are shown below:
```
├── raajaction_musicplayer_py/
│   ├── raajaction_musicplayer_py/
│   │   ├── music_player_server.py
│   │   ├── music_player_client.py
│   │   └── __init__.py
│   ├── resource/
│   ├── test/
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   └── README.md
└── raaj_action_interfaces/
    ├── action/
    │   └── PlaySong.action
    ├── include/
    ├── src/
    ├── CMakeLists.txt
    └── package.xml
```

-   **Executable-server:** `music_server`\
-   **Executable-client:** `music_client`\
-   **PlaySong.action:**  *Goal:* `"song_name"`\
                	      **Goal:** `"skip_current"`\ 
			              **Result:** `"result_message"`\
			              **Feedback:** `"progress_percent"`\

------------------------------------------------------------------------

## 🚀 Running the Code

1.  Compile the package:

    ``` bash
    colcon build --packages-select raajaction_musicplayer_py raaj_action_interfaces
    ```

2.  Source your workspace:

    ``` bash
    source ~/.bashrc
    ```

3.  Run the server:

    ``` bash
    ros2 run raajaction_musicplayer_py music_server
    ```

    It will display "🎶 Music Player Action Server ready!"


------------------------------------------------------------------------

## 🎸️🎵️ Playing the songs
### (i) Using Client to play Bohemian Rhapsody

While `music_server` is running in **Terminal 1**, open a new
**Terminal (2)** and run music_client:

1.  Run the client:

    ``` bash
    ros2 run raajaction_musicplayer_py music_client
    ```

2.  Check action_name and action_interface:

    ``` bash
    ros2 action list -t
    ```

------------------------------------------------------------------------

### 🥅️ (ii) Using send_goal to play song

1. While the music_server is still running, open another Terminal and 
   use the command below to play any song:
   (**When no song is playing**) (**with feedback**)

    ``` bash
    ros2 action send_goal -f /play_song raaj_action_interfaces/action/PlaySong "{song_name: Shape of You - Ed Sheeran}"
    ```
   
2. However (**When there is a song playing**), this is where it gets 
   interesting.
   
       **(a) Send another goal "skip_current: false" to reject song change** 
        
While Terminal 1 is still running the music_server, lets play a song 
in Terminal 2:

    ``` bash
    ros2 action send_goal /play_song raaj_action_interfaces/action/PlaySong "{song_name: Despacito - Luis Fonsi}"
    ```
    
Then in Terminal 3, try requesting a song change by running this:
   
    ``` bash
    ros2 action send_goal /play_song raaj_action_interfaces/action/PlaySong "{song_name: Blinding Lights - The Weeknd, skip_current: false}"
    ```
    
It will reject the request and not play the song. To accept song change:
 
       **(b) Send "skip_current: true" to accept song change**
        
    ``` bash
    ros2 action send_goal /play_song raaj_action_interfaces/action/PlaySong "{song_name: I Had Some Help - Morgan Wallen, skip_current: true}"
    ```

This will Abort whatever song that was playing and accept to start 
playing this new song request.


------------------------------------------------------------------------

### ❌️ (iii) Stopping the song (cancel action)

1. Open another Terminal and use the command below to stop the song:

    ``` bash
    ros2 service call /play_song/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
    ```
    
------------------------------------------------------------------------

### 🔮 Next Steps (to try personally)

-   add plugins for lyrics for some songs? so that with certain song 
    request, it will show the lyrics along with the progress bar as 
    feedback
