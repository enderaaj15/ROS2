from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    song_name = LaunchConfiguration('song_name')
    skip = LaunchConfiguration('skip')
    goal = LaunchConfiguration('goal')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'song_name',
            default_value='"Helena - My Chemical Romance"',
            description='Song - Artist'
        ),
        
        DeclareLaunchArgument(
            'skip',
            default_value='False',
            description='Skip current song'
        ),
        
        DeclareLaunchArgument(
            'goal',
            default_value='True',
            description='Unlocks Client'
        ),
        
        Node(
            package='raajaction_musicplayer_py',
            executable='music_server',
            name='music_player_server',
            output='screen',
            emulate_tty=True,
        ),
        
        Node(
            condition=IfCondition(goal),
            package='raajaction_musicplayer_py',
            executable='music_client',
            name='music_player_client',
            output='screen',
            emulate_tty=True,
        ),
       
        ExecuteProcess(
            condition=IfCondition(PythonExpression([ goal, ' == True'])),
            
            cmd=[[
                ' ros2 action send_goal --feedback',
                ' /play_song',
                ' raaj_action_interfaces/action/PlaySong',
                ' "{song_name: "', song_name, '", skip_current: ', skip, '}"'
            ]],
            shell=True
        ),
    ])
