from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raaj_bmwparameters_py',
            executable='bmw_showroom',
            name='custom_buycar_param_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'downpayment': 50000,
                 'budget': 400000,
                 'car_model': 'BMW X4',
                 'car_color': 'black',
                 'loan_years': 7,
                 'bank': 'cimb'}
            ]
        )
    ])
