from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

#### TO EXECUTE ####
# In terminal run below to launch
# ros2 launch ~/racecar_ws/final_challenge2026/launch/lane_detector.launch.py
# For Visualization
# ros2 run rqt_image_view rqt_image_view and go to /lane_debug


def generate_launch_description():
    bag_path = "/home/racecar/racecar_ws/final_challenge2026/bag_files/johnson_track_rosbag/rosbag2_2025_04_09-22_01_22"

    return LaunchDescription([
        # --- Use simulated time ---
        SetParameter(name="use_sim_time", value=True),

        # --- Lane detector node ---
        Node(
            package="final_challenge",
            executable="lane_detector",
            name="lane_detector",
            output="screen",
            parameters=[{
                "lookahead_ratio": 0.55,
                "smoothing_alpha": 0.2,
                "use_sim_time": True
            }],
            remappings=[
                ("/image_raw", "/zed/zed_node/rgb/image_rect_color")
            ]
        ),

        Node(
            package="final_challenge",
            executable="lane_evaluator",
            name="evaluator",
            output="screen"
        ),

        # --- Rosbag playback ---
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play",
                bag_path,
                "--loop",
                "--clock"
            ],
            output="screen"
        ),

        # ExecuteProcess(
        #     cmd=[
        #         "ros2", "topic", "echo", "--csv", "/lane_error", ">", "lane_eval_results.csv"
        #     ],
        #     shell=True
        # ),
    ])