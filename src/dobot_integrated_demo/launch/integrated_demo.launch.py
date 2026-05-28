#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("dobot_integrated_demo")
    default_config = os.path.join(pkg_share, "config", "integrated_demo.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "integrated_demo.rviz")

    config = LaunchConfiguration("config_file")
    grpc_addr = LaunchConfiguration("grpc_addr")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="综合 Demo YAML 配置文件路径。",
            ),
            DeclareLaunchArgument(
                "grpc_addr",
                default_value="192.168.5.2:50051",
                description="机器人 gRPC 地址。",
            ),
            DeclareLaunchArgument("enable_voice", default_value="true", description="是否启动语音模块。"),
            DeclareLaunchArgument("enable_safety", default_value="true", description="是否启动避障安全模块。"),
            DeclareLaunchArgument("enable_balance", default_value="true", description="是否启动姿态平衡模块。"),
            DeclareLaunchArgument("enable_demo_monitor", default_value="true", description="是否在控制台打印综合教学面板。"),
            DeclareLaunchArgument("feedback_audio_test_key", default_value="", description="play one feedback wav on startup, e.g. greeting"),
            DeclareLaunchArgument("rviz", default_value="true", description="是否启动 RViz2。"),
            DeclareLaunchArgument("record_plot", default_value="true", description="是否记录姿态曲线。"),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.core.motion_gateway_node",
                    "--config",
                    config,
                    "--grpc-addr",
                    grpc_addr,
                    "--safety-enabled",
                    LaunchConfiguration("enable_safety"),
                    "--feedback-audio-test-key",
                    LaunchConfiguration("feedback_audio_test_key"),
                ],
                name="motion_gateway_node",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.core.integrated_state_manager_node",
                    "--config",
                    config,
                    "--enable-voice",
                    LaunchConfiguration("enable_voice"),
                    "--enable-safety",
                    LaunchConfiguration("enable_safety"),
                    "--enable-balance",
                    LaunchConfiguration("enable_balance"),
                ],
                name="integrated_state_manager_node",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.perception.depth_bridge_node",
                    "--config",
                    config,
                ],
                name="depth_bridge_node",
                output="log",
                condition=IfCondition(LaunchConfiguration("enable_safety")),
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.core.safety_arbitrator_node",
                    "--config",
                    config,
                ],
                name="safety_arbitrator_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_safety")),
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.balance.imu_reader_node",
                    "--config",
                    config,
                ],
                name="imu_reader_node",
                output="log",
                condition=IfCondition(LaunchConfiguration("enable_balance")),
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.balance.balance_compensator_node",
                    "--config",
                    config,
                ],
                name="balance_compensator_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_balance")),
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.balance.demo_monitor_node",
                    "--config",
                    config,
                ],
                name="demo_monitor_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_demo_monitor")),
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.voice.voice_control_node",
                    "--config",
                    config,
                ],
                name="voice_control_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_voice")),
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_integrated_demo.balance.attitude_plot_recorder_node",
                    "--config",
                    config,
                ],
                name="attitude_plot_recorder_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("record_plot")),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "safety_guard_base",
                    "--child-frame-id",
                    "front_depth_camera",
                ],
                name="base_to_front_static_tf",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_safety")),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "3.1415926",
                    "--frame-id",
                    "safety_guard_base",
                    "--child-frame-id",
                    "back_depth_camera",
                ],
                name="base_to_back_static_tf",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_safety")),
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", default_rviz],
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
