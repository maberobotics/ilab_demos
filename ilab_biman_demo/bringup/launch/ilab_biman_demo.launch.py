# Copyright 2023 MABE-ROBOTICS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node



def generate_launch_description():

     # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ilab_biman_demo'), 'description/urdf', 'ilab_biman.urdf.xacro']
            ),
            " use_fake_hardware:=true",
            " use_sim:=false",
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
      [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare('ilab_biman_demo'), 'description/srdf', 'ilab_biman.srdf.xacro']
        ),
      ]
    )

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Get planning parameters
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare('ilab_biman_demo'), "description/moveit2", "joint_limits.yaml",
        ]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution([
            FindPackageShare('ilab_biman_demo'), "description/moveit2", "pilz_cartesian_limits.yaml",
        ]
    )

    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService \
            move_group/ExecuteTaskSolutionCapability"""
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare('ilab_biman_demo'), "description/moveit2", "kinematics.yaml"]
    )

    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare('ilab_biman_demo'), "description/moveit2", "planning_pipelines_config.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution([
            FindPackageShare('ilab_biman_demo'), "description/moveit2", "ompl_planning.yaml",
        ]
    )

    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare('ilab_biman_demo'),
            "description/moveit2", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            planning_pipelines_config,
            ompl_planning_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": False},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ilab_biman_demo'), "bringup/config", "ilab_biman_demo.rviz"]
    )

    robot_controllers = PathJoinSubstitution(
      [FindPackageShare('ilab_biman_demo'), 'bringup/config', 'controllers.yaml']
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['/robots/joint_states', '/franka_gripper/joint_states'], 'rate': 30}],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning_cartesian_limits,
            planning_pipelines_config,
            ompl_planning_config,
            robot_description_kinematics,
            moveit_controllers,
        ],
    )

    # gripper_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [FindPackageShare('ilab_franka_bringup'), 'launch', 'ilab_franka_gripper.launch.py'])]),
    #     launch_arguments={'robot_ip': '0.0.0.0',
    #                       'use_fake_hardware': 'true',
    #                       'arm_id': 'franka'}.items(),
    # )

    control_node = Node(
      package='controller_manager',
      executable='ros2_control_node',
      parameters=[robot_description, robot_controllers],
      output='both',
      remappings=[('/joint_states', '/robots/joint_states')],
    )

    ur_robot_controller_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['ur_scaled_joint_trajectory_controller']
    )

    joint_state_broadcaster_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['joint_state_broadcaster']
    )

    ur_gripper_controller_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['ur_gripper_controller']
    )

    franka_robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_arm_controller']
    )

    franka_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_hand_controller']
    )

    nodes_to_start = [
        control_node,
        ur_robot_controller_spawner,
        joint_state_broadcaster_spawner,
        ur_gripper_controller_spawner,
        franka_robot_controller_spawner,
        franka_gripper_controller_spawner,
        joint_state_publisher,
        robot_state_publisher_node,
        rviz_node,
        # gripper_launch_file,
        move_group_node
    ]

    return LaunchDescription(nodes_to_start)