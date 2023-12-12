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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

     # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ilab_biman_demo'), 'description/urdf', 'ilab_biman.urdf.xacro']
            ),
            " use_fake_hardware:=false",
            " use_sim:=false",
        ]
    )

    ur_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ilab_biman_demo'), 'description/urdf', 'ilab_biman_ur.urdf.xacro']
            ),
            " use_fake_hardware:=false",
            " use_sim:=false",
        ]
    )

    franka_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ilab_biman_demo'), 'description/urdf', 'ilab_biman_franka.urdf.xacro']
            ),
            " use_fake_hardware:=false",
            " use_sim:=false",
        ]
    )

    robot_description = {'robot_description': robot_description_content}
    ur_robot_description = {'robot_description': ur_robot_description_content}
    franka_robot_description = {'robot_description': franka_robot_description_content}


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

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare('ilab_biman_demo'), "description/moveit2", "kinematics.yaml"]
    )

    pick_and_place_node = Node(
        package="ilab_biman_demo",
        executable="ilab_biman_demo_pick_and_place",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
        ],
    )


    nodes_to_start = [
        pick_and_place_node,
    ]

    return LaunchDescription(nodes_to_start)