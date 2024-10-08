from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import xacro
import os
import random

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('num', default_value='5', description='Number of diff drive robots'))
    ld.add_action(DeclareLaunchArgument('x_min', default_value='5', description='Minimum x value for spawn position'))
    ld.add_action(DeclareLaunchArgument('x_max', default_value='20', description='Maximum x value for spawn position'))
    ld.add_action(DeclareLaunchArgument('y_min', default_value='-15', description='Minimum y value for spawn position'))
    ld.add_action(DeclareLaunchArgument('y_max', default_value='0', description='Maximum y value for spawn position'))
    ld.add_action(DeclareLaunchArgument('z', default_value='0.3', description='Spawn height'))

    def run_node_action(context: LaunchContext):
        actions = []
        package_name = "diff_drive_formation"
        pkg = FindPackageShare(package_name).find(package_name)
        xacro_file = os.path.join(pkg, 'urdf', 'diff_drive_robot.xacro')

        num = int(context.launch_configurations['num'])
        x_min = int(context.launch_configurations['x_min'])
        x_max = int(context.launch_configurations['x_max'])
        y_min = int(context.launch_configurations['y_min'])
        y_max = int(context.launch_configurations['y_max'])
        z = float(context.launch_configurations['z'])
        x_range = x_max - x_min
        y_range = y_max - y_min
        rand = random.sample(range(x_range * y_range // 4), num)
        x = [(num // (y_range // 2)) * 2 + x_min for num in rand]
        y = [(num % (y_range // 2)) * 2 + y_min for num in rand]
        spawn_position = [[x[i], y[i], z] for i in range(num)]

        for i in range(num):
            doc = xacro.process_file(xacro_file, mappings={'namespace':'ddrobot_'+str(i)})
            robot_state_publisher = Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                output='screen',
                namespace='ddrobot_'+str(i),
                parameters=[{'use_sim_time': True, 'robot_description': doc.toxml()}]
            )
            actions.append(robot_state_publisher)

            random_yaw = random.uniform(0, 2 * 3.14159265359)
            spawn_entity = Node(
                package='gazebo_ros', executable='spawn_entity.py',
                arguments=['-entity', 'ddrobot_' + str(i),
                        '-topic', 'ddrobot_' + str(i) + '/robot_description',
                        '-x', str(spawn_position[i][0]),
                        '-y', str(spawn_position[i][1]),
                        '-z', str(spawn_position[i][2]),
                        '-Y', str(random_yaw)],
                output='screen',
                )
            actions.append(spawn_entity)
        return actions

    ld.add_action(OpaqueFunction(function=run_node_action))
    return ld