from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument,RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
import yaml

# The functions we need to describe the robot model and parameters
def load_disc_robot(file_name):
    with open(file_name) as f:
        robot = yaml.safe_load(f)
    robot['urdf'] = disc_robot_urdf(robot)
    return robot

def load_map(file_name):
    with open(file_name) as f:
        robot = yaml.safe_load(f)
    return robot

# Get the URDF for the robot
def disc_robot_urdf(robot):
    radius = robot['body']['radius']
    height = robot['body']['height']

    return f"""<?xml version="1.0"?>
                  <robot name="disc">
                      <material name="light_blue">
                          <color rgba="0.5 0.5 1 1"/>
                      </material>
                      <material name="dark_blue">
                          <color rgba="0.1 0.1 1 1"/>
                      </material>
                      <material name="dark_red">
                          <color rgba="1 0.1 0.1 1"/>
                      </material>
                      <link name="base_link">
                          <visual>
                              <geometry>
                                  <cylinder length="{height}" radius="{radius}"/>
                              </geometry>
                              <material name="light_blue"/>
                          </visual>
                      </link>
                      <link name="heading_box">
                          <visual>
                              <geometry>
                                  <box size="{0.9*radius} {0.2*radius} {1.2*height}"/>
                              </geometry>
                              <material name="dark_blue"/>
                          </visual>
                      </link>
                      <link name="laser" />
                      <joint name="base_to_heading_box" type="fixed">
                          <parent link="base_link"/>
                          <child link="heading_box"/>
                          <origin xyz='{0.45*radius} 0.0 0.0'/>
                      </joint>
                      <joint name="base_to_laser" type="fixed">
                          <parent link="base_link"/>
                          <child link="laser"/>
                          <origin xyz="{0.5*radius} 0.0 0.0"/>
                      </joint>
                  </robot>
                  """


def generate_launch_description():
    
    # Use_sim_time in order to synchronicze the time for the robot state publisher
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time',    default_value='true',    description='Use simulation (Gazebo) clock if true')
    # Play the bag file process
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',arg_in_value],
        output='screen',
    )
    
    # Get the path for the robot file and get the description
    arg_model_value='/home/sindhu/ros_ws/project_4d/src/project4d/project4d/ideal.robot'
    robot_desc=load_disc_robot(arg_model_value)
    robot_desc=load_disc_robot(arg_model_value)
    arg_map='/home/sindhu/ros_ws/project_4d/src/project4d/project4d/snake.world'
    map_desc=load_map(arg_map)
    
    
    # The robot publisher node which would have the robot_description
    node_robot_state_publisher=Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,'robot_description': robot_desc['urdf']}], #Send the parameters to the robot_state_publisher
            arguments=[arg_model_value],
    )
    
    
    # The simulator node would broadcast tf msg
    node_simulator = Node(
        package='project4d',
        executable='do_it_1',
        name='Simulator_node',
        output='screen',
        parameters=[{
        # Send the parameters for the robot
        'radius': robot_desc['body']['radius'],
        'error_variance_left': robot_desc['wheels']['error_variance_left'],
        'error_variance_right': robot_desc['wheels']['error_variance_right'],
        'distance': robot_desc['wheels']['distance'],
        'error_update_rate': robot_desc['wheels']['error_update_rate'],
        'initial_pose':map_desc['initial_pose'],
        # Send the parameter for the robot
        'map': map_desc['map'],
        'resolution':map_desc['resolution'],
                'laser_rate': robot_desc['laser']['rate'],
        'laser_count': robot_desc['laser']['count'],
        'laser_angle_min': robot_desc['laser']['angle_min'],
        'laser_angle_max': robot_desc['laser']['angle_max'],
        'laser_range_min': robot_desc['laser']['range_min'],
        'laser_range_max': robot_desc['laser']['range_max'],
        'laser_error_variance': robot_desc['laser']['error_variance'],
        'laser_fail_probability': robot_desc['laser']['fail_probability']
        
        }
                    ],
    )
    
    # The velocity_translator_node
    node_velocity_translate_node = Node(
        package='project4d',
        executable='do_it_2',
        name='Velocity_translator_node',
        output='screen',
        parameters=[{
            # Send the parameter for the robot
            'distance': robot_desc['wheels']['distance'],
        }]
    )
    
    # The navigation_controller_node
    navigation_controller_node = Node(
        package='project4d',
        executable='do_it_3',
        name='Navigation_Controller_Node',
        output='screen',
        parameters=[{
        'resolution':map_desc['resolution'],
        'radius': robot_desc['body']['radius'],
        
              
        }],
    )
    
    

    # Execute an additional process (ros2 bag record) using ExecuteProcess
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a','-o', arg_out_value],
        output='screen',
    )
    # End the launch file when the bag play ends
    event_handler = OnProcessExit(target_action=bag_play, on_exit=[EmitEvent(event=Shutdown())])
    terminate_at_end = RegisterEventHandler(event_handler)
    
    ld = LaunchDescription([ arg_use_sim_time,node_robot_state_publisher,node_velocity_translate_node,node_simulator,navigation_to_goal_node,terminate_at_end])
    return  ld
