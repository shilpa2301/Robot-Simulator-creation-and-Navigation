# The load_disc_robot method reads a file that describes a disc-shaped robot
# and returns a dictionary describing the properties of that robot.

import yaml

def load_disc_robot(file_name):
    with open(file_name) as f:
        robot = yaml.safe_load(f)
    robot['urdf'] = disc_robot_urdf(robot)
    return robot

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
