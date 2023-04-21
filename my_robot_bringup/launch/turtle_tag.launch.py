from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
  
    turtlesim_node= Node(package = 'turtlesim',executable='turtlesim_node')
    turtlesim_spawnner_node = Node(package ='turtle_tag', executable = 'turtle_spawnner', parameters=[{"spawn_frequency":1.0}])
    turtlesim_controller_node = Node(package ='turtle_tag', executable = 'turtle_controller', parameters=[{"catch_closest_turtle_first":True}])

    
    ld.add_action(turtlesim_node)
    ld.add_action(turtlesim_spawnner_node)
    ld.add_action(turtlesim_controller_node)


    return ld