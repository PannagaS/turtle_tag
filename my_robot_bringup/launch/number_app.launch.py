from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    robot_news_node_0 = Node(package = 'my_py_pkg', executable = 'robot_news_node', name = 'Gatech', parameters =[{"uni_name": "Gatech !"}])
    robot_news_node_1 = Node(package = 'my_py_pkg', executable = 'robot_news_node', name = 'UMich', parameters =[{"uni_name": "UMich !"}])
    robot_news_node_2 = Node(package = 'my_py_pkg', executable = 'robot_news_node', name = 'UCSD', parameters =[{"uni_name": "UCSD !"}])
    robot_news_node_3 = Node(package = 'my_py_pkg', executable = 'robot_news_node', name = 'UMD', parameters =[{"uni_name": "UMD !"}])
    robot_news_node_4 = Node(package = 'my_py_pkg', executable = 'robot_news_node', name = 'UMN', parameters =[{"uni_name": "UMN !"}])
    robot_news_node_5 = Node(package = 'my_py_pkg', executable = 'robot_news_node', name = 'ASU', parameters =[{"uni_name": "ASU !"}])
    car_node = Node(package = 'my_py_pkg', executable = 'car')
    ld.add_action(robot_news_node_0)
    ld.add_action(robot_news_node_1)
    ld.add_action(robot_news_node_2)
    ld.add_action(robot_news_node_3)
    ld.add_action(robot_news_node_4)
    ld.add_action(robot_news_node_5)
    ld.add_action(car_node)


    return ld