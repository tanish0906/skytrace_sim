import copy
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def prefix_urdf_links(urdf_tree: ET.ElementTree, prefix: str) -> str:
    root = copy.deepcopy(urdf_tree.getroot())
    for link in root.findall('link'):
        link.attrib['name'] = f"{prefix}{link.attrib['name']}"
    for joint in root.findall('joint'):
        joint.attrib['name'] = f"{prefix}{joint.attrib['name']}"
        joint.attrib['type'] = 'fixed'
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and 'link' in parent.attrib:
            parent.attrib['link'] = f"{prefix}{parent.attrib['link']}"
        if child is not None and 'link' in child.attrib:
            child.attrib['link'] = f"{prefix}{child.attrib['link']}"
    return ET.tostring(root, encoding='unicode')


def generate_launch_description():
    pkg_share = get_package_share_directory('skytrace_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'aircraft.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'skytrace_urdf.rviz')
    urdf_tree = ET.parse(urdf_path)

    nodes = [
        Node(
            package='skytrace_sim',
            executable='skytrace_urdf_node',
            name='skytrace_urdf_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ]

    for i in range(9):
        prefixed_urdf = prefix_urdf_links(urdf_tree, f'aircraft_{i}_')
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=f'aircraft_{i}',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': prefixed_urdf}],
            )
        )

    return LaunchDescription(nodes)
