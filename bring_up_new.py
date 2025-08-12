# micro_ros_agent_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ส่วนนี้จะทำการเรียกใช้ Launch file ของ Lidar (rplidar)
    # ทำให้ Lidar node ทำงานไปพร้อมกับ node อื่นๆ
    rplidar = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rplidar_ros'), 'launch'),
         '/rplidar_c1_launch.py'])
      )
   
    return LaunchDescription([
        # 1. รัน Micro-ROS Agent
        # ทำหน้าที่เป็นสะพานเชื่อมการสื่อสารกับ ESP32
        # ต้องแก้ '--dev', '/dev/ttyUSB1' ให้ตรงกับพอร์ตของหุ่นยนต์
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent1',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyUSB1']
        ),       

        # 2. รัน Node ควบคุมมอเตอร์
        # ซึ่งมาจากสคริปต์ motor.py หรือ motor_mecanum.py
        Node(
            package='motor_control',            
            executable='motor_control_node',
            name='motor_control'
        ),
        
        # 3. รัน Node คำนวณ Odometry
        # ซึ่งมาจากโค้ด odom.cpp หรือ odom_mecanum.cpp
        Node(
            package='robot_odom',            
            executable='odom',
            name='odometry'
        ),        
        
        # 4. เรียกใช้ Lidar Launch file ที่ประกาศไว้ด้านบน
        rplidar
    ])
