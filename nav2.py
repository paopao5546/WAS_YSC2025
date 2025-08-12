# nav2.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- ส่วนที่ 1: ระบุตำแหน่งไฟล์และ Package ที่จำเป็น ---

    # หาตำแหน่งของ Package 'robot_bringup' ที่เราสร้างไว้
    # !! แก้ไข 'robot_bringup' หากคุณใช้ชื่ออื่น
    pkg_bringup = get_package_share_directory('robot_bringup')

    # ระบุตำแหน่งของ Launch file ที่ใช้เปิดระบบพื้นฐานของหุ่นยนต์
    # นี่คือไฟล์ bring_up_new.py หรือที่เปลี่ยนชื่อเป็น bring_up_robot.launch.py
    robot_bringup_launch_path = os.path.join(pkg_bringup, 'launch', 'bring_up_robot.launch.py')

    # <<< จุดสำคัญที่ 1: ระบุไฟล์แผนที่ >>>
    # ระบุว่าแผนที่ชื่อ my_map.yaml อยู่ในโฟลเดอร์ maps ของ package เรา
    # คุณต้องสร้างไฟล์ my_map.yaml และ my_map.pgm จากการทำ SLAM ก่อน
    map_file_path = os.path.join(pkg_bringup, 'maps', 'my_map.yaml')

    # <<< จุดสำคัญที่ 2: ระบุไฟล์พารามิเตอร์ >>>
    # ระบุว่าไฟล์พารามิเตอร์ชื่อ burger.yaml อยู่ในโฟลเดอร์ params
    # ไฟล์นี้คือไฟล์ที่กำหนดค่าการทำงานทั้งหมดของ Nav2
    nav2_params_path = os.path.join(pkg_bringup, 'params', 'burger.yaml')


    # --- ส่วนที่ 2: กำหนดค่าเริ่มต้นสำหรับ Launch Arguments ---

    # กำหนดให้สามารถส่งค่าเหล่านี้มาจากภายนอกได้ แต่ถ้าไม่ส่งมา ก็จะใช้ค่า default ที่เราตั้งไว้
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration('map', default=map_file_path)
    param_dir = LaunchConfiguration('params_file', default=nav2_params_path)


    # --- ส่วนที่ 3: กำหนดสิ่งที่จะรัน ---

    # 1. เรียกใช้ Launch file พื้นฐานของหุ่นยนต์ (Lidar, Motor, Odom)
    # เปรียบเสมือนการ "สตาร์ทเครื่องยนต์" ของหุ่นยนต์
    start_robot_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup_launch_path)
    )

    # 2. เรียกใช้ Launch file หลักของ Nav2
    # นี่คือการ "เปิดระบบ GPS และระบบนำทางอัตโนมัติ"
    start_nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py']),
        
        # <<< จุดสำคัญที่ 3: ส่งค่าแผนที่และพารามิเตอร์เข้าไปให้ Nav2 >>>
        # บอก Nav2 ว่า "ให้ใช้แผนที่นี้ และให้ตั้งค่าตามไฟล์นี้"
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
    )


    # --- ส่วนที่ 4: สร้าง LaunchDescription เพื่อรวบรวมทุกอย่าง ---
    # สรุปรายการทั้งหมดที่ต้องการให้ ROS2 รัน
    
    ld = LaunchDescription()

    # เพิ่ม Launch Arguments ที่ประกาศไว้
    ld.add_action(DeclareLaunchArgument('map', default_value=map_dir))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=param_dir))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

    # เพิ่มคำสั่งให้รันระบบพื้นฐานของหุ่นยนต์
    ld.add_action(start_robot_bringup_cmd)
    
    # เพิ่มคำสั่งให้รันระบบ Nav2
    ld.add_action(start_nav2_bringup_cmd)

    return ld
