import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command  # <-- BU SATIRI EKLE

def generate_launch_description():

    # Paket yollarını bul
    pkg_my_robot_description = get_package_share_directory('my_robot_description')
    pkg_my_ball_chaser = get_package_share_directory('my_ball_chaser')
    
    # XACRO dosyanın tam yolunu bul (PAKET ve DOSYA ADINI KENDİNE GÖRE DÜZELT)
    xacro_file_path = os.path.join(
        pkg_my_robot_description,
        'urdf',  # Xacro dosyanın 'urdf' klasöründe olduğunu varsayıyorum
        'my_robot.urdf.xacro'  # Xacro dosyanın adı (seninkiyle değiştir)
    )

    # Xacro dosyasını işle ve URDF içeriğini bir değişkene al
    robot_description_content = Command(['xacro ', xacro_file_path])

    # 1. Gazebo'yu Başlat (Bu dosya robot_state_publisher'ı BAŞLATMAMALI)
    start_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot_description, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. YENİ: Robot State Publisher'ı Başlat
    # Xacro'dan işlenen içeriği 'robot_description' parametresi olarak ver
    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3. YENİ: Joint State Publisher'ı Başlat
    # (Gazebo diff_drive eklentisi 'joint_states' yayınlasa da, 
    #  bazen joint_state_publisher'a da ihtiyaç duyulur. 
    #  Eğer Gazebo zaten /joint_states yayınlıyorsa bu düğüme gerek kalmayabilir,
    #  ancak eklemenin zararı olmaz ve ödevde isteniyor )
    start_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
        # Eğer GUI'li olanı kurduysan:
        # package='joint_state_publisher_gui',
        # executable='joint_state_publisher_gui',
    )

    # 4. Ball Chaser Düğümünü Başlat
    start_ball_chaser_node = Node(
        package='my_ball_chaser',
        executable='ball_chaser_node',
        output='screen'
    )

    # 5. RViz'i Başlat
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 6. 'ros2 run tf2_tools view_frames' komutunu çalıştır
    run_view_frames_command = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_tools', 'view_frames', '--ros-args', '-r', '__ns:=/tf_frames'],
        output='screen'
    )

    # Tüm düğümleri ve launch dosyalarını döndür
    return LaunchDescription([
        start_gazebo_launch,
        start_robot_state_publisher_node,  # <-- YENİ
        start_joint_state_publisher_node,  # <-- YENİ
        start_ball_chaser_node,
        start_rviz_node,
        run_view_frames_command
    ])
