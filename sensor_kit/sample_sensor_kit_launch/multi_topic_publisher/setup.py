from setuptools import setup

package_name = 'multi_topic_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加 launch 文件的安装规则
        ('share/' + package_name + '/launch', ['launch/multi_topic_publisher_launch.xml']),
        ('share/' + package_name + '/launch', ['launch/convert_laser_to_cloud.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # 替换为您的名字
    maintainer_email='your_email@example.com',  # 替换为您的邮箱
    description='A ROS2 package to publish multiple topics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_topic_publisher_node = multi_topic_publisher.multi_topic_publisher_node:main',
            'laser_to_cloud_converter_node = multi_topic_publisher.laser_to_cloud_converter_node:main',
        ],
    },
)