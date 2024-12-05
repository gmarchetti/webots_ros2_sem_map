from setuptools import find_packages, setup

package_name = 'webots_ros2_sem_map'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/lidar_world.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/house.dae']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/map.dae']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Bark.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Base_BaseColor.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/book_cover.png']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Cupboard_Base_Color.png']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Diffuse.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Ground072_4K-JPG_Color.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/indoor_texture.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Marble06_col.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Oven_Diffuse.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/panel_light.jpg.001.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Pillow-Cover_BaseColor.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Stone_D.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/stove_diffuse.jpg.001.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Structure_Basecolor.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/terrazo_white.jpg.002.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Wood_Diffuse.jpg']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/Wood058_2K_Color.jpg']))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guilherme',
    maintainer_email='guilherme.marchetti@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_recognition = webots_ros2_sem_map.img_object_detection_node:main'
        ],
    },
)
