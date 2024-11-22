from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luc',
    maintainer_email='luc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main", # el nombreejecutable = nombrepackage.nombrefilepy:funcion que quiero correr
            "test_nodo = my_robot_controller.segundo_nodo:main" ,
            "draw_circle = my_robot_controller.circulo:main",
            "subscriptor = my_robot_controller.subscriptor:main",
            "controlador = my_robot_controller.controlador:main"
        ],
    },
)
