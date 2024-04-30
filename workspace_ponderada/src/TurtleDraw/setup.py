from setuptools import find_packages, setup

package_name = 'TurtleDraw'

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
    maintainer='rodrigo-07',
    maintainer_email='rodrigo.santos@sou.inteli.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turtle = TurtleDraw.move_turtle:main',
            'draw_turtle = TurtleDraw.draw:main',
            'draw_turtle_image = TurtleDraw.draw_image_turtles:main',
        ],
    },
)
