from setuptools import setup

package_name = 'capstone'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'scripts.Turtle_1_Control',
        'scripts.center_node',
        'scripts.Map_Plot',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Description of the package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_1_control = scripts.Turtle_1_Control:main',
            'center_node = scripts.Center_Node:main',
            'map_plot = scripts.Map_Plot:main',
        ],
    },
)
