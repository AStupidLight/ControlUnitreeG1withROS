from setuptools import find_packages, setup

package_name = 'robot_task_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests', 'openai'],
    zip_safe=True,
    maintainer='raychen',
    maintainer_email='dengyingshulang0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
    "console_scripts": [
        "prompt_router = robot_task_tools.prompt_router_node:main",
    ],
},

)
