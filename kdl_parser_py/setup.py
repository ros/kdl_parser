from setuptools import setup

package_name = 'kdl_parser_py'

setup(
    name=package_name,
    version='2.6.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['test/test.urdf'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@osrfoundation.org"',
    description='The Kinematics and Dynamics Library (KDL)'
    'defines a tree structure to represent the kinematic and'
    'dynamic parameters of a robot mechanism. <tt>kdl_parser_py</tt>'
    'provides Python tools to construct a KDL tree from an XML robot representation in URDF.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
