from setuptools import setup
from glob import glob

package_name = "kdl_parser_py"

setup(
    name=package_name,
    version="2.3.0",
    author="Jonathan Bohren, Jackie Kay",
    packages=[package_name],
    data_files=[
        # Include package file
        ("share/" + package_name, ["package.xml"]),
        # Install marker file in package index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
)
