import os

from setuptools import setup


package_name = "kdl_parser_py"
share_path = os.path.join("share", package_name)

setup(
    name=package_name,
    version="2.3.0",
    author="Jonathan Bohren, Jackie Kay",
    packages=[package_name],
    data_files=[
        # Include package file
        (share_path, ["package.xml"]),
        # Install marker file in package index
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        # Install test resources
        (os.path.join(share_path, "assets"), [os.path.join("test", "test.urdf")]),
    ],
    tests_require=["pytest"],
    test_suite="test",
)
