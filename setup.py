from setuptools import setup

package_name = "bytewax_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "bytewax"],
    zip_safe=True,
    maintainer="lee-ros",
    maintainer_email="laluk321@gmail.com",
    description="An extention package for the Bytewax data-processing library to allow connection for ROS2",
    license="Apache-2.0 license",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
