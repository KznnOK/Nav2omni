from setuptools import find_packages, setup


package_name = "omni_robot_description"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/omni_robot_description",
            ],
        ),
        (
            "share/omni_robot_description",
            [
                "package.xml",
            ],
        ),
        (
            "share/omni_robot_description/launch",
            [
                "launch/display.launch.py",
            ],
        ),
        (
            "share/omni_robot_description/urdf",
            [
                "urdf/omni_robot.urdf.xacro",
            ],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Robot description for the four-wheel omni robot",
    license="TODO",
    entry_points={
        "console_scripts": [],
    },
)
