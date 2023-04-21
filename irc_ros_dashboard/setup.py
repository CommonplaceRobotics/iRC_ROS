from setuptools import setup

package_name = "irc_ros_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Felix Reuter",
    maintainer_email="fer@cpr-robots.com",
    description="A dashboard for monitoring and controlling modules",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["cli_dashboard = irc_ros_dashboard.cli_dashboard:main"],
    },
)
