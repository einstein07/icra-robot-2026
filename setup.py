from setuptools import find_packages, setup

package_name = "osoyoo_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sindiso Mkhatshwa",
    maintainer_email="sindisomkhatshwa@gmail.com",
    description="Osoyoo base: Twist -> wheel PWM on the Raspberry Pi.",
    license="BSD-3-Clause",
    entry_points={"console_scripts": ["base_node = osoyoo_base.base_node:main"]},
)
