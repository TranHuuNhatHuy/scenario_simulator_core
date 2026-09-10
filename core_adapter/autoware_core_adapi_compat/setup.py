import os

from setuptools import find_packages, setup

package_name = "autoware_core_adapi_compat"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TranHuuNhatHuy",
    maintainer_email="huy9515@gmail.com",
    description="AD API operation-mode endpoints missing from autoware_core.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "adapi_operation_mode = autoware_core_adapi_compat.adapi_compat_node:main",
        ],
    },
)
