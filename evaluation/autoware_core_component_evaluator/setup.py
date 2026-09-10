from glob import glob
import os

from setuptools import find_packages, setup

package_name = "autoware_core_component_evaluator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "expectation"), glob("expectation/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    # `colcon test` selects its Python test runner on this declaration: without a pytest test
    # dependency it falls back to `python -m unittest`, which discovers nothing under test/ and
    # reports "Ran 0 tests ... OK" -- a green result for a suite that never ran.
    # It must be the `test` extra, not `tests_require`: setuptools dropped the latter, so it is
    # warned away as an unknown option and never reaches colcon.
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="TranHuuNhatHuy",
    maintainer_email="huy9515@gmail.com",
    description="Per-component evaluation of autoware_core under scenario_simulator.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "component_evaluator = autoware_core_component_evaluator.evaluator_node:main",
            "evaluation_report = autoware_core_component_evaluator.report:main",
        ],
    },
)
