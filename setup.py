#!/usr/bin/env python3
"""Setup script for MCP Rosbag Server."""

from setuptools import setup, find_packages

setup(
    name="mcp-rosbag-server",
    version="0.1.0",
    description="Model Context Protocol server for ROS bag analysis",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "mcp",
        "rosbags",
        "numpy",
    ],
    extras_require={
        "ros2": ["rclpy", "rosidl_runtime_py"],
    },
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "mcp-rosbag-server=server:main",
        ],
    },
)