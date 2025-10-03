from setuptools import setup, find_packages
import pathlib
from pathlib import Path

this_directory = Path(__file__).parent
setup(
    name="tasgnss",
    version="0.1.3",
    author="Runzhi Hu",
    author_email="run-zhi.hu@connect.polyu.hk",
    description="A Python package for GNSS positioning and processing by TASLAB",
    long_description=open(this_directory/"README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/PolyU-TASLAB/TASGNSS", 
    packages=find_packages(),
    install_requires=[
        "pyrtklib",
        "numpy",
        "pymap3d",
    ],
    python_requires=">=3.7",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: Python :: 3.13",
    ],
)