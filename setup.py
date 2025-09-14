from setuptools import setup, find_packages

setup(
    name="tasgnss",
    version="0.1.0",
    author="Runzhi Hu",
    author_email="run-zhi.hu@connect.polyu.hk",
    description="A Python package for GNSS positioning and processing",
    long_description=open("readme.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/tasgnss",  # Update with actual URL if available
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
    ],
)