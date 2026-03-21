from setuptools import setup, find_packages

setup(
    name="hexapod",
    version="0.1.2",
    author="ggldnl",
    author_email="danielgigliotti99.dg@gmail.com",
    description="Hexapod controller package",
    url="https://github.com/ggldnl/Hexapod-Controller.git",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.7",
    install_requires=[
        "numpy",
        "pyserial",
        "sshkeyboard",
    ],
    package_data={
        'controller.config': ['*.yml']
    }
)