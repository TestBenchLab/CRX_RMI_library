from setuptools import setup, find_packages

setup(
    name="rmi_library",
    version="1.0",
    packages=find_packages(),  # Find-submodules
    install_requires=[  # External Dependencies
        "numpy>=1.22.4",
    ],
    author="Valentin Deguil / Victor Gaudin / Christophe Poulain",
    description="RMI Library to send robot commands through TCP/IP",
    url="https://github.com/TestBenchLab/CRX_RMI_library/tree/standalone",
)
