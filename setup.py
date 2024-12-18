from setuptools import setup, find_packages

setup(
    name='rmi_library',              
    version='0.1',                    
    packages=find_packages(),         # Find-submodules
    install_requires=[                # Dependencies
        # Exemple : 'numpy>=1.19'
    ],
    author='Valentin Deguil / Victor Gaudin',
    description='RMI Library to send robot commands through TCP/IP',
    url='https://github.com/TODO',
)
