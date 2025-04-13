import setuptools

setuptools.setup(
    name='pybulet_industrial_pathplanner',
    version='0.1.0',
    author='Philipp Schulz',
    description='pybulet_industrial_pathplanner extends process-aware simulation with path planning capabilities.',
    long_description='pybulet_industrial_pathplanner integrates process-aware path planning methods into the pybullet simulation framework. The module provides efficient robotic path optimization and simulation components.',
    url='https://github.com/yourusername/pybulet_industrial_pathplanner',  # Update with the actual repository URL.
    license='MIT',
    install_requires=[
        'numpy<2.0.0',
        'pybullet',
        'scipy'
    ],
    packages=setuptools.find_packages(where="src"),
    package_dir={"": "src"}
)