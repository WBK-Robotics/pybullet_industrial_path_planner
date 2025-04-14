import setuptools


setuptools.setup(
    name='pybullet_industrial_path_planner',
    version='1.0.0',
    author='Philipp Schulz',
    description='Pybullet Industrial Path Planner extends Pybullet Industrial to plan collision-free, constraint and optimal paths by integrating the Open Motion Planning Library.',
    long_description='Pybullet Industrial Path Planner extends Pybullet Industrial to plan collision-free, constraint and optimal paths by integrating the Open Motion Planning Library.',
    url='https://github.com/WBK-Robotics/pybullet_industrial_pathplanner',
    license='MIT',
    install_requires=['numpy<2.0.0', 'pybullet', 'scipy'],
    packages=setuptools.find_packages(),
)