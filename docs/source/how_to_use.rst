How to Use
==========

Installation
------------

1. **Install PyBullet Industrial**

   Ensure that the latest version from the main branch is installed:

   .. code-block:: bash

      git clone https://github.com/WBK-Robotics/pybullet_industrial.git
      cd pybullet_industrial
      pip install src/

2. **Install OMPL with Python bindings**

   To install OMPL with Python bindings, follow the official instructions at:
   `https://ompl.kavrakilab.org/python.html <https://ompl.kavrakilab.org/python.html>`_.

   For easy setup, we recommend using the prebuilt Python bindings from the `OMPL development build <https://github.com/ompl/ompl/releases/tag/prerelease>`_.

   .. note::

      Prebuilt binaries are currently available for Linux and macOS only.
      Windows users are advised to use WSL (Windows Subsystem for Linux) to install OMPL.
      This includes the following steps:
        Install WSL and Ubuntu
            wsl --install -d Ubuntu
        Update and install Python
            sudo apt update && sudo apt upgrade -y

            sudo apt install python3 python3-venv python3-pip -y
        Create and activate a virtual environment
            python3 -m venv ompl_venv

            source ompl_venv/bin/activate
        Install the prebuilt OMPL Python bindings
            wget `https://github.com/\.../ompl-<version>.whl`

            pip install ompl-<version>.whl


3. **Install the Path Planner Package**

   Clone and install the path planner package:

   .. code-block:: bash

      git clone https://github.com/WBK-Robotics/pybullet_industrial_path_planner.git
      cd pybullet_industrial_path_planner
      pip install src/