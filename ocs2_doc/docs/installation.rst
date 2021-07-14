.. index:: pair: page; Installation
.. _doxid-ocs2_doc_installation:

Installation
============

Prerequisites
~~~~~~~~~~~~~

The OCS2 library is written in C++11. It is tested under Ubuntu 20.04 with library versions as 
provided in the package sources.


Dependencies
------------

* C++ compiler with C++11 support
* Eigen (v3.3)
* Boost C++ (v1.71)
* GLPK ``sudo apt install libglpk-dev``
* catkin ``sudo apt-get install catkin``
* pybind11_catkin, ROS package, installable via ``sudo apt install ros-noetic-pybind11-catkin``
* catkin-pkg package for python3. Install with ``sudo apt install python3-catkin-tools``
* Doxygen for documentation. Install with ``sudo apt install doxygen doxygen-latex``


Optional Dependencies
---------------------

* For rigid multi-body dynamics library and self collision support clone `Pinocchio`_ and `HPP-FCL`_ into your workspace

.. _`Pinocchio`: https://github.com/stack-of-tasks/pinocchio
.. _`HPP-FCL`: https://github.com/humanoid-path-planner/hpp-fcl

    .. code-block:: bash
    
        # Clone pinocchio
        git clone git@github.com:leggedrobotics/pinocchio.git
        # Clone hpp-fcl
        git clone git@github.com:leggedrobotics/hpp-fcl.git
        # install dependencies 
        sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev

* `RaiSim <https://github.com/raisimTech/raisimLib>`__ simulator can be used as a provider for rollouts. The corresponding ``ocs2_raisim`` package has additional requirements:
  
    RaiSim package, which needs to be installed from source by cloning it into your workspace:
    
    .. code-block:: bash
    
    	git clone --depth 1 https://github.com/raisimTech/raisimLib.git
    
    For installation, follow the `instructions <https://raisim.com/sections/Installation.html>`__ 
    of the RaiSim webpage. Alternatively, in order to make the installation easy to find for 
    catkin and easy to uninstall in the future, consider using 
    `CheckInstall <https://help.ubuntu.com/community/CheckInstall>`__ which will install RaiSim 
    as a debian package. For this, replace the original commands:
    
    .. code-block:: bash
    
    	cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL <other options>
    	make install -j4
    
    with:
    
    .. code-block:: bash
    
    	cmake .. <other options>     (Without INSTALL_PREFIX)
    	make -j4 && sudo checkinstall
    
    This will install RaiSim at the default location of ``/usr/local/lib`` which will also be automatically detected by catkin. When asked by ``checkinstall``, change the package name to something appropriate (e.g. "raisim") to it find later for package management, such as for uninstallation (``dpkg -r <package_name>``).
  
* `Grid Map <https://github.com/ANYbotics/grid_map>`__ catkin package, which may be installed with ``sudo apt install ros-noetic-grid-map-msgs``.

* `RaisimOgre <https://github.com/leggedrobotics/raisimOgre>`__ Visualizer for Raisim. Can be used for debugging purposes to see if conversions between OCS2 and Raisim are correct.


.. _doxid-ocs2_doc_installation_ocs2_doc_install:

Installation
~~~~~~~~~~~~


Build the library
-----------------

Create a new catkin workspace:

.. code-block:: bash

    # Create the directories
    # Do not forget to change <...> parts
    mkdir -p <directory_to_ws>/<catkin_ws_name>/src
    cd <directory_to_ws>/<catkin_ws_name>/

    # Initialize the catkin workspace
    catkin init
    catkin config --extend /opt/ros/noetic
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo

Clone the OCS2 library:

.. code-block:: bash

    # Navigate to the directory of src
    # Do not forget to change <...> parts
    cd <directory_to_ws>/<catkin_ws_name>/src
    git clone git@github.com:leggedrobotics/ocs2.git

Build and run the unit tests:

.. code-block:: bash 

    # Build it
    catkin build ocs2

    # Source it
    source <directory_to_ws>/<catkin_ws_name>/devel/setup.bash

    # run tests
    catkin run_tests ocs2


Build this Documentation
------------------------

Assuming python catkin tools are installed, run the following command:

.. code-block:: bash

    # Navigate to the directory of ocs2_doc
    # Do not forget to change <...> parts
    cd <directory_to_ws>/<catkin_ws_name>/src/ocs2/ocs2_doc

    # make build directory
    mkdir -p build
    # Navigate to the build folder
    cd build

    # build docs
    cmake ..
    make

This will build the documentation and place it in the ``build/output/sphinx`` folder. 
Open the ``index.html`` in your web browser. 

