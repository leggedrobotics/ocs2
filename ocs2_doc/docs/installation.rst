.. index:: pair: page; Installation
.. _doxid-ocs2_doc_installation:

Installation
============

Prerequisites
~~~~~~~~~~~~~

The OCS2 library is written in C++11. It is tested under Ubuntu 20.04 with library versions as 
provided in the package sources.

Source code
------------
The source code is hosted on GitHub: `leggedrobotics/ocs2 <https://github.com/leggedrobotics/ocs2>`_. 

    .. code-block:: bash
    
        # Clone OCS2
        git clone git@github.com:leggedrobotics/ocs2.git

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
        git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
        # Clone hpp-fcl
        git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
        # install dependencies 
        sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev

* For various robotic assets used in OCS2 unit tests and the robotic examples

.. _`ocs2_robotic_assets`: https://github.com/leggedrobotics/ocs2_robotic_assets

    .. code-block:: bash
    
        # Clone ocs2_robotic_assets
        git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
     
* `rqt_multiplot`_ package can be used for visualizing the solver's performance indices and other optimization outputs

.. _`rqt_multiplot`: http://wiki.ros.org/rqt_multiplot

    .. code-block:: bash
    
        sudo apt-get install ros-noetic-rqt-multiplot

* `RaiSim <https://github.com/raisimTech/raisimLib>`__ simulator can be used as a provider for rollouts. The corresponding ``ocs2_raisim`` package has additional requirements:
  
    RaiSim package, which needs to be installed from source by cloning it into your workspace:
    
    .. code-block:: bash
    
    	git clone --depth 1 https://github.com/raisimTech/raisimLib.git -b v1.1.01
    
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

    For visualization, use `RaiSim Unity <https://raisim.com/sections/RaisimUnity.html>`__, where pre-built executables are provided in the ``raisimLib/raisimUnity`` directory. For example, it can be used for debugging purposes to see if conversions between OCS2 and RaiSim are correct.

* `Grid Map <https://github.com/ANYbotics/grid_map>`__ catkin package, which may be installed with ``sudo apt install ros-noetic-grid-map-msgs``.

* `ONNX Runtime  <https://github.com/microsoft/onnxruntime>`__ is an inferencing and training accelerator. Here, it is used for deploying learned :ref:`MPC-Net <doxid-ocs2_doc_mpcnet>` policies in C++ code. To locally install it, do the following:

    .. code-block:: bash

        cd /tmp
        wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz
        tar xf onnxruntime-linux-x64-1.7.0.tgz
        mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime
        rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime
        rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib
        rsync -a ~/git/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/cmake/onnxruntime

    We provide custom cmake config and version files to enable ``find_package(onnxruntime)`` without modifying ``LIBRARY_PATH`` and ``LD_LIBRARY_PATH``. Note that the last command above assumes that you cloned OCS2 into the folder ``git`` in your user's home directory.

* `Virtual environments  <https://docs.python.org/3/library/venv.html>`__ are recommended when training :ref:`MPC-Net <doxid-ocs2_doc_mpcnet>` policies:

    .. code-block:: bash

        sudo apt-get install python3-venv

    Create an environment and give it access to the system site packages:

    .. code-block:: bash

        mkdir venvs && cd venvs
        python3 -m venv mpcnet

    Activate the environment and install the requirements:

    .. code-block:: bash

        source ~/venvs/mpcnet/bin/activate
        python3 -m pip install -r ~/git/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/requirements.txt

    Newer graphics cards might require a CUDA capability which is currently not supported by the standard PyTorch installation.
    In that case check `PyTorch Start Locally  <https://pytorch.org/get-started/locally/>`__ for a compatible version and, e.g., run:

    .. code-block:: bash

        pip3 install torch==1.10.2+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html

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

