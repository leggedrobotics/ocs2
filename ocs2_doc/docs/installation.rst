.. index:: pair: page; Installation
.. _doxid-ocs2_doc_installation:

Installation
============



.. _doxid-ocs2_doc_installation_1ocs2_doc_requirements:

Requirements
~~~~~~~~~~~~

This library is written in C++11. It is tested under Ubuntu 18.04 with library versions as provided in the package sources.



.. _doxid-ocs2_doc_installation_1ocs2_doc_dep:

Dependencies
------------

* C++ compiler with C++11 support

* `Eigen (v3.3) <http://eigen.tuxfamily.org/index.php?title=Main_Page>`__

* `Boost C++ (v1.54) <http://www.boost.org/=Main_Page>`__

* catkin

* `GLPK <https://www.gnu.org/software/glpk/>`__
  
  .. code-block:: bash
  
  	sudo apt install libglpk-dev

* ``catkin-pkg`` package for python3. Install with ``sudo apt install python3-catkin-tools``

* ``pybind11_catkin`` ROS package, installable via ``sudo apt install ros-noetic-pybind11-catkin``

* Doxygen for documentation. Install with ``sudo apt install doxygen doxygen-latex``





.. _doxid-ocs2_doc_installation_1ocs2_doc_dep_optinal:

Optional Dependencies
---------------------

* `RaiSim <http://raisim.com>`__ simulator can be used as a provider for rollouts. The corresponding ``ocs2_raisim`` package has additional requirements:
  
  * ` <https://github.com/raisimTech/raisimLib>`__ package, which needs to be installed from source by cloning it into your workspace:
    
    .. code-block:: bash
    
    	git clone --depth 1 https://github.com/raisimTech/raisimLib.git
    
    For installation, follow the `instructions <https://raisim.com/sections/Installation.html>`__ of the RaiSim webpage. Alternatively, in order to make the installation easy to find for catkin and easy to uninstall in the future, consider using ` <https://help.ubuntu.com/community/CheckInstall>`__ which will install RaiSim as a debian package. For this, replace the original commands:
    
    .. code-block:: bash
    
    	cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL <other options>
    	make install -j4
    
    with:
    
    .. code-block:: bash
    
    	cmake .. <other options>     (Without INSTALL_PREFIX)
    	make -j4 && sudo checkinstall
    
    This will install RaiSim at the default location of ``/usr/local/lib`` which will also be automatically detected by catkin. When asked by ``checkinstall``, change the package name to something appropriate (e.g. "raisim") to it find later for package management, such as for uninstallation (``dpkg -r <package_name>``).
  
  * ` <https://github.com/ANYbotics/grid_map>`__ catkin package, which may be installed with ``sudo apt install ros-melodic-grid-map-msgs``.

* `RaisimOgre <https://github.com/leggedrobotics/raisimOgre>`__ Visualizer for Raisim. Can be used for debugging purposes to see if conversions between OCS2 and Raisim are correct.

* For pinocchio (Rigid multi-body dynamics library) and self collision support:
  
  * clone `pinocchio <https://github.com/leggedrobotics/pinocchio>`__ and `hpp-fcl <https://github.com/leggedrobotics/hpp-fcl>`__ into your workspace (use master branch).
  
  * ``sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev``







.. _doxid-ocs2_doc_installation_1ocs2_doc_install:

Installation
~~~~~~~~~~~~



.. _doxid-ocs2_doc_installation_1ocs2_doc_build_lib:

Build the library
-----------------

.. code-block:: bash

	cd catkin_ws/src
	git clone git@bitbucket.org:leggedrobotics/ocs2.git
	catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo

To build and run the unit tests run:

.. code-block:: bash

	catkin run_tests ocs2





.. _doxid-ocs2_doc_installation_1ocs2_doc_build_doc:

Build this Documentation
------------------------

Assuming python catkin tools are installed, run the following command:

.. code-block:: bash

	catkin build ocs2_doc

This will build the documentation and place it in the ``ocs2_doc/doc/html`` folder.

