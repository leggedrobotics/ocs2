.. index:: pair: page; MPC-Net

.. _doxid-ocs2_doc_mpcnet:

MPC-Net
=======

MPC-Net is an imitation learning approach that uses solutions from MPC to guide the policy search.
The main idea is to imitate MPC by minimizing the control Hamiltonian while representing the corresponding control inputs by a parametrized policy.
MPC-Net can be used to clone a model predictive controller into a neural network policy, which can be evaluated much faster than MPC.
Therefore, MPC-Net is a useful proxy for MPC in computationally demanding applications that do not require the most exact solution.

The multi-threaded data generation and policy evaluation run asynchronously with the policy training.
The data generation and policy evaluation are implemented in C++ and run on CPU, while the policy training is implemented in Python and runs on GPU.
The control Hamiltonian is represented by a linear-quadratic approximation.
Therefore, the training can run on GPU without callbacks to OCS2 C++ code running on CPU to evaluate the Hamiltonian, and one can exploit batch processing on GPU.

Robots
~~~~~~

MPC-Net has been implemented for the following :ref:`Robotic Examples <doxid-ocs2_doc_robotic_examples>`:

============= ================ ================= ======== =============
Robot         Recom. CPU Cores Recom. GPU Memory RaiSim   Training Time
============= ================ ================= ======== =============
ballbot       4                2 GB              No       0m 20s
legged_robot  12               8 GB              Yes / No 7m 40s
============= ================ ================= ======== =============

Setup
~~~~~

Make sure to follow the :ref:`Installation <doxid-ocs2_doc_installation>` page.
Follow all the instructions for the dependencies.
Regarding the optional dependencies, make sure to follow the instruction for ONNX Runtime and the virtual environment, optionally set up RaiSim.

To build all MPC-Net packages, build the meta package:

.. code-block:: bash

    cd <path_to_catkin_ws>
    catkin_build ocs2_mpcnet

    # Example:
    cd ~/catkin_ws
    catkin_build ocs2_mpcnet

To build a robot-specific package, replace :code:`<robot>` with the robot name:

.. code-block:: bash

    cd <path_to_catkin_ws>
    catkin_build ocs2_<robot>_mpcnet

    # Example:
    cd ~/catkin_ws
    catkin_build ocs2_ballbot_mpcnet

Training
~~~~~~~~

To train an MPC-Net policy, run:

.. code-block:: bash

    cd <path_to_ocs2_repo>/ocs2_mpcnet/ocs2_<robot>_mpcnet/python/ocs2_<robot>_mpcnet
    source <path_to_catkin_ws>/devel/setup.bash
    source <path_to_venvs>/mpcnet/bin/activate
    python3 train.py

    # Example:
    cd ~/git/ocs2/ocs2_mpcnet/ocs2_ballbot_mpcnet/python/ocs2_ballbot_mpcnet
    source ~/catkin_ws/devel/setup.bash
    source ~/venvs/mpcnet/bin/activate
    python3 train.py

To monitor the training progress with Tensorboard, run:

.. code-block:: bash

    cd <path_to_ocs2_repo>/ocs2_mpcnet/ocs2_<robot>_mpcnet/python/ocs2_<robot>_mpcnet
    source <path_to_venvs>/mpcnet/bin/activate
    tensorboard --logdir=runs

    # Example:
    cd ~/git/ocs2/ocs2_mpcnet/ocs2_ballbot_mpcnet/python/ocs2_ballbot_mpcnet
    source ~/venvs/mpcnet/bin/activate
    tensorboard --logdir=runs

If you use RaiSim, you can visualize the data generation and policy evaluation rollouts with RaiSim Unity, where pre-built executables are provided in RaiSim's :code:`raisimUnity` folder. For example, on Linux run:

.. code-block:: bash

    <path_to_raisimLib_repo>/raisimUnity/linux/raisimUnity.x86_64

    # Example:
    ~/git/raisimLib/raisimUnity/linux/raisimUnity.x86_64

Deployment
~~~~~~~~~~

To deploy the default policy stored in the robot-specific package's :code:`policy` folder, run:

.. code-block:: bash

    cd <path_to_catkin_ws>
    source devel/setup.bash
    roslaunch ocs2_<robot>_mpcnet <robot>_mpcnet.launch

    # Example:
    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch ocs2_ballbot_mpcnet ballbot_mpcnet.launch

To deploy a new policy stored in the robot-specific package's :code:`python/ocs2_<robot>_mpcnet/runs` folder, replace :code:`<path>` with the absolute file path to the final policy and run:

.. code-block:: bash

    cd <path_to_catkin_ws>
    source devel/setup.bash
    roslaunch ocs2_<robot>_mpcnet <robot>_mpcnet.launch policyFile:=<path>

    # Example:
    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch ocs2_ballbot_mpcnet ballbot_mpcnet.launch policyFile:='/home/user/git/ocs2/ocs2_mpcnet/ocs2_ballbot_mpcnet/python/ocs2_ballbot_mpcnet/runs/2022-04-01_12-00-00_ballbot_description/final_policy.onnx'

How to Set Up a New Robot
~~~~~~~~~~~~~~~~~~~~~~~~~

Setting up MPC-Net for a new robot is relatively easy, as the **ocs2_mpcnet_core** package takes care of the data generation as well as policy evaluation rollouts and implements important learning components, such as the memory, policy, and loss function.

This section assumes that you already have the packages for the robot-specific MPC implementation:

1. **ocs2_<robot>**: Provides the library with the robot-specific MPC implementation.
2. **ocs2_<robot>_ros**:  Wraps around the MPC implementation with ROS to define ROS nodes.
3. **ocs2_<robot>_raisim**:  (Optional) interface between the robot-specific MPC implementation and RaiSim.

For the actual **ocs2_<robot>_mpcnet** package, follow the structure of existing robot-specific MPC-Net packages.
The most important classes/files that have to be implemented are:

* **<Robot>MpcnetDefinition**: Defines how OCS2 state variables are transformed to the policy observations. and how policy actions are transformed to OCS2 control inputs.
* **<Robot>MpcnetInterface**: Provides the interface between C++ and Python, allowing to exchange data and policies.
* **<robot>.yaml**: Stores the configuration parameters.
* **mpcnet.py**: Adds robot-specific methods, e.g. implements the tasks that the robot should execute, for the MPC-Net training.
* **train.py**: Starts the main training script.

Known Issues
~~~~~~~~~~~~

Stiff inequality constraints can lead to very large Hamiltonians and gradients of the Hamilltonian near the log barrier.
This can obstruct the learning process and the policy might not learn something useful.
In that case, enable the gradient clipping in the robot's MPC-Net YAML configuration file and tune the gradient clipping value.

References
~~~~~~~~~~

This part of the toolbox has been developed based on the following publications:

.. bibliography::
   :list: enumerated

   carius2020mpcnet
   reske2021imitation
