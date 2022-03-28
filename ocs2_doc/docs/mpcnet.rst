.. index:: pair: page; MPC-Net

.. _doxid-ocs2_doc_mpcnet:

MPC-Net
=======

MPC-Net is an imitation learning approach that uses solutions from MPC to guide the policy search.
The main idea is to imitate MPC by minimizing the control Hamiltonian while representing the corresponding control inputs by a parametrized policy.
MPC-Net can be used to clone a model predictive controller into a neural network policy, which can be evaluated much faster than MPC.
Therefore, MPC-Net is a useful proxy for MPC in computationally demanding applications that do not require the most exact solution.

The multi-threaded data generation and policy evaluation run asynchronously with the policy training.
The data generation and policy evaluation are implemented in C++ and run on CPU, while the policy training is implemented in Python an runs on GPU.
The control Hamiltonian is represented by a linear quadratic approximation.
Therefore, the training can run on GPU without callbacks to OCS2 C++ code running on CPU to evaluate the Hamiltonian, and one can exploit batch processing on GPU.

Robots
~~~~~~

MPC-Net has been implemented for the following :ref:`Robotic Examples <doxid-ocs2_doc_robotic_examples>`:

============= ================ ============== ======== =============
Robot         Recom. CPU Cores Recom. GPU RAM RaiSim   Training Time
============= ================ ============== ======== =============
ballbot       4                2 GB           No       1 min
legged_robot  12               8 GB           Yes / No 8 min
============= ================ ============== ======== =============

Setup
~~~~~

Make sure to follow the :ref:`Installation <doxid-ocs2_doc_installation>` page.
Follow all the instructions for the dependencies.
Regarding the optional dependencies, make sure to follow the instruction for ONNX Runtime as well as the virtual environment, and optionally set up RaiSim.

To build all MPC-Net packages, build the metapackage:

.. code-block:: bash

    cd ~/catkin_ws
    catkin_build ocs2_mpcnet

To build a robot-specific package, replace :code:`<robot>` with the robot name:

.. code-block:: bash

    cd ~/catkin_ws
    catkin_build ocs2_<robot>_mpcnet

Training
~~~~~~~~

To train an MPC-Net policy, run:

.. code-block:: bash

    cd ~/git/ocs2/ocs2_mpcnet/ocs2_<robot>_mpcnet/python/ocs2_<robot>_mpcnet
    source ~/catkin_ws/devel/setup.bash
    source ~/venvs/mpcnet/bin/activate
    python3 <robot>_mpcnet.py

To monitor the training progress with Tensorboard, run:

.. code-block:: bash

    cd ~/git/ocs2/ocs2_mpcnet/ocs2_<robot>_mpcnet/python/ocs2_<robot>_mpcnet
    source ~/venvs/mpcnet/bin/activate
    tensorboard --logdir=runs

If you use RaiSim, you can visualize the data generation and policy evaluation runs with RaiSim Unity, where pre-built executables are provided in the :code:`~/git/raisimLib/raisimUnity` folder. For example, on Linux run:

.. code-block:: bash

    ~/git/raisimLib/raisimUnity/linux/raisimUnity.x86_64

Deployment
~~~~~~~~~~

To deploy the default policy stored in the robot-specific package's :code:`policy` folder, run:

.. code-block:: bash

    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch ocs2_<robot>_mpcnet <robot>_mpcnet.launch

To deploy a new policy stored in the robot-specific package's :code:`./python/ocs2_<robot>_mpcnet/policies` folder, replace :code:`<path>` with the absolute file path to the final policy and run:

.. code-block:: bash

    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch ocs2_<robot>_mpcnet <robot>_mpcnet.launch policyFile:=<path>

References
~~~~~~~~~~

This part of the toolbox has been developed based on the following publications:

.. bibliography::
   :list: enumerated

   carius2020mpcnet
   reske2021imitation
