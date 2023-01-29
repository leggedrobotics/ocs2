.. index:: pair: page; overview

.. _doxid-ocs2_doc_overviewpage:

Overview
========

**OCS2** is a C++ toolbox tailored for **O**\ ptimal **C**\ ontrol 
for **S**\ witched **S**\ ystems (OCS2). The toolbox provides an 
efficient implementation of the following algorithms:

* **SLQ**\: Continuous-time domain constrained DDP.
* **iLQR**\: Discrete-time domain constrained DDP.
* **SQP**\: Multiple-shooting algorithm based on `HPIPM <https://github.com/giaf/hpipm>`__.
* **SLP**\: Sequential Linear Programming based on `PIPG <https://arxiv.org/abs/2009.06980>`__.
* **IPM**\: Multiple-shooting algorithm based on nonlinear interior point method.

OCS2 handles general path constraints through Augmented Lagrangian or 
relaxed barrier methods. To facilitate the application of OCS2 in robotic 
tasks, it provides the user with additional tools to set up the system 
dynamics (such as kinematic or dynamic models) and cost/constraints 
(such as self-collision avoidance and end-effector tracking) from a URDF 
model. The library also provides an automatic differentiation tool to 
calculate derivatives of the system dynamics, constraints, and cost. To 
facilitate its deployment on robotic platforms, the OCS2 provides tools 
for ROS interfaces. The toolbox’s efficient and numerically stable 
implementations in conjunction with its user-friendly interface have 
paved the way for employing it on numerous robotic applications with 
limited onboard computation power.


How to use the OCS2 toolbox?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

OCS2 can be easily installed on Ubuntu. The `source code`_ is also publicly available.
To get started with the control toolbox, please refer to the 
:ref:`Installation started <doxid-ocs2_doc_installation>`, and
:ref:`Getting Started <doxid-ocs2_doc_getting_started>` pages.

.. _`source code`: https://github.com/leggedrobotics/ocs2


Licence
~~~~~~~

The OCS2 toolbox is released under the *BSD 3-Clause* license. Please 
note the license and notice files in the source directory.


Credits 
~~~~~~~~
The following people have been involved in the development of OCS2:

**Project Manager**: 
Farbod Farshidian.

**Main Developers**: 
Farbod Farshidian,
Ruben Grandia,
Michael Spieler,
Jan Carius,
Jean-Pierre Sleiman.

**Other Developers**:
Alexander Reske,
Sotaro Katayama,
Mayank Mittal,
Jia-​Ruei Chiu,
Johannes Pankert,
Perry Franklin,
Tom Lankhorst,
David Hoeller,
Asutosh Satapathy,
Markus Giftthaler,
Edo Jelavic.


**Acknowledgement**: The OCS2 toolbox development initiated by the ADRL team at ETH Zurich, and the 
project has continued to evolve at RSL, ETH Zurich. The RSL team now actively 
supports the development of OCS2.

Citing OCS2
~~~~~~~~~~~

To cite the toolbox in your academic research, please use the following:

.. code-block:: latex

      @misc{OCS2,
         title = {{OCS2}:  An  open  source  library  for  Optimal  Control  of  Switched Systems},
         note = {[Online]. Available: \url{https://github.com/leggedrobotics/ocs2}},
         author = {Farbod Farshidian and others}
      }

Tutorials
~~~~~~~~~

* Tutorial on OCS2 toolbox, Farbod Farshidian, MPC Workshop, RSS 2021 (`link <https://youtu.be/RYmQN9GbFYg>`__).

* Real-time optimal control for legged locomotion and manipulation, Marco Hutter, MPC Workshop, RSS 2021 (`link <https://youtu.be/sjAENmtO4bA>`__).


Related publications
~~~~~~~~~~~~~~~~~~~~

This toolbox has been used in the following publications:

.. bibliography::
   :list: enumerated
   :all:

   farshidian2017slq
   farshidian2017ocs2
   farshidian2017slqmpc
   giftthaler2017kinematic
   minniti2019ballbot
   grandia2019frequency
   grandia2019feedback
   gawel2019construction
   farshidian2020deepmpc
   carius2020mpcnet
   grandia2020cbf
   sleiman2021locopulation
   gaertner2021collision
   sleiman2021constraint
   reske2021imitation
   minniti2021adaptive
   mittal2021articulated