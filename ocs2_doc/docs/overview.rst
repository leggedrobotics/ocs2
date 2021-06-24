.. index:: pair: page; overview
.. _doxid-overviewpage:

Overview
========

**OCS2** is a C++ toolbox tailored for **O**\ ptimal **C**\ ontrol for **S**\ witched **S**\ ystems (OCS2). The toolbox provides an efficient implementation of the following algorithms:

* **SLQ**\: Continuous-time domin DDP
* **iLQR**\: Discrete-time domain DDP
* **SQP**\: Multiple-shooting algorithm based on `HPIPM <href="https://github.com/giaf/hpipm"/>`__
* **PISOC**\: Path integral stochatic optimal control

OCS2 handles general path constraints through Augmented Lagrangian or relaxed barrier methods. To facilitate the application of OCS2 in robotic tasks, it provides the user with additional tools to set up the system dynamics (such as kinematic or dynamic models) and cost/constraints (such as self-collision avoidance and end-effector tracking) from a URDF model. The library also provides an automatic differentiation tool to calculate derivatives of the system dynamics, constraints, and cost. The toolbox’s efficient and numerically stable implementations in conjunction with its user-friendly interface have paved the way for employing it on numerous robotic applications with limited onboard computation power. The library consists of the following main modules:


.. _doxid-index_1ocs2_doc_source_code:

Source code
~~~~~~~~~~~

The `source code`_ is publicly available.

.. _`source code`: https://bitbucket.org/leggedrobotics/ocs2/



.. _doxid-index_1cs2_doc_how_to_use:

How to use the OCS2 toolbox?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To get started with the control toolbox, please see :ref:`getting started <doxid-ocs2_doc_getting_started>`.


.. _doxid-index_1ocs2_doc_licence:

Licence
~~~~~~~

The OCS2 toolbox is released under the *BSD 3-Clause* license. Please note the license and notice files in the source directory.


.. _doxid-index_1ocs2_doc_credits:

Credits 
~~~~~~~~
The following people have been involved in the development of the OCS2: (TBD)

* Farbod Farshidian (ETHZ): the project manager and one of the developers
* Ruben Grandia (ETHZ): one of the developers
* Jan Carius (ETHZ): one of the developers
* Michael Spieler (ETHZ): one of the developers
* Jean-Pierre Sleiman (ETHZ): centroidal model and augmented lagrangian
* Johannes Pankert (ETHZ): features extension
* Tom Lankhorst (ETHZ): documentation and continuous integration
* Perry Franklin (ETHZ): features extension
* David Hoeller (ETHZ): features extension
* Asutosh Satapathy (ETHZ): features extension
* Markus Giftthaler (ETHZ): features extension


.. _doxid-index_1ocs2_doc_ack:

Acknowledgement
~~~~~~~~~~~~~~~
TBD


.. _doxid-index_1ocs2_doc_related:

Related publications
~~~~~~~~~~~~~~~~~~~~

this toolbox has been used in the following publications:

.. bibliography::

   farshidian17d
   farshidian17a
   farshidian17b
   giftthaler17