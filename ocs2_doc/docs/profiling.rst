.. index:: pair: page; How to Profile the Solvers
.. _doxid-ocs2_doc_profiling:

How to Profile the Solvers
==========================

This document lists some profiling and benchmarking tools and how to use 
them with OCS2.



.. _doxid-ocs2_doc_profiling_1ocs2_doc_profiling_general:

Test conditions
~~~~~~~~~~~~~~~

To improve accuracy and make comparison fair turn off powersave and 
turbo boost:

.. code-block:: bash

	cpupower frequency-set --governor performance
	echo "1" | tee /sys/devices/system/cpu/intel_pstate/no_turbo
	# run your benchmark
	cpupower frequency-set --governor powersave
	echo "0" | tee /sys/devices/system/cpu/intel_pstate/no_turbo

You may also want to build in Release mode and enable architecture 
specific features:

.. code-block:: cpp

	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native -mtune=native"





.. _doxid-ocs2_doc_profiling_1ocs2_doc_profiling_builtin:

Use the built-in timers
~~~~~~~~~~~~~~~~~~~~~~~

OCS2 keeps internal statistics of the solver timings. If enabled, the 
solver will print a summary from the destructor. You can enable them 
in the task.info file:

.. code-block:: ini

	; DDP settings
	ddp
	{
	  ; ...
	  displayShortSummary   true
	  ; ...
	}

or you can directly access them through ``getBenchmarkingInfo()`` method.





.. _doxid-ocs2_doc_profiling_1ocs2_doc_profiling_perf:

linux perf-tools
~~~~~~~~~~~~~~~~

Overview:

* Perf collects statistical data of your process

* ``perf stat`` : CPU performance counters
  
  * instructions
  
  * branches, branch-misses
  
  * L1, LLC cache loads and misses
  
  * context-switches

* ``perf record`` :
  
  * see in which functions time is spent

* Note: perf does not give exact measurements but only a statistical approximation!

Installation:

.. code-block:: bash

	sudo apt-get install linux-tools-common linux-tools-generic linux-tools-`uname -r`

Usage:

* Compile with ``-DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer``

* Run the target process and retrieve the PID: eg. ``ps -eo pid,command | grep ocs2_anymal_croc_mpc_node | grep -v grep``

* For recording perf.data: ``sudo perf record -g -p PID``

* For gathering performance counter statistics: ``sudo perf stat -d -p PID``

* Stop recording with Ctrl-C. The data is written to ``perf.data`` or stderr respectively.

* Analyze perf.data with: sudo perf report -g 'graph,0.5,caller -i perf.data<tt>

* Alternative:rosrun ocs2_benchmark run_perf.py stat timeout=60 output=stat.csv`:
  
  * automatically finds and attaches to process
  
  * runs for a given timeout

Generate a flamegraph from perf.data:

* ``git clone https://github.com/brendangregg/FlameGraph.git``

* ``cd FlameGraph``

* ``perf script --max-stack=20 -i path/to/perf.data | ./stackcollapse-perf.pl | ./flamegraph.pl > flame.svg``

* open the interactive SVG in a web browser

References:

* `Tutorial wiki.kernel.org <https://perf.wiki.kernel.org/index.php/Tutorial>`__

* `perf Examples <http://www.brendangregg.com/perf.html>`__





.. _doxid-ocs2_doc_profiling_1ocs2_doc_profiling_valgrind:

Valgrind
~~~~~~~~

Installation:

.. code-block:: bash

	sudo apt-get install valgrind



.. _doxid-ocs2_doc_profiling_1ocs2_doc_profiling_massif:

Massif: A Heap Profiler
-----------------------

Valgrind Massif profiles memory usage for snapshots, which are taken at 
regular time intervals. It allows analyzing memory usage down to 
individual functions and lines if debug symbols are available.

Usage:

* Launch with ``launch-prefix="valgrind --tool=massif"``

* retrieve ``~/.ros/massif.out.PID``

* ``ms_print massif.out.PID | less -S``

massif-visualizer GUI:

* ``sudo apt-get install massif-visualizer``

* ``massif-visualizer massif.out.PID``

References:

* `Massif Manual <https://www.valgrind.org/docs/manual/ms-manual.html>`__





.. _doxid-ocs2_doc_profiling_1ocs2_doc_profiling_cachegrind:

Cachegrind: A Cache and Branch-prediction Profiler
--------------------------------------------------

Usage:

* Compile with debug info: ``-DCMAKE_BUILD_TYPE=RelWithDebInfo``

* Launch with ``launch-prefix="valgrind --tool=cachegrind"``

* retrieve ``~/.ros/cachegrind.out.PID``

* ``cg_annotate cachegrind.out.PID | less -S``

References:

* `Cachegrind Manual <https://valgrind.org/docs/manual/cg-manual.html>`__

