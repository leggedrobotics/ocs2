.. index:: pair: page; Frequently Asked Questions

.. _doxid-ocs2_doc_faq:

Frequently Asked Questions
==========================


At runtime, I receive warnings on setting thread priority
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The priority of a thread affects how the thread is scheduled by the OS. 
A higher priority thread is likely to be scheduled more often and lowers 
the latency of its execution.

We can set the thread priority for the solver through the `threadPriority`
attribute in the `task.info` file. This is set to `50` in the
provided robotic examples.

Normal users cannot set priorities higher than `0` by default. To allow 
your linux user account to do so, you need to append the following 
entries to the `/etc/security/limits.conf` file (replacing <username> with
your username):

.. code-block::
    
    <username>       -       rtprio           99
    <username>       -       nice            -20

To allow your entire group to set higher priorities, append (replacing <groupname>
with the created group name):

.. code-block::

    @<groupname>     -       rtprio           99
    @<groupname>     -       nice            -20

