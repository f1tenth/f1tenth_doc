.. _doc_software_host:

Host/Laptop Setup
==================

The f1tenth vehicle runs using ROS. This page walks you through how to set up the operating system on your laptop and install the simulator. 

.. note:: We currently support OS Ubuntu Xenial 16.04.01 and ROS Kinetic. Could other combinations work? Sure. Will we support them? Nope.

Installing Ubuntu
------------------
If you do not already have Linux running natively on your laptop, we suggestion you dual boot or install Ubuntu on an external hard drive. Instructions for installing Ubuntu can be found `here <http://releases.ubuntu.com/xenial/>`_. If you use a VM instead, see :ref:`Appendix B <doc_appendix_b>` on sharing folders.

Installing ROS
------------------
Follow the instructions `here <https://github.com/mlab-upenn/f1tenth>`_ to install ROS Kinetic.

Note: you might get the following error message when you execute

.. code-block:: bash

	$ sudo apt-get install ros-kinetic-desktop-full
	Building dependency tree
	Reading state information... Done
	Some packages could not be installed. This may mean that you have requested an impossible situation or if you are using the unstable distribution that some required packages have not yet been created or been moved out of Incoming.
	The following information may help to resolve the situation:

	The following packages have unmet dependencies:
	ros-kinetic-desktop-full : Depends: ros-kinetic-desktop but it is not going to be installed

	E: Unable to correct problems, you have held broken packages.

You will find many suggestions online. `This one <https://askubuntu.com/questions/140246/how-do-i-resolve-unmet-dependencies-after-adding-a-ppa>`_ worked for us.

Specifically, these steps (but it's goot to try the steps in the suggested order):

.. code-block:: bash

	$ sudo apt-get -u dist-upgrade
	$ sudo apt-get -o Debug::pkgProblemResolver=yes dist-upgrade

Then re-run

.. code-block:: bash

	$ sudo apt-get update

and re-try installing ros-kinetic-desktop-full

If you have never used ROS before, ROS has many in-depth and useful tutorials `here <http://wiki.ros.org/ROS/Tutorials>`_ that you may want to try after installing.
