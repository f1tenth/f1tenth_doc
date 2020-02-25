.. _doc_software_jetson:

Jetson Setup
==================

| This page details the steps to setup the Jetson.
| We need to install 4 items on the Jetson:

#. Linux GUI
#. Jetpack 3.2 flash
#. A re-flash of the Connect Tech Orbitty
#. ROS Kinetic

Connect terminals to the Jetson (aka "the device")
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First, a few things need to be connected to the Jetson. These instructions are also in the Jetson’s `Quick Start Guide <https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-282/index.html>`_ under “Force USB Recovery Mode”. Refer to it to see all the buttons, ports and whatnot.

#. Connect a display to the jetson via HDMI port
#. Connect a USB keyboard
#. Connect the jetson to your host PC via the USB micro-B plug
#. Plug to power

.. image:: img/software.jpg

The Jetson should power on. If it doesn’t, push the ON button.

.. code-block:: bash

	Login: nvidia
	Password: nvidia


1. Install Linux
^^^^^^^^^^^^^^^^^
Run

.. code-block:: bash

	$ cat NVIDIA-INSTALLER/README.txt
	
And run the instructions that are in that file to install Ubuntu Linux. Note that TX1 comes with 14.04 LTS and TX2 comes with 16.04 LTS. There may be an additional step for TX1 if the course is using 16.04 LTS.

2. Flashing the Jetpack
^^^^^^^^^^^^^^^^^^^^^^^^^
**NOTE: you will need some 14GB of free space on the host computer for this step.**

Now that we have the GUI, we want to flash the Jetson with Nvidia’s `Jetpack 4.3 <https://developer.nvidia.com/embedded/jetpack>`_.

To do this, we need a host computer that is running Linux 14.04 (it seems 16.04 also works - try it if that’s what you have). The Jetpack is first downloaded onto the host computer and then transferred by micro USB cable over to the Jetson. Download the file and follow the instructions `here <https://developer.nvidia.com/embedded/jetpack>`_ to transfer.

What if you don’t have a Linux 14.04 computer laying around? (Most of us don’t.) See ​
:ref:`Appendix A <doc_appendix_a>` of this doc for an amazing set of instructions by Klein Yuan which details how to use a virtual machine with a Mac to do the flash. Steps would probably work similarly for a PC that is running Virtual Box.

3. Re-flashing the Orbitty
^^^^^^^^^^^^^^^^^^^^^^^^^^^
After the Jetson has been flashed with Jetpack, we will actually need to re-flash it with the Connect Tech Orbitty firmware. Otherwise on the TX2 there can be issues with the USB 3.0 not working on the Orbitty carrier board. A great link to instructions is from `NVIDIA-Jetson <https://github.com/NVIDIA-AI-IOT/jetson-trashformers/wiki/Jetson%E2%84%A2-Flashing-and-Setup-Guide-for-a-Connect-Tech-Carrier-Board>`_. Note that each time you flash all of the files will essentially be deleted from your Jetson​. So make sure to save any work you may have already done and upload it.

4. Installing ROS 
^^^^^^^^^^^^^^^^^^^^^^^^^
| Lastly, we will want to install ROS Kinetic. Jetson Hacks on Github has scripts to install ROS Kinetic.
| `Here <https://github.com/jetsonhacks/installROSTX2​>`_ for TX2
| `Here <https://github.com/jetsonhacks/installROSTX1​>`_ for TX1