.. _doc_software_jetson:

Configuring the Jetson TX2
==========================
| We need to install 4 items on the Jetson:

#. Linux GUI
#. Jetpack 3.2 flash
#. A re-flash of the Connect Tech Orbitty
#. ROS Kinetic

The TX2 is essentially a supercomputer on a module. We attach it to the Orbitty Carrier board so that we can access the TX2's peripheral.

0. Connecting the TX2
-------------------------
4 things need to be connected to the TX2.

#. A display via the HDMI port
#. A keyboard via one of the USB ports on the USB hub.
#. The Pit laptop via a micro USB
#. Power

.. image:: img/jetson/jetson01.JPG

The TX2 should power on. If it doesn't, press the ON button.

.. note::

	**KIM WILL PROBABLY HAVE TO TRANSCRIBE THIS (after Billy does the checking)**

`Professor Rosa Zheng <http://www.lehigh.edu/~yrz218/>`_ from Lehigh University has compiled a fantastic on how to set up the software.

.. raw:: html

	<iframe width="700" height="500" src="https://drive.google.com/file/d/1N1FiPtAqpbeAYlKoFA4Tsxl0XC_Y8niT/preview" width="640" height="480"></iframe>

..
	A command prompt window asking for login credentials should show on your Monitor. The login information is:

	.. code-block:: bash

		Login: nvidia
		Password: nvidia

	.. seealso::
		These instructions are also in the Jetson’s `Quick Start Guide <https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-282/index.html>`_ under “Force USB Recovery Mode”. Refer to it to see all the buttons, ports and whatnot.


	1. Installing the Linux GUI
	------------------------------
	In the command prompt window, run

	.. code-block:: bash

		$ cat NVIDIA-INSTALLER/README.txt
		
	Follow the instructions that are in that file to install Ubuntu Linux. Note that TX1 comes with 14.04 LTS and TX2 comes with 16.04 LTS. There may be an additional step for TX1 if the course is using 16.04 LTS.

	2. Flashing the Jetpack
	^^^^^^^^^^^^^^^^^^^^^^^^^
	.. note:: 
		You will need some 14GB of free space on the host computer for this step. If you have previously used the same TX2 for other projects, you may need to remove some files to make space.

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