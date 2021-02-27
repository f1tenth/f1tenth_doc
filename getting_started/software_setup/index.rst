.. _doc_software_setup:

System Configuration
========================
.. note:: This section assumes that you have already completed :ref:`Building the Car <doc_build_car>`.

At the end of this section, you will have the TX2 and host laptop set up.

**Required Equipment:**
	* Fully built F1TENTH vehicle
	* Laptop/computer
	* External monitor/display
	* HDMI cable
	* Keyboard
	* Mouse
	* Wireless router (or your phone as a wireless hotspot)
	* Ethernet cable (needed if Pit/Host laptop does not have WiFi capability)

**Difficulty Level:** Intermediate-Advanced

**Approximate Time Investment:** 2-3 hours

Now that you have a working physical F1TENTH car, you'll want to program it to do something useful, otherwise it's just a glorified and expensive car model at this point.

This **System Configuration** section will walk you through how to configure the TX2 so that you will be able to run programs on the F1TENTH Autonomous Vehicle System and communicate with the Vehicle.

The image below represents the flow of information on the F1TENTH Autonomous Vehicle System.

.. figure:: img/f1tenth_sys_flow.png
  	:align: center

	Flow of information on the F1TENTH Autonomous Vehicle System.

The **TX2** is the main brain of the entire system. It gives commands to the **VESC** which controls the **Servo** and the **Brushless Motor**. The **TX2** also receives information from the **Lidar**. The **Pit** laptop is where we can connect remotely to the **TX2**.

This section has five subsections:

.. toctree::
   :maxdepth: 1
   :name: Software Setup

	 software_host
	 optional_software_nx
   software_combine
   software_advance
	 software_jetson


#. :ref:`Pit/Host setup <doc_software_host>` details how to install ROS and the simulators on the **Pit/Host** computer/laptop.
#. :ref:`Setup NVIDIA Jetson Xavier NX <doc_optional_software_nx>` contains all necessary steps to configure the **NVIDIA Jetson Xavier NX** if you are using this module instead of the **TX2**.
#. :ref:`Combine setup <doc_software_combine>` goes over how to set up a wireless communication system between the **Pit/Host** laptop and the **TX2** once you have already completed the above two sections.
#. :ref:`Advanced Setup <doc_software_advance>` concerns hotspot setup and VNC setup on the TX2. This section is optional and for the advanced user.
#. :ref:`DEPRECATED - Configuring the TX2 <doc_software_jetson>` goes over how to set up the **TX2**.


.. tip::
  If you have any build and/or setup questions, post to the `forum <http://f1tenth.org/forum.html>`_.

Many thanks to `Dr. Rosa Zheng <http://www.lehigh.edu/~yrz218/>`_ from Lehigh University for compiling the majority of information in this section.
