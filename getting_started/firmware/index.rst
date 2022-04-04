.. _doc_build_car_firmware:

Install F1TENTH Driver Stack
==============================
.. note:: This section assumes that you have already completed :ref:`Building the Car <doc_build_car>` and :ref:`System Configuration <doc_software_setup>`.

At the end of this section, you will have the VESC tuned and the lidar connection completed.

**Required Equipment:**
	* Fully built F1TENTH vehicle
	* (see each section for more required equipment)

**Difficulty Level:** Intermediate-Advanced

**Approximate Time Investment:** 1.5 hour

With the physical car is built and the system configuration setup, we can start to install the firmware needed on the car.

There are two subsections here. You will need to :ref:`Configure the VESC <doc_firmware_vesc>` but you can skip the :ref:`Hokuyo Ethernet Connection <doc_firmware_hokuyo10>` section if you have a lidar (e.g. Hokuyo 30LX) that is connected via USB instead.

.. toctree::
   :maxdepth: 1
   :name: Firmware Setup
   :hidden:

   firmware_vesc
   firmware_hokuyo10
   drive_workspace

#. :ref:`Configuring the VESC <doc_firmware_vesc>` goes over how to set up and tune the VESC.
#. :ref:`Hokuyo 10LX Ethernet Connection Setup <doc_firmware_hokuyo10>` details how to set up the connection on the Jetson NX.
#. :ref:`F1TENTH Driver Stack Setup <doc_drive_workspace>` goes over how to setup the software drivers necessary to drive the vehicle.

.. tip::
  If you have any build and/or setup questions, post to the `forum <https://f1tenth.discourse.group/>`_.
