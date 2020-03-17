.. _doc_build_car_firmware:

Installing Firmware
====================
At the end of this section, you will have the VESC tuned and the lidar connection completed.

**Required Equipment:**
	* Fully built F1TENTH vehicle 
	* (see each section for more required equipment)

**Difficulty Level:** Medium

**Approximate Time Investment:** 15-45 minutes


With the physical car is built and the system configuration setup, we can start to install the firmware needed on the car.

.. note:: 
  If you have any build and/or setup questions, post to the `forum <http://f1tenth.org/forum.html>`_.

There are two subsections here. You will need to :ref:`Configure the VESC <doc_firmware_vesc>` but you can skip the :ref:`Hokuyo Ethernet Connection <doc_firmware_hokuyo10>` section if you have a lidar that is connected via USB instead.

.. toctree::
   :maxdepth: 1
   :name: Firmware Setup

   firmware_vesc
   firmware_hokuyo10 

#. :ref:`Configuring the VESC <doc_firmware_vesc>` goes over how to set up and tune the VESC. 
#. :ref:`Hokuyo 10LX Ethernet Connection Setup <doc_firmware_hokuyo10>` details how to set up the connection on the TX2.