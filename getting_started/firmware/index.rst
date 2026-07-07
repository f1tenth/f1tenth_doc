.. _doc_build_car_firmware:

Install RoboRacer Driver Stack
==============================
.. note:: This section assumes that you have already completed :ref:`Building the Car <doc_build_car>` and :ref:`System Configuration <doc_software_setup>`.

At the end of this section, you will have the VESC tuned and the lidar connection completed.

**Required Equipment:**
	* Fully built RoboRacer vehicle
	* (see each section for more required equipment)

**Difficulty Level:** Intermediate-Advanced

**Approximate Time Investment:** 1.5 hour

With the physical car is built and the system configuration setup, we can start to install the firmware needed on the car.

You will need to :ref:`Configure the VESC <doc_firmware_vesc>` and set up the driver stack. Use the native :ref:`Driver Stack Setup <doc_drive_workspace>` (section 2) **or** the :ref:`Docker Containers <doc_drive_workspace_docker>` version (section 3) depending on your JetPack version — not both. LiDAR setup (Hokuyo or SICK) is covered within the Driver Stack Setup section.

.. toctree::
   :maxdepth: 1
   :name: Firmware Setup
   :hidden:

   firmware_vesc
   drive_workspace
   drive_workspace_docker

#. :ref:`Configuring the VESC <doc_firmware_vesc>` goes over how to set up and tune the VESC.
#. :ref:`RoboRacer Driver Stack Setup <doc_drive_workspace>` goes over how to set up the software drivers natively to drive the vehicle on **JetPack 6 (Ubuntu 22.04)**.
#. :ref:`RoboRacer Driver Stack Setup with Docker Containers <doc_drive_workspace_docker>` goes over how to set up the software drivers on **older JetPack releases (Ubuntu 20.04)** using **Docker Containers**.

.. tip::
  If you have any build and/or setup questions, post to the `RoboRacer Slack <https://join.slack.com/t/robo-racer/shared_invite/zt-42lsbf50y-_3YPNLl_d3s~wPylAOMg0g>`_.
