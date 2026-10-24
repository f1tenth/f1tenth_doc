.. _doc_offroad_simulator:

Off-Road Simulation
===================

The RoboRacer Off-Road can be simulated closed-loop with the `Autoware-RoboRacer Off-Road Simulator <https://github.com/autowarefoundation/autoware_off-road_sim>`_, which runs in Docker on an x86 machine. Autoware connects to it in place of the real vehicle and sensors. There are two setups.

Software-in-the-loop
--------------------

Autoware and the simulator both run on one x86 host. This is the quickest way to try the software without any hardware. See the full instructions: `Software-in-the-Loop Simulation <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/simulation/software-in-loop.md>`_.

Hardware-in-the-loop
--------------------

Autoware runs on the vehicle's Jetson AGX Orin and the simulator runs on a separate x86 machine, connected by Ethernet. This tests the real compute and network setup before driving on the track. It needs the Jetson from the :ref:`hardware build <doc_offroad_build>`, or at least a Jetson with Autoware installed.

Full steps: `Hardware-in-the-Loop Simulation <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/simulation/hardware-in-loop.md>`_.

.. note::
   Both setups need the network and DDS settings from the `Autoware DDS documentation <https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/>`_.
