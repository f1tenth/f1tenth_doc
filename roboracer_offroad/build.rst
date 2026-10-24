.. _doc_offroad_build:

Building the Car
================

This part covers the hardware build of the RoboRacer Off-Road. At the end you will have a vehicle ready for the :ref:`Autoware stack <doc_offroad_autoware>`.

The build has four parts:

#. :ref:`Lower-level chassis <doc_offroad_lower_chassis>`: the RC platform, drivetrain, motor controller and steering servo.
#. :ref:`Preparing the enclosure <doc_offroad_enclosure>`: the housing that holds the compute and electronics.
#. :ref:`Autonomy elements <doc_offroad_autonomy>`: the Jetson AGX Orin, ZED camera, capture card and power distribution.
#. :ref:`Putting it all together <doc_offroad_together>`: mounting the enclosure on the chassis, wiring, and a first power-on check.

.. danger::
   **LIPO (LITHIUM POLYMER) BATTERY SAFETY WARNING**

   The RoboRacer Off-Road uses lithium polymer batteries. They store a large amount of energy in a small space and can start a fire if misused.

   * Charge batteries only while monitored, in a fireproof bag, on a non-flammable surface.
   * Do not leave a LiPo battery connected to the vehicle when it is not in use.
   * Unplug the battery immediately if you notice popping sounds, bloating, a burning smell, or smoke.
   * Never short the battery leads, and never connect the battery backwards.

.. note::
   If you have any build or setup questions, post to the `RoboRacer Slack <https://join.slack.com/t/robo-racer/shared_invite/zt-42lsbf50y-_3YPNLl_d3s~wPylAOMg0g>`_.

.. toctree::
   :maxdepth: 1
   :caption: Building the Off-Road Car
   :name: sec-offroad-build
   :titlesonly:
   :hidden:

   lower_level_chassis
   enclosure
   autonomy_elements
   all_together
