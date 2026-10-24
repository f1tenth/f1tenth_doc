.. _doc_offroad_autoware:

Installing the Autoware Stack
=============================

The RoboRacer Off-Road software is `Autoware <https://github.com/mlab-upenn/autoware.roboracer>`_ with interfaces and configuration for this platform. It lives in the `roboracer_humble branch <https://github.com/mlab-upenn/autoware.roboracer/tree/roboracer_humble>`_ of the `autoware.roboracer repository <https://github.com/mlab-upenn/autoware.roboracer>`_. The instructions there are the source of truth, so this page links to them rather than copying them.

Supported platforms
-------------------

* **Jetson AGX Orin** (JetPack 6.2.1, Ubuntu 22.04): the target platform on the vehicle.
* **x86_64 host** (Ubuntu 22.04): recommended for development, faster builds and simulation.

Installation
------------

* `Installation, Jetson AGX Orin <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/installation/jetson-agx-orin.md>`_
* `Installation, x86 Host <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/installation/x86-host.md>`_

Running and configuring
-----------------------

* `Architecture Overview <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/architecture.md>`_
* `Launch Modes <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/launch-modes.md>`_: Regular, Lite and Minimal, and when to use each
* `On-Vehicle Operation <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/on-vehicle-operation.md>`_: driving the real vehicle
* `Map Creation & Calibration <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/map-creation.md>`_: point cloud and Lanelet2 maps
* `RoboRacer Off-Road Configuration <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/configuration/roboracer-offroad.md>`_: parameter overrides for this platform
* `Track Tuning Guide <https://github.com/mlab-upenn/autoware.roboracer/blob/roboracer_humble/docs/configuration/tuning-guide.md>`_: symptom-based tuning

.. note::
   Complete the :ref:`hardware build <doc_offroad_build>` before the on-vehicle steps. The installation and simulation steps do not need the vehicle.
