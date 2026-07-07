.. _doc_going_forward_simulation:

RoboRacer Simulator
===========================================

The RoboRacer simulator lets you develop and test autonomy code without a physical car. It is maintained as a separate project, so this page points you to the current, actively
maintained simulator documentation rather than duplicating it here.

**ROS 2 simulator (recommended).** The ``f1tenth_gym_ros`` bridge runs the simulator in ROS 2 and mirrors the interfaces used on the car, so code you write against it ports
directly to a real vehicle:

* `f1tenth_gym_ros on GitHub <https://github.com/f1tenth/f1tenth_gym_ros>`_

**Core gym environment.** The underlying Gym environment (multi-agent, no ROS required, suitable for RL policy training) and its documentation:

* `f1tenth_gym documentation <https://f1tenth-gym.readthedocs.io/>`_
* `f1tenth_gym on GitHub <https://github.com/f1tenth/f1tenth_gym>`_

Follow the setup and usage instructions in those repositories to get the simulator running.
