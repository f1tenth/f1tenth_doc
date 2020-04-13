.. _doc_software_host:

1. Pit/Host Setup
==================
**Equipment Used:**
	* Laptop/Computer

**Approximate Time Investment:** 1-2 hours

Overview
----------
The F1TENTH vehicle runs on a Linux operating system. You will need to have Linux and ROS (Robot Operating System) on your **Pit** laptop. We will assume that you already have some basic knowledge of Linux and ROS as explaining either systems in depth is beyond the scope of this documentation.

We will refer to the **Pit** computer as **Pit** or **Host** computer/laptop interchangeably.

In this **System Configuration** section,the **Pit** laptop is first used to flash the JetPack onto the **TX2** and then to SSH into the **TX2**. After this section, the **Pit**'s sole job is to be used to SSH in to the **TX2**.

.. important:: We currently support **Ubuntu Xenial 16.04/ROS Kinetic** and **Bionic 18.04/ROS Melodic**.

1. Installing Ubuntu
---------------------
If you do not already have Linux running natively on your laptop, we suggestion you dual boot or install Ubuntu on an external hard drive. You can download the Ubuntu image `here <https://ubuntu.com/download/desktop>`_. Instructions for installing Ubuntu can be found `here <https://ubuntu.com/tutorials/tutorial-install-ubuntu-desktop#1-overview>`_. 

If you use a VM instead, see :ref:`Appendix A <doc_appendix_a>` on sharing folders.

Installing Ubuntu may take a while so you may want to get a cup of coffee or tea and settle in.

2. Installing ROS
------------------
Follow the instructions `here <https://wiki.ros.org/ROS/Installation>`_ to install the supported ROS versions above.

If you have never used ROS before, ROS has many in-depth and useful tutorials `here <https://wiki.ros.org/ROS/Tutorials>`_ that you may want to try after installing.

With a Linux operating system on the **Pit/Host** computer, you're ready to move on to setting up the **TX2!**

.. image:: img/host/host01.gif
	:align: center
	:width: 300px

