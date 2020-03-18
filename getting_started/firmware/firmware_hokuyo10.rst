.. _doc_firmware_hokuyo10:

2. Hokuyo 10LX Ethernet Connection Setup
==========================================
.. note::
	If you have a 30LX or a LIDAR that connects via USB, you can skip this section.

**Equipment Required:**
	* Fully built F1TENTH vehicle with a Hokuyo 10LX lidar
	* Pit/Host Laptop OR
	* External monitor/display, HDMI cable, keyboard, mouse

**Approximate Time Investment:** 30 minutes

Connect to the TX2 either via SSH or a wired connection (monitor, keyboard, mouse).

In order to utilize the 10LX you must first configure the eth0 network. From the factory the 10LX is assigned the following ip: ``192.168.0.10``. Note that the lidar is on subnet 0.

Open **Network Configuration** in the Linux GUI on the TX2. In the ipv4 tab, add a route such that the eth0 port is assigned

	* IP address ``192.168.0.15``
	* Subnet mask is ``255.255.255.0``
	* Gateway is ``192.168.0.1``

Call the connection ``Hokuyo``. Save the connection and close the network configuration GUI.

When you plug in the 10LX make sure that the Hokuyo connection is selected. If everything is configured properly you should now be able to ping ``192.168.0.1``.

.. image:: img/hokuyo1.gif
	:align: center
	:width: 200px
