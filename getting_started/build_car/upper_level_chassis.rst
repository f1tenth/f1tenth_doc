.. _doc_build_upper_level:


3. Upper Level Chassis
========================

In the third part we’re going to assemble the upper level chassis and mount the important components. The complete process for the third hardware part can be watched in this video tutorial too.

.. raw:: html

	<iframe width="560" height="315" src="https://www.youtube.com/embed/L-V-0zzkl10?start=530" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

	
Place the **Platform Deck** so it’s facing you like this. The **Platform Deck** is the `laser cut piece <https://drive.google.com/drive/u/1/folders/1o3jRww0UwfmjTBDACD8qu7SDabRzpr5g>`_.

.. figure:: img/ulchassis/ulchassis01.JPG
	:align: center

	Platform Deck.

To make things easier to follow, we define **Front**, **Rear**, **Away**, and **Towards** like above.

.. important::
	**This piece is not symmetric.** Notice in the figure above that the big M5 VESC mounting holes marked in yellow are not centered. The holes need to be skewed **TOWARDS** to you.

The following image shows the five components that we will be mounting on the Platform Deck.

.. figure:: img/ulchassis/ulchassis02.JPG
	:align: center

	Locations of parts to be mounted.

1. Mounting the VESC and the Powerboard
-----------------------------------------
Place the VESC on the Platform Deck so that the power wires are facing AWAY from you and the three cables labelled **A**, **B**, and **C** are facing TOWARDS you.

.. figure:: img/ulchassis/ulchassis03.JPG
	:align: center

	VESC placed in correct position on Platform Deck.

Flip the Platform Deck and the VESC over and use two M5 screws mounted diagonal from each other to attach the VESC to the Platform Deck, like shown below.

.. figure:: img/ulchassis/ulchassis04.JPG
	:align: center

	Mounting holes for Powerboard and VESC. Note that this is view of the bottom of the Platform Deck.

The image above shows the three holes used to mount the Powerboard. Only three of the Powerboard mounting holes are used. Flip the Platform Deck with the VESC attached to it back over so the VESC is facing up. Attach the Powerboard to the Platform deck by using the M3 screws that were removed from the chassis in the Lower Level Chassis section. Thread the M3 screw from underneath the Platform Deck up and secure with a 25mm standoff. Then, Secure the Powerboard to the 25mm standoff with a M3 x 8mm screw.

.. figure:: img/ulchassis/ulchassis19.JPG
	:align: center

	Side view of Powerboard mounted on Platform Deck.

.. figure:: img/ulchassis/ulchassis05.JPG
	:align: center

	Top view of Powerboard mounted on Platform Deck. The green terminal blocks are facing the FRONT of the car.

There should be a gap between the Powerboard and the VESC.

.. figure:: img/ulchassis/ulchassis06.JPG
	:align: center

	Notice there is space between the VESC and the Powerboard.

2. Mounting the TX2 and Antenna
------------------------------------------------
Using four of the M3 screws that held the Nerf Bars to the chassis, attach the TX2 assembly to the Platform Deck. The screws attach from the bottom side of the Platform Deck. The I/O connectors on the TX2 assembly should face towards the rear of the vehicle and the fan should be immediately above the middle opening of the Platform Deck.

.. figure:: img/ulchassis/ulchassis07.JPG
	:align: center

	Bottom view of the Platform Deck with the Jetson's fan peeking through.

Connect two wires (preferably red and black) from the green terminal on the Orbitty to one 12V terminal on the Powerboard. Make sure that the red wire connects +VIN on the Orbitty to 12V on the Powerboard terminal. The black wire should connect GND on the Orbitty to GND on the Powerboard terminal.

.. DANGER::
	**MAKE SURE THE POLARITY IS CORRECT. +VIN TO 12V. ORBITTY GND TO GND.** If you plug this in backwards, fire will happen and global warming will immediately speed up exponentially. And you do not want to be the cause of the death of the dinosaurs part two.

.. figure:: img/ulchassis/ulchassis11.JPG
	:align: center

	Powering the Orbitty/TX2 with the Powerboard.

Mount the antenna to the rear of the VESC.

.. figure:: img/ulchassis/ulchassis08.JPG
	:align: center

	Top view of the Platform Deck with the Antenna, VESC, Powerboard, and TX2/Orbitty mounted.

3. Mounting and Connecting the Lidar and USB Hub
---------------------------------------------------
The last component to mount is the lidar. Here, we use the Hokuyo UTM-30LX. The mounting holes for the Hokuyo UST-10LX are slightly different.

.. figure:: img/ulchassis/ulchassis20.JPG
	:align: center

	Lidar mounting holes.

Use the appropriate mounting holes for your lidar.

Use four M3 screws to mount from underneath.

.. figure:: img/ulchassis/ulchassis10.JPG
	:align: center

	Lidar mounted on Platform Deck.

To mount the USB hub, place a piece of double sided tape on the back of the USB hub like so:

.. figure:: img/ulchassis/ulchassis12.JPG
	:align: center

	Double sided tape affixed tot he bottom of the USB hub.

Stick the hub next to the TX2 and press firmly down.

.. figure:: img/ulchassis/ulchassis13.JPG
	:align: center

	USB HUB attached next to TX2.

Plug the USB hub into the USB port on the Orbitty Carrier Board. So far, the top view of the upper level chassis should look like this:

.. figure:: img/ulchassis/ulchassis14.JPG
	:align: center

	Top view with USB HUB attached and plugged in.

A. Cable Management
^^^^^^^^^^^^^^^^^^^
The lidar comes with two very long cables. We are going to try out best to manage them. Split the two cables of the lidar and loop them under the slots on the Platform Deck.

.. figure:: img/ulchassis/ulchassis15.JPG
	:align: center

	Looping the lidar cables through the slots.

Using a twist tie, rubber band, or zip tie, gather the majority of the cables on each side.

For the stripped cable side, insert the **BROWN (POWER)** and **BLUE (GROUND)** wires into one of the 12V terminal blocks on the Powerboard.

.. DANGER::
	***BROWN is POWER and BLUE is GROUND.*  DO NOT MIX THESE UP OTHERWISE YOU WILL FRY YOUR VERY EXPENSIVE LIDAR.** And then life will be very very sad. When in doubt, check the side of the Hokuyo. It will list out the correspondence of each wire.

.. figure:: img/ulchassis/ulchassis16.JPG
	:align: center

	Side view with wires coiled.

.. figure:: img/ulchassis/ulchassis17.JPG
	:align: center

	Lidar power is plugged into the terminal block with Brown to Power and Blue to Ground.

If using the UTM-30LX, plug the USB into one of the ports on the USB hub.

.. figure:: img/ulchassis/ulchassis18.JPG
	:align: center

	USB side of lidar plugged into USB hub. If you are using a 10LX, plug it into the ethernet port on the Orbitty.

If using the UST-10LX, plug the ethernet cable into the ethernet port on the Orbitty board.

The upper level chassis is complete and we're ready to assemble everything!

.. figure:: img/ulchassis/ulchassis21.gif
   :align: center
   :width: 300px
