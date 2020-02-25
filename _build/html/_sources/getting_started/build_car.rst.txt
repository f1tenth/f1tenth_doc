.. _doc_build_car:


Building the Car
==================

Bill of Materials:
**KIM NEEDS TO UPDAT LINK BECAUSE THIS IS OUTDATED**
The master BOM can be found `here <https://docs.google.com/spreadsheets/d/1ykUyrZq-vLlMTf0TIcdMrRbGKcRWglW6ol76QyGst2I/edit#gid=2038095912>`_

Tools Needed:
**KIM NEEDS TO UPDATE**
- list things here

Difficulty Level: Medium
**KIM NEEDS TO UPDATE**

Approximate Time Investment: 1 hour

**KIM WILL REBUILD CAR AND UPDATE THESE SECTIONS**
**KIM ADD VIDEO OF BUILD AND UPDATE CAR BUILD**

.. warning:: 
	**LIPO (LITHIUM POLYMER) BATTERY SAFETY WARNING**
	
	LiPO batteries allow your car to run for a long time, but they are not something to play with or joke about. They store a large amount of energy in a small space and can damage your car and cause a fire if used improperly. With this in mind, here are some safety tips for using them with the car.

	* When charging batteries, always monitor them and place them in a fireproof bag on a non-flammable surface clear of any other objects.
	* Do not leave a LIPO battery connected to the car when you’re not using it. The battery will discharge and its voltage will drop to a level too low to charge it safely again.
	* Unplug the battery from the car immediately if you notice any popping sounds, bloating of the battery, burning smell, or smoke.
	* Never short the battery leads.
	* Do not plug the battery in backwards. This will damage the VESC and power board (and likely the Jetson as well) and could cause a short circuit.
	* See ​this `video <https://www.youtube.com/watch?v=gz3hCqjk4yc>`_ for an example of what might happen if you don’t take care of your batteries. Be safe and don’t let these happen to you!


Preparing and Assembling the Car
---------------------------------

.. image:: img/F10Materials.png

Preparing the Car
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Coming Soon: will fill out when the fourth Traxxas car arrives

Obtaining the Power Board
[Instructions on getting the power board from Penn,
and a link to files that you would send to a PCB company like 4PCB or PCBWay. Currently on ​github​]
A note on why we have a power board:​ The power board is used to provide a stable voltage source for the car and its peripherals since the battery voltage drops as the battery runs out. The board does not do any charging of the battery, so you will need a Traxxas EZ-Peak charger to charge it (you can find them on Traxxas' website). At present, there's no way to know the battery's charge level except by measuring it with a multimeter or the BLDC Tool as it runs, but we could think about adding a low voltage LED or seven-segment LCD display (to show the voltage) to the next iteration of the board. The LIPO protection module and green connectors are currently unused and are a legacy from previous F1/10 car iterations which used the Teensy microcontroller as a motor driver.

Installing the Body Standoffs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Begin by installing four 45mm M3 standoffs into the holes of the main car body pictured below. Secure the standoffs to the bottom of the car using 10mm M3 screws, and use either nosepliers or a hex driver to hold the standoff in place while you turn the screw on the other side. Pay attention to where you install the standoffs since there are several mounting holes on the car's base. See the picture below for clarification.

Next, install two threaded 14mm M3 standoffs into the front holes in the car base, using the pliers or hex driver to thread the standoffs into the holes. Don't mount the chassis to the car yet since we still need to mount the Jetson and power board to the chassis.

.. image:: img/InstallingStandOffs.png
.. image:: img/InstallingStandOffs2.png

Mounting the FOCbox to the Chassis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Feed the three motor wires for the FOCbox through the rectangular slot in the chassis as shown below.

.. image:: img/FOCBoxToChassis.png

Alternate method of securing the FOCbox if screws don’t fit​ : Some FOCboxes (the more recently-made ones) have smaller screw holes that won’t fit M3-size screws. If this is the case for your FOCbox, you can use a few pieces of double-sided tape to secure the FOCbox instead.

.. image:: img/AlternateFOCbox.png


Installing the Chassis Standoffs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Mount eight 35mm M3 standoffs to the ​ glossy side​ of the black laser-cut chassis in the positions shown below. Thread 10mm M3 screws through the drill holes and screw them into the standoffs to secure them. (​ Important​ : If you do not mount the standoffs to the glossy side, the power board won't fit since the screw holes will be misaligned.)

Note there are several drill holes in the chassis, so make sure you’re using the right ones. In the image below, the 4 screws on the left(which is the back of the car) will eventually hold the power board, so that’s a good way to see if they’re properly placed. The 4 screws on the right (2 on each side of the oval cutout) will eventually hold the Orbitty carrier attached to the Jetson.

.. image:: img/chasisstandoff1.png

Mount two more (19mm M3) standoffs to the front part of the chassis for the LIDAR, and secure with 10mm screws. The top part of the chassis should look like the picture below.



Detaching the Jetson from the Development Board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
When you purchase a Jetson, it is attached to a development board. In order to use it on the car, you will need to unscrew the Jetson and its Wi-Fi antenna from the development board.

Before you remove the antenna, you will need to remove the bottom plate from the development board. Remove the four screws marked below and lift the development board away from the plate.

.. image:: img/jetson1.png

Next, remove the Wi-Fi antenna by unscrewing the two screws marked below. Keep the screws in a safe place, as you’ll use them in a bit to attach the antennas to standoffs.

.. image:: img/jetson2.png

Remove the two brass-colored nuts holding the antennas to the L-shaped bracket, and then remove the two antennas from the bracket. It helps to use two pairs of pliers: one to hold the rear nuts in place and another to unscrew the nuts on the end with the antenna connectors.

.. image:: img/jetson3.png

Using a Phillips screwdriver, thread the two screws you saved earlier completely into the bracket as pictured below. Attach two standoffs to the opposite ends of the screws and hand-tighten them until they won’t turn anymore. Use the pliers to tighten the standoffs more while you hold the head of the screw in place using the screwdriver. Once you’ve done these steps, place the antennas and washers back into the bracket, and tighten the brass nuts onto the threaded connectors again.

.. image:: img/jetson4.png

Unplug the Jetson’s fan and remove the Jetson from the development board by using a T3 Torx screwdriver to unscrew the Jetson (the large silver heat sink), and then pull up gently to detach it from the development board. Keep the Jetson in a safe place while you attach the antennas to the power board.

.. image:: img/jetson5.png

Mounting the Wi-Fi Antennas to the Power Board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Attach the two standoffs for the Wi-Fi antenna bracket to the power board, making sure the antennas, when installed and extended, lie ​ over ​ the board and now away from it. Install the two black antennas onto the threaded connectors if they aren’t already on.

.. image:: img/wifi1.png

Attach the two wires for the Jetson Wi-Fi antenna to the two gold-colored connectors near the fan connector on the heat sink (the order of the wires doesn’t matter). This can be a little tricky, so you might want to use a flathead screwdriver to ensure the connections are tight. ​ Don’t press too hard​ , however as you can easily damage the connectors if you use excessive force!

.. image:: img/wifi2.png

Mounting the Power Board to the Chassis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Screw the power board onto its chassis standoffs using 10mm M3 screws. The screw positions are indicated with arrows below.

.. image:: img/powerboard1.png


Attaching the Orbitty to the Jetson
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Attach the Orbitty to the Jetson by connecting the two long black ports and connect the Jetson’s fan to the Orbitty’s fan connector as shown in the pictures below.

.. image:: img/orbittyjetson.png

Connecting the Jetson and Power Board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Cut two 8-inch pieces of wire of different colors (preferably red and black), and strip both ends to a short length (1/8 inch). Locate the green terminal block on the Jetson and attach one end of one wire to the + ​ Vin​ terminal, and the other end to one of the green 1 ​ 2V​ terminals on the power board. (Any of the 12 volt terminals are acceptable. To attach, insert the stripped end into the terminal and screw the little screw tight with a small flathead screwdriver.) Attach the other stripped wire to the G ​ ND​ terminal on the Jetson and to the G ​ ND​ terminal on the corresponding terminal block on the power board.

.. image:: img/orbittyjetson.png

Mounting the Jetson to the Chassis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Your kit comes with four white plastic standoffs; place these between the Jetson PCB and heatsink (see picture) ​ before ​ threading the screws through. Otherwise, you risk bending the Orbitty while screwing it in. Use 20mm screws to secure the Jetson to its chassis standoffs.

.. image:: img/jetsonchassis.png

Ensure that when you mount the Jetson that the wires for neither the Wi-Fi antenna nor the Jetson's power connections get pinched. It might help to tuck both sets of wires underneath the power board. (Don't tuck them underneath the Jetson because they might restrict airflow or obstruct the fan's blades.) Your configuration should now look something like this:

.. image:: img/jetsonchassis2.png


Mounting the Chassis to the Car Underbody
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Place the chassis onto the five standoffs on the car base and align the chassis’ drill holes with the car’s base standoffs you attached earlier as shown below. Use six 10mm M3 screws to secure the chassis to the standoffs.

.. image:: img/chassisunderbody.png

Mounting the LIDAR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The LIDAR should have two cords: one for power and another for either Ethernet or USB. Cut the end of the power cord, leaving 1-2 feet of cable. Strip the end, cut away all wires except for the blue and brown ones, and strip those two wires to 1/8 inch as shown below.

.. image:: img/lidar1.png

Attach the LIDAR to the tree-shaped base using two (10LX) or four (30LX) screws, such that the wires protruding from the LIDAR go towards the back of the car. (For the 30LX, the two LEDs at the top of the LIDAR should face the front of the car.) Then mount the C-shaped brackets to the black tree-shaped base as shown in the pictures below. ​ Note: if the black mount does not fit in the holes of the C brackets, sand the inserts until they do.

.. image:: img/lidar2.png

Mount the LIDAR to the car by fitting the two thick protruding parts of the mounting brackets into the holes. The open part of the "C" in the brackets should face forward as shown below.

.. image:: img/lidar3.png

If you completed the previous steps correctly, two of the holes on the narrow end of the LIDAR base should match up with the two 19mm standoffs you mounted earlier. Wind the power and USB cords of the LIDAR around its base so that there is just enough available to plug into the power board and Jetson, with a little bit of slack so that it is not too tight. Tuck both cords under the LIDAR mounting plate between the two silverstandoffs, and secure the LIDAR base to the standoffs on the chassis using two 10mm M3 screws.

.. image:: img/lidar4.png

Mounting the USB Hubs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Attack two pieces of double-sided tape to a USB hub. Place the hub onto the empty space of the chassis directly below the Jetson and to the right of the power board as shown. If your hub has power buttons, make sure all of them are turned on.

.. image:: img/usb1.png

If you need more USB ports (required if your LiDAR uses USB), you can stack a second hub onto the top of the first. Again, use double-sided tape to secure the second hub and make sure all power buttons are on.

Plug the first USB hub into the Orbitty board. If you’re using a second hub, use the white micro USB adapter that comes with the Jetson to plug in the second hub.

.. image:: img/usb2.png


Connecting the LIDAR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Attach the LIDAR's power cable to a free 12V terminal block on the power board. The brown wire should go to the 12V terminal, and the blue wire should go to the corresponding GND terminal. The side of the LIDAR has a pinout as well if you forget.

.. image:: img/connect_lidar1.png

If the LIDAR has an Ethernet cable, attach it to the Ethernet port on the Jetson. If it has a USB cable, plug it into the USB hub. Route any excess cables behind the USB hubs as shown.

.. image:: img/connect_lidar2.jpg

Connecting the FOCbox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Pass the 3 round FOCbox wires through the rectangular slot in the plastic chassis, then connect the 3 circular bullet connectors to the three motor wires. (The order in which you connect the wires kinda doesn’t matter (electrically speaking). If you run the car and it goes backwards when it should go forwards, you can swap any two of the three wires.) Connect the 3-wire servo ribbon cable as well, making sure the colors match up.

.. image:: img/foc1.jpg
.. image:: img/foc2.jpg

If your micro USB cable is thin enough, thread it through the rectangular wire slot and around the FOCbox to the USB connector as shown below, or route it around the rear end of the chassis if it isn’t. Plug the cable into the FOCbox’s USB connector and into a free port on your USB hub. Tie the USB cable up using a cable tie, and tuck all of the wires underneath the chassis. You can also use this time to plug in the LIDAR (if it is USB), the external Wi-FI adapter, and the receiver for the F710 gamepad.

.. image:: img/foc3.jpg
.. image:: img/foc4.jpg
.. image:: img/foc5.jpg


Connecting the Car to the Battery
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Connect the battery to the FOCbox using the battery connector. ​Make sure that red is aligned with red and black is aligned with black - otherwise things will get hot and dicey. ​Then connect the FOCbox to the power board using the white port shown in the picture below.

.. image:: img/foc6.jpg

At this point, your car should be assembled, the Jetson lights should flash when you flip the power switch, and the car is ready to run!

Note: At present, there's no way to know the battery's charge level except by measuring it with a multimeter as it runs. You might consider adding a low voltage LED or seven-segment LCD display (to show the voltage).


