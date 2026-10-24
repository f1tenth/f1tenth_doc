.. _doc_offroad_together:

4. Putting It All Together
==========================

This step joins the enclosure, baseplate and chassis, then connects power and signal cables. Connect the battery last.

Mount the enclosure
-------------------

**1. Fasten the enclosure to the chassis.**

Lower the enclosure onto the standoffs and screw it down using three M3x12mm screws in the circled positions.

.. figure:: img/car_with_box.jpg
   :align: center

   Enclosure on the chassis (screws circled).

Baseplate
---------

**2. Insert the baseplate.**

Lower the baseplate into the enclosure with the VESC at the back. Angle it as shown to clear the camera cable, passing the motor wires from the VESC through the slot, then lower it fully into the enclosure.

.. figure:: img/car_insert_baseplate.jpg
   :align: center

   Baseplate going into the enclosure.

**3. Screw the baseplate down.**

Fasten the baseplate with the four M5x10mm screws (circled).

.. figure:: img/car_box_screws.jpg
   :align: center

   Baseplate screws (circled).

Motor and steering cables
-------------------------

**4. Connect the motor.**

Pass the VESC motor wires through the bottom opening and connect them to the motor wires.

.. figure:: img/car_connect_motor.jpg
   :align: center

   Motor wire connections.

**5. Route the servo cable.**

Pass the steering servo cable up into the enclosure.

.. figure:: img/car_servo_wire.jpg
   :align: center

   Servo cable in the enclosure.

**6. Connect the servo to the VESC.**

Plug the servo cable into the PPM port on the VESC.

.. figure:: img/car_servo_connect.jpg
   :align: center

   Servo cable at the VESC.

.. figure:: img/car_vesc_ppm_port.jpg
   :align: center

   PPM port (circled).

**7. Connect the VESC to the Jetson.**

Using a USB-isolator, connect the VESC to the Jetson with its USB cable (circled).

.. warning::
   The USB-isolator is recommended to avoid damaging the Jetson in case the battery is accidentally connected to the VESC with reversed polarity.

.. figure:: img/car_connect_vesc_orin.jpg
   :align: center

   VESC to Jetson USB cable (circled).

**8. Connect the camera.**

Plug the camera cable into the capture card cable.

.. figure:: img/car_connect_camera.jpg
   :align: center

   Camera cable connected.

**9. (Optional) Connect the router.**

A small router can be connected to the Jetson and placed inside the enclosure to provide convenient wifi-access.

.. figure:: img/car_router.jpg
   :align: center

   Router and Ethernet cable (circled).

**10. (Optional) Connect a DisplayPort adapter.**

If you will be using a remote-desktop application to access the Jetson, plug a headless DisplayPort adapter into it. This lets the Jetson boot a desktop without a monitor.

.. figure:: img/car_orin_displayport.jpg
   :align: center

   DisplayPort adapter (circled).

Power
-----

**11. Install the power bank.**

Attach a right-angle USB-C connector to the powerbank as shown. Make sure to use the 140W-rated output port (OUT1). Lower the powerbank onto its holder and secure it with the strap. Pass its USB cable out through the side opening.

.. figure:: img/car_powerbank_connector.jpg
   :align: center

   Power bank connector.

.. figure:: img/car_powerbank_insert.jpg
   :align: center

   Power bank in the holder.

**12. Power the Jetson.**

Route the power cable from the power bank along the outside of the enclosure and connect it to the Jetson. This gives easier access to connect/disconnect the powerbank, charge the powerbank, power the Jetson using externally during development, etc.

.. figure:: img/car_powerbank_cable.jpg
   :align: center

   Cable through the side opening.

.. figure:: img/car_powerbank_connect_orin.jpg
   :align: center

   Cable at the Jetson (circled).

**13. Connect the battery.**

Slide the battery into the chassis as shown. Connect the battery to the XT90 connector of the VESC. Connect the battery monitor to the balance-lead extension cable and attach it to the enclosure using Velcro™ dots. The extra length of wires can be tucked between the battery and the enclosure.

.. warning::
   Ignore the cut wires and WAGO™ connectors on the VESC power wires. Your stock VESC will come with a XT90 connector already attached.

.. figure:: img/car_battery_insert.jpg
   :align: center

   Battery in the chassis.

.. figure:: img/car_battery_connect.jpg
   :align: center

   Battery connected.

.. figure:: img/car_battery_monitor.jpg
   :align: center

   Battery monitor on the side of the enclosure.

.. figure:: img/car_battery_wires.jpg
   :align: center

   Battery wires tucked inside.

First power-on check
--------------------

**14. Check the car.**

#. Put the car on a stand so the wheels are off the ground.
#. Connect the battery and confirm that the Jetson boots.
#. Confirm that the ZED camera and the VESC are detected.
#. Disconnect the battery when you are finished.

Next steps
----------

**15. Install the software.**

Continue with :ref:`installing the Autoware stack <doc_offroad_autoware>`.

.. todo::
   Confirm the cable order, what each cable does (the power bank powers the Jetson, the battery powers the VESC and motor), and the expected device names for the power-on check.
