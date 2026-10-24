.. _doc_offroad_autonomy:

3. Autonomy Elements
====================

This step builds three modules: the Jetson AGX Orin on its custom base, the baseplate that carries the Jetson, VESC and power bank, and the camera mount. The :ref:`final assembly <doc_offroad_together>`  joins them to the enclosure.

Jetson AGX Orin
---------------

**1. Remove the original base.**

Take the Jetson off its original base by removing the four circled screws.

.. figure:: img/orin_base_original_screws.jpg
   :align: center

   Underside of the Jetson with the original screws (circled).

**2. Prepare the custom base.**

Use a soldering iron to insert four M3 heat-set inserts in the custom 3D-printed base as shown in the picture (circled). These will be used to mount the Jetson to the baseplate.

.. figure:: img/orin_base_custom_inserts.jpg
   :align: center

   Custom base with inserts (circled).

**3. Install the ZED capture card.**

Fit the capture card to the Jetson. Refer to the official `Stereolabs instructions <https://docs.stereolabs.com/docs/products/embedded/zed-link-capture-card/gmsl2/zed-link-duo/jetson-orin-agx-devkit-setup>`_.

.. figure:: img/orin_capture_card.jpg
   :align: center

   Capture card installed (boxed).

**4. Connect the camera cable.**

Re-use the original screws to mount the Jetson on the custom base and connect the camera cable to the capture card.

.. figure:: img/orin_with_base_custom_cable.jpg
   :align: center

   Jetson on its custom base with the camera cable attached.

**5. Plug in the controller receiver.**

Use a right-angle USB-C adapter, plug the controller's USB receiver into the Jetson. You can use Velcro™ dots as shown to avoid the receiver coming loose due to vibrations.

.. figure:: img/orin_controller_receiver.jpg
   :align: center

   Controller receiver plugged in.

Baseplate
---------

**6. Start with the bare baseplate.**

The baseplate is the acrylic plate that slides into the enclosure.

.. figure:: img/baseplate.jpg
   :align: center

   Baseplate.

**7. Mount the VESC.**

Place the VESC on the plate in the orientation shown, passing the motor wires through the slot. Attach the VESC from below using 2x M5 screws.

.. warning::
   Ignore the cut power wires to the VESC. DO NOT cut the wires on your VESC at any point.

.. figure:: img/baseplate_vesc_placement.jpg
   :align: center

   VESC placed on the baseplate.

.. figure:: img/baseplate_vesc_screws.jpg
   :align: center

   VESC screws (circled).

**8. Attach the Standoffs.**

Attach four standoffs around the VESC in the positions shown and attach them from below using 4x M3x12mm screws with washers.

.. figure:: img/baseplate_power_standoffs_top.jpg
   :align: center

   Standoffs (circled).

.. figure:: img/baseplate_power_standoffs_screws.jpg
   :align: center

   Screws that attach the standoffs (circled).

**9. Mount the Jetson.**

Attach the Jetson and its base to the plate beside the VESC in the orientation shown. Use 4x M3x12mm screws with washers.

.. figure:: img/baseplate_orin_screws.jpg
   :align: center

   Jetson base screws (circled).

.. figure:: img/baseplate_orin_top.jpg
   :align: center

   Jetson and VESC on the baseplate.

**10. Mount the power bank holder.**

Screw the power bank holder to the plate and thread the Velcro strap through it. The power bank is inserted in the final assembly.

.. figure:: img/baseplate_powerbank_mount.jpg
   :align: center

   Power bank holder (screws circled).

Camera mount
------------

**11. Prepare the camera housing.**

Using a soldering iron, insert two heat-set inserts (one on each side) into the side of the printed camera housing in the location shown.

.. figure:: img/zed_mount_insert.jpg
   :align: center

   Insert in the camera housing (circled).

**12. Install the camera.**

Slide the ZED into the housing.

.. figure:: img/zed_mount_with_zed.jpg
   :align: center

   Camera in the housing.

**13. Secure the camera.**

Fasten the camera with two M3x12mm screws at the bottom of the housing. The middle hole is not used.

.. figure:: img/zed_mount_bottom_screws.jpg
   :align: center

   Bottom screws (circled).

**14. Attach the camera cable.**

Attach the right-angle extension cable to the ZED camera.

.. figure:: img/zed_mount_with_zed_cable.jpg
   :align: center

   Camera cable connected.

**15. Attach the housing to the bracket.**

Fit the housing into the bracket. Two M5x20mm screws (one on each side) are inserted to act as hinges. Use nyloc nuts on the inside to secure them. These should be tight enough to remove side-to-side play while still allowing the camera to be tilted up or down without excessive force.

Screw two M3x12mm screws with washers (one on each side) into the heat-set inserts. These are used to lock the camera tilt at a chosen angle. Tighten them fully once you have chosen the angle.

.. figure:: img/zed_mount_bracket_side_hinge_screw.jpg
   :align: center

   Hinge screw at the side (circled).

.. figure:: img/zed_mount_bracket_back_hinge_nuts.jpg
   :align: center

   Nyloc nuts at the back (circled).

.. figure:: img/zed_mount_bracket_side_adjustment_screw.jpg
   :align: center

   Tilt adjustment screw (circled).

**16. Attach the bracket to the enclosure.**

Pass the camera cable through the slots in the mounting bracket and the enclosure, then attach the camera assembly to the front of the enclosure using two M5x16mm screws, secure them from the inside using two nyloc nuts (not shown).

.. figure:: img/zed_mount_bracket_box.jpg
   :align: center

   Bracket on the front of the enclosure (screws circled).

.. todo::
   Confirm part names and the order of the VESC standoff and screw steps (steps 7 and 8 are a first guess from the photos), the camera tilt angle to use, and whether the camera mount needs a printable file link.
