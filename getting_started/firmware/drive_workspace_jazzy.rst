.. _doc_drive_workspace_jazzy:

1. Ubuntu 24.04 - ROS 2 Jazzy
===============================
**Equipment Required:**
	* Fully built RoboRacer vehicle
	* Pit/Host computer OR
	* External monitor/display, HDMI cable, keyboard, mouse

**Approximate Time Investment:** 1.5 - 2 hours

.. note:: This page is the complete driver stack procedure for a Jetson running **JetPack 7 (L4T R39), Ubuntu 24.04 "Noble"**. If your car runs Ubuntu 20.04 or 22.04, follow :ref:`2. Ubuntu 22.04 - ROS 2 Humble <doc_drive_workspace_humble>` instead. Check with ``lsb_release -a``.

Everything below runs **on the Jetson**, either over SSH from the Pit laptop or with a monitor, keyboard, and mouse attached directly.

.. warning:: ``f1tenth_system`` has no Jazzy branch, so step 4 clones ``humble-devel`` and patches it. See :ref:`Why the Jazzy path needs an extra patch <doc_drive_workspace>` for the background, and consider whether a Humble container suits your car better.

1. Installing ROS 2 Jazzy
---------------------------
Noble is not in ROS's default apt sources, so the ROS repository has to be added first. Skipping this produces ``E: Unable to locate package ros-jazzy-ros-base``.

.. code-block:: bash

    sudo apt update && sudo apt install -y curl gnupg software-properties-common
    sudo add-apt-repository -y universe

    sudo curl -fsSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list

    sudo apt update

.. tip:: Read the ``apt update`` output and check the ``packages.ros.org`` line for errors. A bad signing key surfaces there, not later at install time.

Then install the base distribution and the build tools:

.. code-block:: bash

    sudo apt install -y ros-jazzy-ros-base ros-dev-tools
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ros2 --help

``ros-base`` is the bare-bones installation: communication libraries, message packages, and command-line tools, with no GUI. Substitute ``ros-jazzy-desktop`` if you want RViz running on the car itself rather than on the Pit laptop.

.. note:: The JetPack image already sets a UTF-8 locale, so the ``locale-gen`` step in the official ROS instructions is not needed here.

2. Installing rosdep
----------------------
``rosdep`` resolves the driver stack's dependencies to system packages.

.. code-block:: bash

    sudo apt install -y python3-rosdep
    sudo rosdep init      # skip this if it reports that the file already exists
    rosdep update

Confirm that ``Add distro "jazzy"`` appears in the output.

.. note:: A ``pkg_resources is deprecated`` warning is cosmetic noise from setuptools on Python 3.12 and can be ignored.

3. udev Rules Setup
----------------------
When you connect the VESC to the Jetson, the operating system assigns it a device name of the form ``/dev/ttyACMx``, where ``x`` depends on the order in which devices were plugged in. This can change between reboots, which is a problem because the car's configuration needs to know which device name belongs to which piece of hardware.

udev solves this by assigning a persistent symbolic name based on the device's USB vendor and product IDs, through configuration files ("rules") in ``/etc/udev/rules.d``.

.. note:: **These rules are not shipped with the stack.** The ``f1tenth_system`` repository contains no ``.rules`` files — you create them by hand, once per car. They live in ``/etc/udev/rules.d/``, outside the workspace, so they survive workspace rebuilds and deletions.

.. tip:: These rule files have to be created **as root** with a text editor. If you don't already have a terminal editor you're comfortable with, install ``nano`` and use it to open and edit each file — it's the easiest option:

    .. code-block:: bash

        sudo apt install nano

    Then open any file below with ``sudo nano <filename>`` (for example ``sudo nano /etc/udev/rules.d/99-vesc.rules``), paste the rule, and save with ``Ctrl+O`` then exit with ``Ctrl+X``.

The VESC rule
^^^^^^^^^^^^^^^
Open ``/etc/udev/rules.d/99-vesc.rules`` and copy in the following rule, exactly as it appears below and on a single line:

.. code-block:: bash

    KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"

The Hokuyo rule
^^^^^^^^^^^^^^^^^
**Skip this rule if you are not using a Hokuyo USB LiDAR** — for example, if you have an ethernet SICK LiDAR. Otherwise open ``/etc/udev/rules.d/99-hokuyo.rules`` and copy in:

.. code-block:: bash

    KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", SYMLINK+="sensors/hokuyo"

Why the joypad needs no rule
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The older RoboRacer documentation includes a ``99-joypad-f710.rules`` file. **On Jazzy you should skip it.** It is a leftover from the Foxy-era stack, when the ``joy`` node read ``/dev/input/jsX`` directly and needed a stable path.

Jazzy's ``joy`` node uses SDL2 instead. The stack's ``joy_teleop.yaml`` leaves ``device_name`` commented out and uses ``device_id: 0`` — an SDL enumeration index, not a filesystem path. SDL finds the controller regardless of any symlink. This is true for the Logitech F710, the 8BitDo, and any other pad.

Applying the rules
^^^^^^^^^^^^^^^^^^^^
Reload the rules, trigger them, and add yourself to the ``dialout`` group so that you can open the serial device:

.. code-block:: bash

    sudo udevadm control --reload-rules
    sudo udevadm trigger --action=add
    sudo usermod -aG dialout $USER

.. warning:: ``--action=add`` is required. A bare ``udevadm trigger`` fires a ``change`` event, but the rules above match ``ACTION=="add"`` — so it silently does nothing and the symlink never appears. Physically unplugging and replugging the VESC has the same effect as ``--action=add``.

Log out and back in (or reboot) for the ``dialout`` group membership to take effect, then verify:

.. code-block:: bash

    lsusb | grep STMicro      # expect 0483:5740
    ls -l /dev/sensors/       # expect vesc -> ../ttyACM0
    groups | grep dialout

If you want to add additional devices and don't know their vendor or product IDs, you can use:

.. code-block:: bash

    sudo udevadm info --name=<your_device_name> --attribute-walk

replacing ``<your_device_name>`` with the name of your device (e.g. ``ttyACM0`` if that's what the OS assigned it — the Unix utility ``dmesg`` can help you find that). The topmost entry will be the entry for your device; lower entries are for the device's parents.

4. Installing the RoboRacer Driver Stack
------------------------------------------
Install the apt dependencies first — all of them are released for Jazzy:

.. code-block:: bash

    sudo apt install -y ros-jazzy-sick-scan-xd ros-jazzy-asio-cmake-module \
                        ros-jazzy-serial-driver ros-jazzy-ackermann-msgs \
                        ros-jazzy-joy ros-jazzy-joy-teleop ros-jazzy-rosbridge-suite \
                        ros-jazzy-diagnostic-updater python3-colcon-common-extensions

Then create the workspace and clone the stack. Note that you still clone ``humble-devel``, because there is no Jazzy branch:

.. code-block:: bash

    mkdir -p ~/f1tenth_ws/src && cd ~/f1tenth_ws/src
    git clone --branch humble-devel https://github.com/f1tenth/f1tenth_system.git
    cd f1tenth_system
    git submodule update --init --recursive --remote

Applying the Jazzy patch
^^^^^^^^^^^^^^^^^^^^^^^^^^
This is the one change that genuinely matters. The ``vesc_driver``, ``vesc_ackermann`` and ``ackermann_mux`` packages pin ``CMAKE_CXX_STANDARD 14``, but Jazzy's ``rclcpp``, ``serial_driver`` and ``diagnostic_updater`` all require **C++17**, so the build fails without this change. The same packages also declare ``cmake_minimum_required(VERSION 3.5)``, which CMake 3.28 on Noble deprecates loudly and CMake 4 rejects outright.

.. code-block:: bash

    cd ~/f1tenth_ws/src/f1tenth_system
    find . -name CMakeLists.txt -exec sed -i \
      -e 's/cmake_minimum_required(VERSION 3\.5)/cmake_minimum_required(VERSION 3.16)/' \
      -e 's/set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 17)/' {} +

    grep -rn "CMAKE_CXX_STANDARD\|cmake_minimum_required" --include=CMakeLists.txt .

Every result of that last command should now read ``3.16`` and ``17``.

Building the workspace
^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: bash

    cd ~/f1tenth_ws
    rosdep install --from-paths src -i -y --rosdistro jazzy
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash

Expect 11 packages and a couple of minutes on the Jetson.

.. warning:: Do not use ``--symlink-install``. ``f1tenth_stack`` is an ``ament_python`` package with a legacy ``setup.cfg`` that setuptools 68+ on Noble cannot process in editable mode. A plain ``colcon build`` works correctly.

Two messages during the build are expected and harmless:

* ``rosdep install`` may report ``vesc`` or ``ackermann_mux`` as unresolvable keys. They are built from source, not from apt.
* ``vesc_driver`` and ``vesc_ackermann`` emit a notice about ``ament_auto_package`` header install destinations changing in ROS 2 Kilted. This is forward-compatibility information only.

You can find more details on how the drivers are set up in the README of the `f1tenth_system repo <https://github.com/f1tenth/f1tenth_system>`_.

5. Setting Up Your LiDAR
--------------------------
The driver stack supports different LiDARs. Follow the option that matches your hardware.

Option 1 - Hokuyo LiDAR
^^^^^^^^^^^^^^^^^^^^^^^^^^
If you are using a **USB Hokuyo** (e.g. the 30LX), no extra setup is needed — it is referenced through the udev rule you created in step 3.

If you have a **Hokuyo 10LX** that connects over **ethernet**, you'll need to configure the ``eth0`` network. From the factory, the 10LX is assigned the IP ``192.168.0.10`` (note that the LiDAR is on subnet 0). Assign the Jetson's wired port:

    * IP address ``192.168.0.15``
    * Subnet mask ``255.255.255.0``
    * Gateway ``192.168.0.10``

Name the connection ``Hokuyo`` and save it. When you plug in the 10LX, make sure the ``Hokuyo`` connection is selected. If everything is configured properly, you should now be able to ping ``192.168.0.10``.

.. image:: img/hokuyo1.gif
    :align: center
    :width: 200px

Option 2 - SICK ethernet LiDAR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The SICK TiM connects over ethernet on its own isolated subnet. These units ship with a factory IP on the ``192.168.0.x`` subnet — commonly ``192.168.0.1`` — and the Jetson takes a different address on that same subnet, ``192.168.0.15``. The ``ros-jazzy-sick-scan-xd`` driver was installed in step 4.

Network setup
"""""""""""""""
Configure the Jetson's wired connection so that it is on the **same subnet** as the LiDAR. No gateway is needed for a direct point-to-point link.

*Using the desktop GUI:* open the network settings, edit the **Wired** connection's **IPv4** tab, set the method to **Manual**, and add an address:

    * Address: ``192.168.0.15``
    * Netmask: ``255.255.255.0``
    * Gateway: leave blank

The address just needs to be on the ``192.168.0.x`` subnet and different from the LiDAR's address. Click **Apply**.

.. figure:: img/sick/ipv4_setup.png
    :align: center

    Setting a static IPv4 address for the wired connection to the SICK LiDAR.

*Over SSH, with no desktop:* identify the ethernet interface first — on the Jetson Orin this is ``enP8p1s0``. The flags ``UP,LOWER_UP`` confirm that a cable is connected.

.. code-block:: bash

    ip a | grep -A1 "enP"

Then create the connection with ``nmcli``, substituting your own interface name. ``sudo`` is required; without it ``nmcli`` reports *Insufficient privileges*:

.. code-block:: bash

    sudo nmcli con add type ethernet ifname enP8p1s0 con-name sick \
         ipv4.method manual ipv4.addresses 192.168.0.15/24
    sudo nmcli con up sick

Confirming that the LiDAR responds
""""""""""""""""""""""""""""""""""""
Check the Jetson's address, then check that the LiDAR answers on both of its service ports — ``2111`` is configuration and ``2112`` is scan data:

.. code-block:: bash

    ip a show enP8p1s0 | grep inet     # expect 192.168.0.15/24

    sudo apt install -y nmap
    sudo nmap -p 2111,2112 192.168.0.1 # expect both open

If the LiDAR is **not** at ``192.168.0.1`` — a unit reconfigured by a previous user, for example — scan the subnet and look for a MAC address that resolves to *Sick AG*:

.. code-block:: bash

    sudo nmap -sn 192.168.0.0/24

Then substitute that address wherever ``192.168.0.1`` appears below.

.. note:: **Two harmless surprises.** ``ping`` may fail with ``socket: Operation not permitted``. That is a missing ``cap_net_raw`` capability on the JetPack ``ping`` binary and is unrelated to the LiDAR — fix it with ``sudo setcap cap_net_raw+p /usr/bin/ping``, or just use ``nmap`` instead. Separately, TiM units run no web server, so ``curl http://192.168.0.1`` returning nothing is expected.

Pointing the launch files at the device
"""""""""""""""""""""""""""""""""""""""""
Two files in ``f1tenth_stack/launch/`` need your LiDAR's address.

Open ``sick_tim_5xx.launch`` and set the ``hostname`` argument to your LiDAR's IP address:

.. code-block:: xml

    <arg name="hostname" default="192.168.0.1"/>

.. figure:: img/sick/sick_tim_5xx_launch.png
    :align: center

    Setting the LiDAR IP in ``sick_tim_5xx.launch``.

Then open ``sick_bringup_launch.py`` and set the ``arguments`` entry of the ``sick_node`` so that it points at the absolute path of your ``sick_tim_5xx.launch`` file:

.. code-block:: python

    arguments=["/home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch/sick_tim_5xx.launch"]

.. figure:: img/sick/sick_bringup_launch.png
    :align: center

    Pointing ``sick_bringup_launch.py`` at the SICK launch file.

Two things are worth knowing about these files:

* ``sick_tim_5xx.launch`` is a **SICK-internal configuration file**, not a ROS 2 launch file. ``sick_generic_caller`` parses it directly, which is why it uses ROS 1 XML syntax. Leave the syntax as it is.
* It sets ``scanner_type = sick_tim_5xx``, which is correct for TiM5xx units. A different model (TiM7xx, LMS1xx) requires changing this value, or the node will connect successfully and then fail during scan configuration.

Testing the LiDAR on its own
""""""""""""""""""""""""""""""
Testing the LiDAR before full bringup isolates any network or model mismatch from the rest of the stack. Rebuild first, so that the edited launch files are copied into ``install/``:

.. code-block:: bash

    cd ~/f1tenth_ws && colcon build --packages-select f1tenth_stack
    source install/setup.bash

    ros2 run sick_scan_xd sick_generic_caller \
      ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch/sick_tim_5xx.launch

Success looks like ``Setup completed, sick_scan_xd is up and running``, followed shortly by ``Software PLL is ready and locked now!``. A handful of dropped packets before the PLL locks is normal.

In a second terminal, source ROS 2 and your workspace, then check the scan topic:

.. code-block:: bash

    source /opt/ros/jazzy/setup.bash && source ~/f1tenth_ws/install/setup.bash
    ros2 topic hz /scan
    ros2 topic echo /scan --once

A TiM5xx publishes at **15 Hz** across a 270° field of view (−135° to +135°) at 0.33° angular resolution. Anything else means the wrong ``scanner_type``.

.. tip:: Use ``--once``, not ``| head -5``. Piping into ``head`` closes the pipe, and ``ros2 topic echo`` responds with an alarming ``BrokenPipeError`` traceback that means nothing at all.

6. Configuring the Joypad
---------------------------
The joypad mapping lives in ``f1tenth_stack/config/joy_teleop.yaml``. Two settings matter before the first drive: which buttons act as the **deadman switches**, and the **speed scale**.

Logitech F710
^^^^^^^^^^^^^^^
The F710 is the reference joypad for RoboRacer, and the values shipped in ``joy_teleop.yaml`` already match it — axis 1 for speed, axis 2 for steering, button 4 (LB) as the manual deadman and button 5 (RB) as the autonomous deadman. No mapping changes are needed.

Make sure the **D/X** switch on the back is set to **D**, and that the mode light on the front is **not** constantly lit. If it is, press the mode button once.

8BitDo Ultimate 2C
^^^^^^^^^^^^^^^^^^^^
SDL2 has no gamepad profile for the 8BitDo Ultimate 2C, so it reports raw indices rather than the standard Xbox-style layout that the stack's defaults assume. The axes happen to land where the stack already expects them, so **only the two deadman buttons need changing**:

.. list-table::
   :header-rows: 1
   :widths: 40 20 40

   * - Control
     - 8BitDo index
     - Default in repo (F710)
   * - Speed (left stick, vertical)
     - axis **1**
     - 1 — no change
   * - Steering (right stick, horizontal)
     - axis **2**
     - 2 — no change
   * - Manual deadman (L)
     - button **6**
     - 4 — *must change*
   * - Autonomous deadman (R)
     - button **8**
     - 5 — *must change*

Apply both changes, and drop the speed scale to ``1.0`` for initial testing:

.. code-block:: bash

    cd ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config

    sed -i 's/deadman_buttons: \[4\]/deadman_buttons: [6]/; \
            s/deadman_buttons: \[5\]/deadman_buttons: [8]/; \
            s/scale: 5.0/scale: 1.0/' joy_teleop.yaml

    grep -n "deadman_buttons\|scale" joy_teleop.yaml

Leave the ``default:`` block alone — its scales are ``0.0``, so its indices are irrelevant.

Any other joypad
^^^^^^^^^^^^^^^^^^
For any pad not listed above, read the indices off the live topic. In one terminal:

.. code-block:: bash

    ros2 run joy joy_node

And in a second:

.. code-block:: bash

    ros2 topic echo /joy

Push each stick to its extreme and hold each bumper, noting which entry of the array changes. The index of that entry is the axis or button id to put into ``joy_teleop.yaml`` under ``human_control``. ``joy_node`` also prints the controller name on startup — confirm that it opened the device you expect.

.. warning:: **Count from index 0.** The *seventh entry printed* is index 6. Off-by-one here is the single most common error in this procedure. Analog triggers also rest at ``1.0`` when uncompressed — those are L2/R2, not the bumpers.

Rebuild after editing any config or launch file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
``f1tenth_stack`` is an ``ament_python`` package, so files in ``config/`` and ``launch/`` are **copied** into ``install/``, not symlinked. Editing the source file alone changes nothing at runtime.

.. code-block:: bash

    cd ~/f1tenth_ws && colcon build --packages-select f1tenth_stack
    source install/setup.bash

    # confirm the edit actually reached the installed copy
    grep -n "deadman_buttons" \
      install/f1tenth_stack/share/f1tenth_stack/config/joy_teleop.yaml

7. Launching the Driver Stack
-------------------------------
Source the ROS 2 underlay and your workspace's overlay, then launch the bringup. Use ``sick_bringup_launch.py`` if you have a SICK LiDAR and ``bringup_launch.py`` otherwise:

.. code-block:: bash

    source /opt/ros/jazzy/setup.bash
    cd ~/f1tenth_ws && source install/setup.bash

    ros2 launch f1tenth_stack bringup_launch.py        # Hokuyo / default
    ros2 launch f1tenth_stack sick_bringup_launch.py   # SICK ethernet LiDAR

This starts the VESC drivers, the LiDAR driver, the joystick drivers, the command multiplexer, and the static transform publisher. A healthy startup brings up all eight processes, including these key lines:

.. code-block:: text

    [vesc_driver_node] Connected to VESC with firmware version 5.2
    [joy_node] Opened joystick: <your controller>
    [sick_scan_xd] Setup completed, sick_scan_xd is up and running
    [sick_scan_xd] Software PLL is ready and locked now!
    [ackermann_mux] Topic handler 'topics.joystick' subscribed to topic 'teleop'

Verify from a second terminal that the sensors are publishing:

.. code-block:: bash

    source /opt/ros/jazzy/setup.bash && source ~/f1tenth_ws/install/setup.bash
    ros2 topic list
    ros2 topic hz /scan
    ros2 topic echo /vesc/odom --once

Visualization
^^^^^^^^^^^^^^^
If you installed ``ros-base``, the Jetson has no GUI tools — run RViz on the Pit laptop instead:

.. code-block:: bash

    rviz2

Add a **LaserScan** display on the ``/scan`` topic and set **Fixed Frame** to ``laser`` to see your LiDAR data.

Foxglove is a good alternative, particularly over WiFi. On the Jetson:

.. code-block:: bash

    sudo apt install -y ros-jazzy-foxglove-bridge
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml

Then connect from the laptop to ``ws://<jetson-wifi-ip>:8765``.

.. warning:: Use the Jetson's **WiFi** address here, not ``192.168.0.15`` — that is the isolated LiDAR subnet. Subscribe only to the topics you are actively viewing; adding the ``/cloud`` pointcloud alongside ``/scan`` will saturate a WiFi link.

With the stack running, head over to :ref:`Manual Control <drive_manualcontrol>` to check the teleop chain and take the car for its first drive.

Troubleshooting
-----------------

**A VESC port error, or** ``/dev/sensors/vesc`` **is missing.**
    The udev rule did not fire. Re-run ``sudo udevadm trigger --action=add``, or physically replug the device. *Permission denied* instead means that the ``dialout`` group change requires a logout and login.

**The build fails on** ``vesc_driver``, ``vesc_ackermann`` **or** ``ackermann_mux``.
    The CMake patch was not applied, or was applied after a previous failed build. Re-run the ``find ... sed`` command, confirm with the ``grep`` that follows it, then ``rm -rf build install log`` and build again.

**Edits to a config or launch file appear to have no effect.**
    You did not rebuild. Always run ``colcon build --packages-select f1tenth_stack`` after touching anything in ``config/`` or ``launch/``, and confirm the change by inspecting the file under ``install/``.

**The SICK node connects and then fails during scan configuration.**
    The ``scanner_type`` in ``sick_tim_5xx.launch`` does not match your hardware. It ships set to ``sick_tim_5xx``.

**SSH from the Pit laptop fails.**
    Check the service and the current address on the Jetson with ``systemctl status ssh``, ``ip a show wlP1p1s0`` and ``ip route``. The WiFi address is DHCP-assigned and will change. There should be **no default route** via the ethernet interface — the LiDAR connection was created without a gateway, so if one appears there it will break outbound connectivity. A DHCP reservation on the lab router is worth setting up if you connect to this car regularly.

Joypad and teleop problems are covered in the :ref:`Manual Control <drive_manualcontrol>` section.
