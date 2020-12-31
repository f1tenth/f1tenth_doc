.. _doc_drive_rosbag:

Recording Data on the Car
================================

.. note:: This section requires a fully built F1TENTH vehicle.

ROSbags are useful for recording data from the car (e.g. LIDAR, odometry) and playing it back later. This feature is useful because it allows you to capture data from when the car is running and later study the data or perform analysis on it to help you develop and implement better racing algorithms.

One great thing about ROSbags compared to just recording the data into something simpler (like a CSV file) is that data is recorded along with the topics it was originally sent on. What this means is that when you later play the bag, the data will be transmitted on the same topics that it was originally sent on, and *any code that was listening to these topics can run, as if the data was being generated live*.

For example, suppose I record LIDAR data being broadcasted on the ``/scan`` topic. When I later play the data back, the ``rostopic list`` and ``rostopic echo`` commands will show the LIDAR data being transmitted on the ``/scan`` topic as if the car was actually running.

Here’s a concrete example of how to use ROSbags to acquire motor telemetry data and play it back.

#. Make sure both your computer and car are connected to the same network. On your laptop, open a terminal and SSH into the car. Once you’re in, run ``tmux`` so that you can spawn new terminal sessions over the same SSH connection.
#. In your tmux session, spawn a new window and run ``roscore`` to start ROS.
#. In the other free terminal, navigate to your working directory, run ``catkin make``, and source the directory using ``source devel/setup.bash``.
#. Run ``roslaunch f1tenth_racecar teleop.launch`` to launch the car. Place the car on the ground or on a stand and press the center button on your joystick so you can control the car.
#. In your tmux session, spawn a new window and examine the list of active ROS topics using ``rostopic list``. Make sure that you can see the ``/vesc/sensors/core`` topic , which contains drive motor parameters.
#. Here’s where ROSbags come into play. Run ``rosbag record /vesc/sensors/core`` to start recording the data. The data will start recording to a file in the current directory with naming format ``YYYY-MM-DD-HH-MM-SS.bag``. Recording will continue until you press Control-C to kill the rosbag process.

	* If you get an error about low disk space, you can specify the directory to record to (e.g. on a USB flash drive or hard drive) after the topic name). For example, ``rosbag record /vesc/sensors/core -o /path/to/external/drive`` to record into an external hard drive.
	* Note that ``rosbag`` also supports recording multiple topics at the same time. For example, I could record both laser scan and motor data using rosbag record ``/vesc/sensors/core /scan`` 

#. Let the recording run for about 30 seconds. Drive the car around during this time using the controller and then hit ``Ctrl-C`` to stop recording.
#. Play the rosbag file using ``rosbag play <your rosbag file>``. While the bag is playing, examine the topics list, and you will see a list of all topics that were recorded into the bag. Note that in addition to the topics you specified, ROS will also record the ``rosout``, ``rosout_agg``, and ``clock`` topics, which can be useful for debugging.
#. View that recorded motor data by echoing the ``/vesc/sensors/core`` topic. Pay attention to how the motor RPM changed as you drove the car around. When the bag is out of data, it will stop publishing.
