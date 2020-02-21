# articulating-hand

**Operation:**<br>
Driving single servo:<br>
    `rostopic pub /servo_cmd articulating_hand/ServoDrive <servo_number> <servo_position> --once`

**Common errors and how to fix:**<br>
When uploading sketch to Arduino:
- *ioctl("TIOCMSET"): Protocol error*<br>
    Unplug USB cable and plug back in

ROS Errors:
- *Unable to sync with device; possible link problem...*<br>
    Follow these steps:<br>
    1. Exit everything (ROS, Arduino IDE, etc)
    2. Close all terminal windows
    3. Delete /build and /devel folders from workspace
    4. Delete ros_lib from Arduino /libraries directory
    5. Open terminal and navigate to workspace root directory
    6. `catkin_make`
    7. `source devel/setup.bash`
    8. Open new terminal and re-do steps 6 & 7
    9. Run `roscore` in new terminal window
    10. In original terminal window, build Arduino ROS libraries using `rosrun rosserial_arduino make_libraries.py <path_to_install_location>`
    11. Open servo_controller.ino in Arduino IDE and re-upload - be sure servo driver is correctly attached
    12. In original terminal window, execute `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600`. Make sure all 5 INFO messages show up:<br>
        [INFO]: ROS Serial Python Node<br>
        [INFO]: Connecting to /dev/ttyACM0 at 57600 baud<br>
        [INFO]: Requesting topics...<br>
        [INFO]: Note: subscribe buffer size is 280 bytes<br>
        [INFO]: Setup subscriber on servo_cmd [articulating_hand/ServoDrive]
