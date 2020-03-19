# articulating-hand

### Contents

- CAD files
- Laser cutter profiles
- Arduino sketches
- ROS directories and files
    - /config
    - /launch
    - /model
    - /msg
    - /nodes
    - CMakeLists.txt
    - package.xml
- Bill of materials

## ROS Structure

### Nodes
- /converter
    - Creates ROS JointState messages from servo position commands
    - Publishers: `joint_states`
    - Subscribers: `servo_cmd`
- /slider_bar_gui
    - Creates and updates the control GUI window
    - Generates commands to drive servos
    - Publishers: `servo_cmd`
    - Subscribers: None
- /arduino
    - Creates joint motion via PWM signals in response to command
    - Publishers: None
    - Subscribers: `servo_cmd`
- /robot_state_publisher
    - Makes the robot state (ex: position and velocity) available for motion planning, visualization, and simulation
- /rviz
    - Provides visualization for robot positioning

### Topics/Types
Note: only custom types written for this package are included here.

- `servo_cmd`: custom message type `ServoArray`, array of 2-vectors each holding a servo number and position; allows for compact communication by holding all servo position commands in one message


## Assembly

### Printed Parts

Component files provided in .stl form for easy 3D printing. If multiple files exist for the same part, use the file with the highest version number.

Print parts with 0.1mm layer height for best results. Print all pieces with largest flat side touching print bed (see below). Use 25% infill with an appropriate pattern to prevent flat surfaces from sagging. Use supports.

![Print Bed](misc/print_flat.png)

**Number of Pieces to Print:**
- Distal finger: 5
- Proximal finger: 4
- CMC joint small: 1
- CMC joint big: 1

### Laser-Cut Parts



**Operation:**<br>
Driving single servo:<br>
    `rostopic pub /servo_cmd articulating_hand/ServoDrive <servo_number> <servo_position> --once`

**Common errors and how to fix them:**<br>
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
