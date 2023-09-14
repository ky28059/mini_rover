# mini_rover
Boiler Robotics ROS mini rover challenge.

To publish `cmd_vel` data and control the rover, either run
```bash
ros2 run mini_rover twist_publisher
```
to read controller input via `pygame` or
```bash
ros2 run mini_rover twist_publisher_middleman
```
```bash
ros2 run joy joy_node
```
to use the `joy` ROS library to subscribe to and publish joystick data.
Note that if the joystick was detected, running `joy_node` should look something like
```bash
kevin@ky28059:~$ ros2 run joy joy_node
[INFO] [1694708210.131486648] [joy_node]: No haptic (rumble) available, skipping initialization
[INFO] [1694708210.131830166] [joy_node]: Opened joystick: Core (Plus) Wired Controller.  deadzone: 0.050000
```
and the topic list should look something like
```bash
kevin@ky28059:~$ ros2 topic list
/cmd_vel
/joy
/joy/set_feedback
/parameter_events
/rosout
```
You can also launch both nodes directly with
```bash
ros2 launch mini_rover minirover_launch.py
```

## Running on WSL2
If running on the latest version of WSL, `colcon build` may fail with [setup,py install is deprecated](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/).
Downgrade setuptools to `58.2.0` to fix:
```bash
pip3 install setuptools==58.2.0
```

<!--
```bash
colcon build --symlink-install
```
-->

### Controller input
To attach a USB device WSL, install the dependencies listed in [this tutorial](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).
In powershell, list USB devices with
```shell
PS C:\Users\kevin> usbipd wsl list
BUSID  VID:PID    DEVICE                                                        STATE
2-2    046d:c53f  USB Input Device                                              Not attached
2-6    045e:0990  Surface Camera Front, Surface IR Camera Front                 Not attached
2-10   8087:0026  Intel(R) Wireless Bluetooth(R)                                Not attached
3-2    20d6:a711  USB Input Device                                              Not attached
3-4    0bda:8152  Realtek USB FE Family Controller                              Not attached
```
and attach the desired USB device to the running WSL instance with
```shell
usbipd wsl attach --busid <busid>
```
Within WSL, `lsusb` should now show the connected device:
```bash
kevin@ky28059:~$ lsusb
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 002: ID 20d6:a711  Core (Plus) Wired Controller
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

WSL's default kernel [does not support input devices](https://www.reddit.com/r/bashonubuntuonwindows/comments/ju64oa/wsl2_devinput/)
(ie. `/dev/input` does not exist). To circumvent this, you can install a [custom kernel image and edit `.wslconfig` to point to it](https://github.com/microsoft/WSL/issues/7747#issuecomment-1328217406).

More concretely, download the `kotc9` kernel from [this link](https://github.com/microsoft/WSL/files/10098030/kernel-xpad.zip)
and extract the `bzImage`, then edit the `kernel` option in `.wslconfig` to point to the location of the extracted `bzImage`;
if the image is located at `C:\kernel-xpad\bzImage`, update `C:\Users\<username>\.wslconfig` to look something like
```
[wsl2]
kernel=C:\\kernel-xpad\\bzImage
```
After a `wsl --shutdown`, the kernel should be updated and `/dev/input` should work.
```bash
kevin@ky28059:~$ uname -a
Linux ky28059 5.10.102.1-kotc9 #7 SMP Sun Nov 27 15:22:50 +05 2022 x86_64 x86_64 x86_64 GNU/Linux
```

### Simulated controller input
To simulate `joy` publishing from the command line without connecting a controller,
```bash
kevin@ky28059:~$ ros2 topic pub /joy sensor_msgs/Joy "axes: [0.0, 1.0, 0.0]"
publisher: beginning loop
publishing #1: sensor_msgs.msg.Joy(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), axes=[0.0, 1.0, 0.0], buttons=[])

publishing #2: sensor_msgs.msg.Joy(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), axes=[0.0, 1.0, 0.0], buttons=[])

publishing #3: sensor_msgs.msg.Joy(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), axes=[0.0, 1.0, 0.0], buttons=[])
```
