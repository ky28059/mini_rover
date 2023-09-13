# mini_rover
Boiler Robotics ROS mini rover challenge.

If running on WSL, `colcon build` may fail with [setup,py install is deprecated](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/).
Downgrade setuptools to `58.2.0` to fix:
```bash
pip3 install setuptools==58.2.0
```
