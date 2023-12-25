# ROS2 azd-kd Driver

A node package based on pymodbus (2.5.3) to communicate with Oriental Motor AZD-KD driver.

## Installation

navigate into your ROS workspace's `src` directory and clone `azd-kd`:

```bash
$ cd ~/ros_workspace/src
$ git clone https://github.com/ietamaher/azd-kd
```
Then build it -  use `colcon build`:

```bash
$ cd ~/ros_workspace/

# ROS2 Foxy/Humble
$ colcon build
$ source install/local_setup.bash 
```
update config_modbus.yaml based on communication parameters (port: '/dev/ttyUSB0', baudrate: 115200, etc ...)
The nodes should now be built and ready to use.  Remember to source the overlay as shown above so that ROS can find the nodes.
* In case you have multiple USB devices you can use :
* 
```bash
$ ls /dev/serial/by-path/plateform-ff0005xxxxx.pcie.pci 
$ ls /dev/serial/by-id/USB-Silicon_labxxxxxxxxx
```

## Testing

```bash
# ROS2 Foxy/Humble
$ sudo chmod 666 /dev/ttyUSB0
$ ros2 launch azd_kd azd_kd_modbus.launch.py
```
to displays topics /azd_kd_query /azd_kd_response /azd_kd_state, run :

```bash
$ ros2 topic list 
```

To start communicating with the azd-kd driver, run one of the samples. Open new tab and run the following : 
```bash
$ source install/local_setup.bash 
$ python src/azd-kd/samples/sample1.py
```
you can observe topics /azd_kd_query /azd_kd_response /azd_kd_state by runing ; 

```bash
$ ros2 topic echo /azd_kd_response 
```
