# Ethercat ROS2

## EtherLab installation

### Bios settings
First thing is to check if SecureBoot is enabled. Access the BIOS settings and disable it. To check without accessing the BIOS:
```
sudo apt-get install mokutil
mokutil --sb-state
```

If this returns: `SecureBoot enabled` proceed to disable it.


### EtherCAT install

1. Install common dependencies
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git autoconf libtool pkg-config make build-essential net-tools
```

2. Clone and configure etherlab:
```
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout stable-1.5
sudo rm /usr/bin/ethercat
sudo rm /etc/init.d/ethercat
./bootstrap  # to create the configure script
./configure --prefix=/usr/local/etherlab  --disable-8139too --disable-eoe --enable-generic
```

3. Build:
```
sudo make all modules
sudo make modules_install install
sudo depmod
```

**[Note]:** Last step is necessary every time the Linux kernel is updated.

4. Configure system:
```
sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
sudo mkdir -p /etc/sysconfig
sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
```

5. Modify the content of the file to contain the configuration:
```
sudo gedit /etc/udev/rules.d/99-EtherCAT.rules
```
and paste: 
```
KERNEL=="EtherCAT[0-9]*", MODE="0666"
```

6. Copy the ethernet MAC address to the configuration file in order to contain:
```
MASTER0_DEVICE="YOUR_MAC_ADDRESS"
DEVICE_MODULES="generic"
```

**[Info]:** To check what's the ethernet MAC address: open a new terminal and run: `ip addr`.

## EtherCAT run
```
sudo /etc/init.d/ethercat start
ethercat slaves
```

The setup is successfull if terminal displays this:
```
Starting EtherCAT master 1.5.2  done
0  0:0  PREOP  +  device_0_name
```

## Building ethercat_driver_ros2
1. source ROS2 env: 
```
source /opt/ros/humble/setup.bash
```

2. create a new ws: 
```
mkdir ~/eth_ros2_ws/src
```

3. build:
```
cd ~/eth_ros2_ws
git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git src/ethercat_driver_ros2
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```

### Debugging etherCAT
This is a verbose on the start: `sudo strace -f -e trace=finit_module,init_module ethercatctl start`
