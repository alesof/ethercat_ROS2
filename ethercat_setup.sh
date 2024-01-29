#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

sudo apt-get install mokutil

# Vars
sb_state=$(mokutil --sb-state)
rules_file="/etc/udev/rules.d/99-EtherCAT.rules"
ethercat_file="/etc/sysconfig/ethercat"
rules_content='KERNEL=="EtherCAT[0-9]*", MODE="0666"'

if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}Error: Please run the script with sudo.${NC}"
  exit 1
fi

if [[ "$sb_state" == "SecureBoot enabled" ]]; then
  echo -e "${RED}Error: SecureBoot is enabled. Please disable SecureBoot and try again.${NC}"
  exit 2
fi

echo -e "${GREEN}SecureBoot is not enabled. Continuing installation.${NC} \n"
echo -e "Installing common dependencies..."
#sudo apt-get update
#sudo apt-get upgrade
sudo apt-get install git autoconf libtool pkg-config make build-essential net-tools

echo -e "Cloning and configuring etherlab..."
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout stable-1.5
sudo rm /usr/bin/ethercat
sudo rm /etc/init.d/ethercat
./bootstrap
./configure --prefix=/usr/local/etherlab --disable-8139too --disable-eoe --enable-generic
echo -e "${GREEN}Cloning completed.${NC}"

echo -e "Building..."
sudo make all modules
sudo make modules_install install
sudo depmod
echo -e "${GREEN}Building completed.${NC}\n"

echo -e "Configuring system env..."
sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
sudo mkdir -p /etc/sysconfig
sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
echo -e "${GREEN}System env configured successfully.${NC}\n"

echo -e "Creating rules file..."
echo "$rules_content" | sudo tee "$rules_file" > /dev/null
if [ $? -eq 0 ]; then
  echo -e "${GREEN}File '$rules_file' created successfully.${NC}\n"
else
  echo -e "${RED}Error: Failed to create the file '$rules_file'.${NC}"
  exit 3
fi

echo -e "Modifying ethercat file with ethernet MAC address..."
eth_interface=$(ip link | awk -F: '$0 !~ "lo|vir|wl|^[^0-9]"{print $2;exit}' | tr -d ' ')

if [ -z "$eth_interface" ]; then
  echo -e "${RED}Error: Failed to retrieve an Ethernet interface.${NC}"
  exit 4
fi

eth_mac_address=$(ip addr show dev "$eth_interface" | awk '/ether/ {print $2}')

if [ -z "$eth_mac_address" ]; then
  echo -e "${RED}Error: Failed to retrieve Ethernet MAC address for interface $eth_interface.${NC}"
  exit 5
fi

MASTER0_DEVICE="$eth_mac_address"
DEVICE_MODULES="generic"

echo -e "${YELLOW}Obtained Ethernet interface: $eth_interface${NC}"
echo -e "${YELLOW}Obtained Ethernet MAC address: $MASTER0_DEVICE${NC}"

sed -i "s/^MASTER0_DEVICE=.*/MASTER0_DEVICE=\"$MASTER0_DEVICE\"/" "$ethercat_file"
sed -i "s/^DEVICE_MODULES=.*/DEVICE_MODULES=\"$DEVICE_MODULES\"/" "$ethercat_file"

echo -e "${GREEN}File '$ethercat_file' modified successfully.${NC} \n"

echo -e "Test start..."
start_output=$(sudo /etc/init.d/ethercat start)

if [[ "$start_output" =~ "Starting EtherCAT master 1.5.2  done" ]]; then
  echo -e "${GREEN}Setup successfully completed!${NC}"
else
  echo -e "${RED}Error: Failed to start EtherCAT master.${NC}"
  exit 6
fi

exit 0

