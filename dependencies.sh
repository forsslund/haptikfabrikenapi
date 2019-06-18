# Basic dependencies
sudo apt install libudev-dev libboost-all-dev libusb-1.0-0-dev

# For building
sudo apt install qt5-qmake qt5-default gcc-multilib

# For Sensoray drivers and API
cd external/sensoray
tar -xjvf sdk_826_linux_3.3.11.tar.bz2
cd sdk_826_linux_3.3.11/

make modules
sudo make install
modprobe s826
make lib
sudo make lib_install


