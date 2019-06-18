LINUX:

sudo apt install gcc-multilib
tar -xjvf sdk_826_linux_3.3.11.tar.bz2 
cd sdk_826_linux_3.3.11/

make modules
sudo make install
modprobe s826
make lib
sudo make lib_install

Lib (both static and dynamic) and headers are in
middleware/


WINDOWS:

Get sdk_826_win_3.3.9.zip and unzip it in this folder (and install, if you have it)
(it was too large to just include here)
http://www.sensoray.com/downloads/sdk_826_win_3.3.9.zip
http://www.sensoray.com/products/826.htm (downloads)
