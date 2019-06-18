# haptikfabrikenapi

Haptikfabriken API version 0.x
The API is currently under development and might change.

LINUX:

Install dependencies:
See the depenencies.sh script. It is a few standard ubuntu packages 
(including boost) and the Sensoray DAQ PCIe card driver+lib that 
is provided in the external/ folder (and the libhid is there too
but it is compiled in so you wont have to do anything). 
If you are lucky you can just run the ./dependencies.sh script and 
it will install the packages, compile the sensoray driver+library 
and install them. If the latter fails, please read their documentation
in the provided .tar.bz. 

Build & install:
qmake
make
sudo make install

This should place the libhaptikfabrikenapi.so in /usr/local/lib and the
only needed header file haptikfabrikenapi.h in /usr/local/include.

Building debug: 
qmake CONFIG+=debug
Building chai3d debug (or any cmake projekt)
-DCMAKE_BUILD_TYPE=Debug

BUILD TEST APPLICATION:
There are two ways, either build the library as an executable, or build the example application.
1. Build executable: edit haptikfabrikenapi.pro 
   Uncomment (remove #) these lines...
   TEMPLATE = app
   SOURCES += src/main.cpp

   ...and comment these lines out:
   TEMPLATE = lib
   CONFIG += dynamiclib
2. Build example application
   In examples/terminalapp
   qmake
   make
   ./terminalapp


Good luck :)
Jonas Forsslund (jonas@forsslundsystems.com) 2019-06-18



