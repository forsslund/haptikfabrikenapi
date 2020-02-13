# Haptikfabriken API
Haptikfabriken API version 0.x
The API is currently under development and might change.
This is the public API for the Polhem, WoodenHaptics and other haptic device 
controllers made by Haptikfabriken / Forsslund Systems AB.
More information: www.haptikfabriken.com 

## Requirements

To build you need Qmake, which is part of QT, although the Qt libraries
are not used. Currently you also need [Boost 1.62](https://www.boost.org/users/history/version_1_62_0.html) (exact version), but this 
dependency may be optional in future releases. 

If you want to use the Sensoray DAQ PCIe interface instead of USB, you also need 
to  build its drivers+lib. The required files are included in the external/ folder.
You can look at [dependencies.sh](https://github.com/forsslund/haptikfabrikenapi/blob/master/dependencies.sh) to build the dependencies including DAQ
support in Linux. If the latter fails, please read their documentation
in the provided .tar.bz.

## Configuration
Edit [haptikfabrikenapi.pro](https://github.com/forsslund/haptikfabrikenapi/blob/master/haptikfabrikenapi.pro). For Linux it is recommended to build a dynamic lib, just
uncomment "#CONFIG += dynamiclib", and comment "CONFIG += staticlib". 
If you have a Polhem haptic device, uncomment "#CONFIG += polhemv2".
In Windows, you need to specify the location of Boost (more on that later).
You can also configure it to build a http server for debuging (default).
Finally, you can also build a test app instead of the full library. This 
is useful if you want to step through the inner workings of the library.

## Building and Installing
### Linux (and Mac):
```console
$ qmake
$ make
$ sudo make install
```

This should place the libhaptikfabrikenapi.so in /usr/local/lib and the
only needed header file haptikfabrikenapi.h in /usr/local/include. To make 
a debug build, run "qmake CONFIG+=debug", followed by make and make install.

### Windows

First you need to build Boost. Since there are some non-backwards compatible 
changes in Boost ASIO we need a previous version. 1.62 is known to work well.
Download [Boost 1.62](https://www.boost.org/users/history/version_1_62_0.html) and
unzip to some place with quite a lot o free disk space (I know...).
In the decompressed folder, run:
```
F:\boost_1_62_0> bootstrap
F:\boost_1_62_0> .\b2 address-model=64
```

To build Haptikfabriken API itself I recommend using QtCreator. Open [haptikfabrikenapi.pro](https://github.com/forsslund/haptikfabrikenapi/blob/master/haptikfabrikenapi.pro) as a project, use the standard settings. Edit the line "BOOST = F:\boost_1_62_0" pointing to your Boost folder. Click on "Projects" and then under Build, click "Add Build Step". Select "Make" and
Enter "install" as make argument. This way, when you build the .lib and .h file will
be copied to the haptikfabrikenapi/lib/ folder. Set building to release and build. 


## BUILD TEST APPLICATION:
In [examples/terminal/terminalapp](https://github.com/forsslund/haptikfabrikenapi/tree/master/examples/terminal/terminalapp) there is a collection of small examples 
in the same .cpp file, separated by #defines. Open the .pro file in QtCreator
or build in Linux with:
```console
qmake
make
./terminalapp
```

## Usage

```c++
int main(){
    cout << "Welcome to Haptikfabriken API!\nPress any key to close." << endl;

    // Select model
    Kinematics::configuration c = Kinematics::configuration::polhem_v3();

    // Create haptics communication thread.
    HaptikfabrikenInterface hfab(c, HaptikfabrikenInterface::USB);

    // Open the communcication
    hfab.open();

    while(!_kbhit()){

        // Get position (last received)
        fsVec3d pos = hfab.getPos();

        // Get orientation of manipulandum
        fsRot orientation = hfab.getRot();

        // Print position (note that printing to terminal is "slow")
        std::cout << "\nPosition: \n" << pos.x() << ", " << pos.y() << ", " << pos.z()
                  << "\nOrientation: \n" << toString(orientation);

        // Compute a force
        fsVec3d f = -100 * pos;

        // Set force
        hfab.setForce(f);
    }

    hfab.close();
    return 0;
}
```

Good luck :)

Jonas Forsslund (jonas@forsslundsystems.com) 2020-02-13



