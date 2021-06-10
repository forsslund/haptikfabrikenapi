# Haptikfabriken API
Haptikfabriken API version 0.x
The API is currently under development and might change.
This is the public API for the Polhem, WoodenHaptics and other haptic device 
controllers made by Haptikfabriken / Forsslund Systems AB.
More information: www.haptikfabriken.com 

## Permissions to use usb 
```console
sudo cp 49-teensy.rules /etc/udev/rules.d/49-teensy.rules
```


## Requirements

To build you need Qmake, which is part of QT, although the Qt libraries
are not used. Besides that there are no dependicies.

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

Not tested as of now. You can use the uhaptikfabrikenapi.h directly if you like. It works :)


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

Jonas Forsslund (jonas@forsslundsystems.com) 2021-06-10



