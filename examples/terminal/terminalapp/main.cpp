/*
 *  Examples for using HaptikfabrikenAPI in your applications
 *  or as a simple stand-alone terminal application.
 *
 *
 */

// Uncomment the example you want to run
#define SIMPLE_EXAMPE
//#define VERBOSE_EXAMPE
//#define LISTENER_EXAMPLE



#include <iostream>
#include <sstream>
#include <thread>
#include "haptikfabrikenapi.h"

using namespace std;
using namespace haptikfabriken;


#ifdef LINUX
// ******************** FOR LINUX KEYBOARD LOOP BREAK ***********
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stropts.h>

int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
// *************************************************************
#else
#include <conio.h>
#endif








#ifdef SIMPLE_EXAMPE
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
#endif



















#ifdef VERBOSE_EXAMPE
int main(){
    cout << "Welcome to Haptikfabriken API!\nPress any key to close." << endl;

    // Select model
    Kinematics::configuration c = Kinematics::configuration::polhem_v3();
    //Kinematics::configuration c = Kinematics::configuration::woodenhaptics_v2015();
    //Kinematics::configuration c = Kinematics::configuration::aluhaptics_v2();
    //Kinematics::configuration c = Kinematics::configuration::vintage();

    // Create haptics communication thread.
    HaptikfabrikenInterface hfab(c, HaptikfabrikenInterface::USB);

    // Optionally set max millimaps according to your escons (default 2000).
    hfab.max_milliamps = 4000;

    // Open the communcication
    hfab.open();

    // Verbose or not (set to at least 1 to show debug messages)
    int verbose=0;

    bool running=true;
    while(running){
        if(_kbhit()) running=false;

        // Get position (last received)
        fsVec3d pos = hfab.getPos();

        // Alternatively block until new position is received
        //fsVec3d pos = hfab.getPos(true);

        // Get orientation of manipulandum (never blocks)
        fsRot orientation = hfab.getRot();

        // Position is in Chai3D convention
        //double x = pos.x();  // Increasing positive towards user
        //double y = pos.y();  // Increasing positive to the user's right
        //double z = pos.z();  // Increasing positive upwards

        // Sometimes you want to use another convention, like H3D,
        // where x is to the right, y is upwards and z is towards user.
        // If so you can use this transform:
        /*
        fsRot chaiToH3d{0,1,0,  // h3d x is chai y (second column)
                        0,0,1,  // h3d y is chai z (third column)
                        1,0,0}; // h3d z is chai x (first column)
        fsRot rotx90{1,0,0,
                     0,0,-1,
                     0,1,0};
        fsRot roty90{0,0,1,
                     0,1,0,
                    -1,0,0};
        fsVec3d h3dPos = chaiToH3d * pos;
        fsRot h3dRot = chaiToH3d * orientation * rotx90 * roty90;
        */

        // Compute a force
        fsVec3d f = fsVec3d(0,0,0);

        // For example haptic rendering of a surrounding box
#define RENDER_BOX
#ifdef RENDER_BOX
        fsVec3d boxpos = fsVec3d(0.0,0.0,0.0);
        double k=200;
        double b=0.03;
        double x=pos.x()-boxpos.x();
        double y=pos.y()-boxpos.y();
        double z=pos.z()-boxpos.z();
        double fx,fy,fz;
        fx=0;fy=0;fz=0;

        if(x >  b) fx = -k*(x-b);
        if(x < -b) fx = -k*(x+b);
        if(y >  b) fy = -k*(y-b);
        if(y < -b) fy = -k*(y+b);
        if(z >  b) fz = -k*(z-b);
        if(z < -b) fz = -k*(z+b);

        f = fsVec3d(fx,fy,fz);
#endif



        // Set force
        hfab.setForce(f);

        // Or, set current to the motors directly, in amps
        //hfab.setCurrent(fsVec3d(0,0,0));

        // If you want you have access to the core values, e.g. encoders
        // Uncomment this line to improve speed. Just for info.
        if(verbose){
            int enc[6];
            hfab.getEnc(enc);
            int ma[3];
            hfab.getLatestCommandedMilliamps(ma);

            stringstream ss;

            ss << "\"DeviceName\": \"" << hfab.kinematicModel.name << "\",\n";
            ss << "\"Encoders\": [" << enc[0] << ", " << enc[1] << ", " << enc[2] << ", " << enc[3]
               <<", " << enc[4] << ", " << enc[5] << "],\n";
            ss << "\"CommandedMilliamps\": [" << ma[0] << ", " << ma[1] << ", " << ma[2] << "],\n";
            ss << "\"Position\": [" << toString(pos) << "],\n";
            ss << "\"Orientation\": [\n" << toString(orientation) << "],\n";
            ss << "\"BodyAngles\": [" << toString(hfab.getBodyAngles()) << "],\n";
            //ss << "\"Configuration\": " << toJSON(c) << ",\n";
            ss << "\"CommandedForce\":   [" << f.x() << ", " << f.y() << ", " << f.z() << "]\n";

            std::cout << ss.str();
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Sleep e.g. 1ms = 1000us
        this_thread::sleep_for(std::chrono::microseconds(1000));
    }


    hfab.close();
    std::cout << "Closing from main.\n";

    return 0;
}
#endif





















#ifdef LISTENER_EXAMPLE
/*
 *
 *  Haptic Listner example
 *
 */
class MyHapticListener : public HapticListener {
       void positionEvent(HapticValues& hv){

           // Print every 1000th position for info.
           if((msgcount++)%1000==0)
               std::cout << "Pos: " << toString(hv.position) << "\n";

           // Compute force
           fsVec3d f = -100 * hv.position;

           // Set the force to be rendered on device
           hv.nextForce = f;
       }
       int msgcount{0};
};



int main()
{
    cout << "Welcome to Haptikfabriken API!\nPress any key to close." << endl;

    // Select model
    Kinematics::configuration c = Kinematics::configuration::polhem_v3();
    //Kinematics::configuration c = Kinematics::configuration::woodenhaptics_v2015();
    //Kinematics::configuration c = Kinematics::configuration::aluhaptics_v2();
    //Kinematics::configuration c = Kinematics::configuration::vintage();

    // Create haptics communication thread.
    HaptikfabrikenInterface hfab(c, HaptikfabrikenInterface::USB);

    // Optionally set max millimaps according to your escons (default 2000).
    hfab.max_milliamps = 4000;

    // Open the communcication
    hfab.open();

    // Add our listener
    MyHapticListener* myHapticListener = new MyHapticListener;
    hfab.addEventListener(myHapticListener);

    // Main loop
    bool running=true;
    while(running){
        if(_kbhit()) running=false;

        // Just wait for keyboard hit
        this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    // Remove listener
    hfab.removeEventListener(myHapticListener);
    delete myHapticListener;
    myHapticListener = nullptr;
}

#endif








