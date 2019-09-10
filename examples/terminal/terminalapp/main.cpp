#include <iostream>
#include <sstream>
#include <thread>
#include "haptikfabrikenapi.h"

using namespace std;
using namespace haptikfabriken;

//#define RENDER_BOX
#define LOOP

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

int main()
{
    cout << "Welcome to Haptikfabriken API!\nPress any key to close." << endl;

    // Select model
    Kinematics::configuration c = Kinematics::configuration::polhem_v3();
    //Kinematics::configuration c = Kinematics::configuration::polhem_v2();
    //Kinematics::configuration c = Kinematics::configuration::polhem_v1();
    //Kinematics::configuration c = Kinematics::configuration::woodenhaptics_v2015();
    //Kinematics::configuration c = Kinematics::configuration::aluhaptics_v2();
    //Kinematics::configuration c = Kinematics::configuration::vintage();

    // Should the thread block and wait for at least one new position message before continue? (may improve stability)
    // Only implemented in UDP currently.
    bool wait_for_next_message = false;

    // Create haptics communication thread.
    //HaptikfabrikenInterface hfab(wait_for_next_message, c, HaptikfabrikenInterface::DAQ);
    HaptikfabrikenInterface hfab(wait_for_next_message, c, HaptikfabrikenInterface::USB);

    // Optionally set max millimaps according to your escons (default 2000). Might need to set in firmware too.
    hfab.max_milliamps = 4000;

    // Open the communcication
    hfab.open();

    // Verbose or not (set to at least 1 to show debug messages)
    int verbose=0;

    // Main loop doing some haptic rendering
    bool running=true;


    double dir=1;
    double amplitude=0.01;
    while(running){
        if(_kbhit()) {
            char cc;
            std::cin >> cc;
            running=false;
        }

        // Get position and orientation of manipulandum
        fsVec3d pos = hfab.getPos();        
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
#ifdef RENDER_BOX
        fsVec3d boxpos = fsVec3d(0.200,0.0,0.033);
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

/*
        if(fx>3) fx=0;
        if(fx<-3) fx=0;
        if(fy>3) fy=0;
        if(fy<-3) fy=0;
        if(fz>3) fz=0;
        if(fz<-3) fz=0;
*/
        f = fsVec3d(fx,fy,fz);
#endif



        // Set force
        //hfab.setForce(f);

        // Or, set current to the motors directly, in amps
        //amplitude=0.1;
        std::cout << "Output amps: " << dir*amplitude << " " << dir << "\n";
        fsVec3d amps = fsVec3d(1*dir*amplitude,4*dir*amplitude,4*dir*amplitude);
        hfab.setCurrent(amps);

        this_thread::sleep_for(std::chrono::milliseconds(1000));
        dir=-1*dir;
        //amplitude= (amplitude<0.05) ? 0.1 : 0.001;



        // If you want you have access to the core values, e.g. encoders
        int enc[6];
        hfab.getEnc(enc);
        int ma[3];
        hfab.getLatestCommandedMilliamps(ma);

        stringstream ss;
        // Uncomment this line to improve speed. Just for info.
        if(verbose){
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
        }

        // Sleep e.g. 1ms = 1000us
        //this_thread::sleep_for(std::chrono::microseconds(1000));
    }


    hfab.close();

    //char cc;
    //std::cin >> cc;


    return 0;
}
