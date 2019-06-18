#include <iostream>
#include "haptikfabrikenapi.h"
#include "webserv.h"

using namespace std;
using namespace haptikfabriken;

#define RENDER_BOX
#define LOOP

/*
int main()
{
    cout << "Hello World!" << endl;


}*/
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
    cout << "Hello World!" << endl;


    double a[3][3] = {{1,2,3},
                      {4,5,6},
                      {7,8,9}};
    fsMatrix3 m(a);
    fsVec3d v;
    v.m_x = 2;
    v.m_y = 3;
    v.m_z = 4;
    std::cout << "Solution: " << toString(m*v) << " Transpose: " << toString(m.transpose());



    // Select model
    //Kinematics::configuration c = Kinematics::configuration::polhem_v2();
    //Kinematics::configuration c = Kinematics::configuration::polhem_v1();
    Kinematics::configuration c = Kinematics::configuration::woodenhaptics_v2015();
    //Kinematics::configuration c = Kinematics::configuration::aluhaptics_v2();
    //Kinematics::configuration c = Kinematics::configuration::vintage();

    // Should the thread block and wait for at least one new position message before continue? (may improve stability)
    // Only implemented in UDP currently.
    bool wait_for_next_message = false;

    // Create haptics communication thread.

    HaptikfabrikenInterface hfab(wait_for_next_message, c, HaptikfabrikenInterface::USB);

    // Optionally set max millimaps according to your escons (default 2000). Might need to set in firmware too.
    hfab.max_milliamps = 4000;

    // Open the communcication
    hfab.open();

    // Verbose or not (set to at least 1 to show debug messages)
    int verbose=4;

    // NOT NEEDED; IS INCLUDED NOWADAYS
    //Webserv w;
    //w.initialize(8089);

    //hfab.open();


     double fz=0;
    // Main loop doing some haptic rendering
    bool active_phase=true;
    while(active_phase){
        if(_kbhit()) {
            char cc;
            std::cin >> cc;
            fz += 0.1;
            //active_phase=false;
        }

        fsVec3d pos = hfab.getPos();
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

            if(verbose>=3){
              ss << "\"Position\": [" << toString(pos) << "],\n";
              ss << "\"Orientation\": [\n" << toString(hfab.getRot()) << "],\n";
              ss << "\"BodyAngles\": [" << toString(hfab.getBodyAngles()) << "],\n";
              ss << "\"Configuration\": " << toJSON(c) << ",\n";
            }
        }

        // Haptic rendering of a surrounding box
        fsVec3d f = fsVec3d(0,0,0);
//#define RENDER_BOX
#ifdef RENDER_BOXQ
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
#endif
        double fx,fy;

        fx=0;

        fy=0;//pos.y()>-0.074? -300*(pos.y()+0.074) : 0;
        //fz=0;

        fz=1.4;


        f = fsVec3d(fx,fy,fz);

        if(verbose){
            ss << "\"CommandedForce\":   [" << f.x() << ", " << f.y() << ", " << f.z() << "]\n";
            //ss << std::string(24-9,'\n');
            //w.setMessage(ss.str());
            //this_thread::sleep_for(std::chrono::microseconds(10));
        }
        if(verbose>3) std::cout << ss.str();

        // Set force
#ifdef POSITION_CONTROL



        fsVec3d amps = fsVec3d(2,0,0);
        fs.setCurrent(amps);
#else
        hfab.setForce(f);
#endif

        //fsVec3d t = hfab.getBodyAngles();
        //std::cout << t.m_x << "    ";

        //double a = t.m_x > 0? -t.m_x * 5.0 : 0;

        //std::cout << a << std::endl;

        //fsVec3d amps = fsVec3d(a,0.0,0.0);
        //hfab.setCurrent(amps);


    }


    //fsVec3d amps = fsVec3d(0.0,0.0,0.0);
    //hfab.setCurrent(amps);
    hfab.close();

    char cc;
    std::cin >> cc;




    return 0;
}
