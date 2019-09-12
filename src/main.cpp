/*
 *
 *   Main file for testing purposes only.
 *   To use in your application please compile as "lib" (see .pro file)
 *   and link with the library.
 */


#include <iostream>
#include "haptikfabrikenapi.h" // Kinematics stuff

#include <thread>
#include <chrono>


#ifdef UNIX
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

using namespace std;
using namespace haptikfabriken;
using namespace std::chrono;

duration<int, std::micro> hundred_milliseconds{100000};
duration<int, std::micro> one_millisecond{1000};



int main()
{
    cout << "Hello World!" << endl;


    HaptikfabrikenInterface hi(false,
                               Kinematics::configuration::polhem_v3(),
                               HaptikfabrikenInterface::USB);
    hi.open();


    std::this_thread::sleep_for(hundred_milliseconds);
    std::this_thread::sleep_for(hundred_milliseconds);
    //hi.calibrate();



    int printcount = 100;

    double t=0;

    bool active_phase=true;
    while(active_phase){
        if(_kbhit()) {
            char cc;
            std::cin >> cc;
            active_phase=false;
        }
        fsVec3d v = hi.getPos();
        int e[6];
        hi.getEnc(e);
        fsVec3d thetas = hi.getBodyAngles()*(180/3.141592);


        double k = 1500;
        /*
         * Spring to the 0,0,0
         */
        /*
        fsVec3d x = v;
        fsVec3d f = -k*x;//fsVec3d(0,1,0);
        */

        fsVec3d f;
        if(v.m_y>0.01) f.m_y = -k*(v.m_y-0.01);


        // Set force
        hi.setForce(f);

        // Alternatively, set actual current in Amperes to respective motor (a,b,c)
        //hi.setCurrent(fsVec3d(0.1,0.1,0.1));

        int ma[3];
        hi.getLatestCommandedMilliamps(ma);

        if(printcount--==0){
            t+=0.1;

            std::cout << "P: " << toString(v)
                      << " enc: " << e[0] << " " << e[1] << " " << e[2] << " "
                                  << e[3] << " " << e[4] << " " << e[5]
                      << " t: "<< toString(thetas)
                      << " ma: " << ma[0] << " " << ma[1] << " " << ma[2]  <<
                         " " <<  hi.getNumReceivedMessages() << " " << hi.getNumSentMessages() <<
                      " " <<  hi.getNumReceivedMessages()/t << " " << hi.getNumSentMessages()/t
                      <<"\n";

printcount = 100;
        }


        std::this_thread::sleep_for(1*one_millisecond);
    }

    cout << "Goodbye World!" << endl;
    hi.close();


    /*

    Kinematics k(Kinematics::configuration::polhem_v2());

    //Kinematic position:     210.34      0.00      -6.31     lambda: -59.22
    //Encoders 0, -5952, 13002

    // The testing values in paper drawing
    //int encoderValues[] = {0,-5952,13002};

    int encoderValues[] = {-2,-7228,14243};
    double end_weight = 0.144; //kg on scale





    fsVec3d force(0,0,1.4);
    fsVec3d t = k.computeMotorAmps(force,encoderValues);

    std::cout << "\nt basic " << toString(k.debugTorques[0]);
    std::cout << "\nt1 " << toString(k.debugTorques[1]);
    std::cout << "\nt2 " << toString(k.debugTorques[2]);
    std::cout << "\nt3 " << toString(k.debugTorques[3]);
    std::cout << "\nt4 " << toString(k.debugTorques[4]);
    std::cout << "\ntgrav sum " << toString(k.debugTorques[1]+
                 k.debugTorques[2]+k.debugTorques[3]+k.debugTorques[4]);

    std::cout << "\nt0: " << t.m_x << "\n";
    std::cout << "t1: " << t.m_y << "\n";
    std::cout << "t2: " << t.m_z << "\n";


    // gives 0, 0.119, 0.088


*/








}
