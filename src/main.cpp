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

#include "../external/sensoray/826api.h"

//#define OVERDRIVE_EXPERIMENT
//#define LISTENER_EXAMPLE
#define SIMPLE_EXAMPE
//#define BOOST_SERIAL_EXAMPLE
//#define WEBSERV_TEST

#ifdef WEBSERV_TEST
#include "webserv.h"
int main()
{
    for (int i = 0; i < 2; ++i)
    {
        Webserv *w = new Webserv();
        w->initialize();
        delete w;
        w = 0;
    }
}
#endif

#ifdef BOOST_SERIAL_EXAMPLE
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <thread>
#endif

#ifdef UNIX
// ******************** FOR LINUX KEYBOARD LOOP BREAK ***********
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stropts.h>

int _kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
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

#ifdef BOOST_SERIAL_EXAMPLE
using namespace boost::asio;

#include <iostream>

struct hid_to_pc_message
{ // 7*2 = 14 bytes
    short encoder_a;
    short encoder_b;
    short encoder_c;
    short encoder_d;
    short encoder_e;
    short encoder_f;
    short info;
    int toChars(char *c)
    {
        int n = sprintf(c, "%hd %hd %hd %hd %hd %hd %hd", encoder_a, encoder_b, encoder_c,
                        encoder_d, encoder_e, encoder_f, info);
        std::cout << n << " chars written\n";
        while (n < 62)
            c[n++] = '.';
        c[62] = '\n';
        c[63] = '\0';
        return 63;
    }
    void fromChars(const char *c)
    {
        sscanf(c, "%hd %hd %hd %hd %hd %hd %hd", &encoder_a, &encoder_b, &encoder_c,
               &encoder_d, &encoder_e, &encoder_f, &info);
    }
};

class Bserial
{
public:
    void wakeup_thread()
    {
        while (running)
        {

            // Sleep 100ms
            std::chrono::duration<int, std::micro> microsecond{1};
            this_thread::sleep_for(100000 * microsecond);

            if (!got_message)
            {
                std::cout << "Init writing\n";
                write(*port, buffer("0 0 0 0 0 0 0\n"));
            }
            got_message = false;
        }
    }
    Bserial() : got_message(false), running(true)
    {

        boost::asio::io_service io;
        using namespace std;

        cout << "          1         2         3         4         5         6\n";
        cout << "0123456789012345678901234567890123456789012345678901234567890123\n";

        port = new serial_port(io, "COM9");
        char d[64];
        for (int i = 0; i < 64; ++i)
            d[i] = 0;

        m_wakeup_thread = new boost::thread(boost::bind(&Bserial::wakeup_thread, this));

        boost::asio::streambuf sb;

        for (int p = 0; p < 1000; p++)
        {
            //read(port,buffer(d,63));
            cout << "Reading...\n";
            size_t n = read_until(*port, sb, '\n');
            got_message = true;
            std::string s((std::istreambuf_iterator<char>(&sb)), std::istreambuf_iterator<char>());
            std::cout << "n: " << n << " " << s;
            for (int i = 0; i < 64; i++)
            {
                d[i] = d[i] == ' ' ? '_' : d[i];
                d[i] = d[i] == '\n' ? '*' : d[i];
                d[i] = d[i] == '\0' ? '&' : d[i];
                cout << d[i];
            }
            cout << '\n';

            std::cout << "Force writing\n";
            write(*port, buffer("0 0 0 0 0 0 0\n"));
            std::chrono::duration<int, std::micro> microsecond{1};
            this_thread::sleep_for(50000 * microsecond);
        }

        //io.run();
    }
    void msg()
    {
    }

    boost::thread *m_wakeup_thread;
    serial_port *port;
    bool got_message;
    bool running;
};

int main()
{
    Bserial b;
    return 0;
}

#endif

#ifdef SIMPLE_EXAMPE
int main()
{
    cout << "Hello World!" << endl;

#ifdef PURE_SERIAL
    unsigned int a = HaptikfabrikenInterface::findUSBSerialDevices();
    cout << "Found " << a << " devices\n";
    cout << "Serialport name: " << HaptikfabrikenInterface::serialport_name << "\n";
    if (!a)
        return 0; // no devices found
#endif

    HaptikfabrikenInterface hi(Kinematics::configuration::woodenhaptics_v2015(),
                               HaptikfabrikenInterface::USB);
    hi.open();

    std::this_thread::sleep_for(hundred_milliseconds);
    std::this_thread::sleep_for(hundred_milliseconds);

    //hi.calibrate(); // for woodenhaptics

    bool active_phase = true;

    fsVec3d thetas_start = hi.getBodyAngles() * (180 / 3.141592);





    bool go = 0;
    while (active_phase)
    {
        if (_kbhit())
        {
            char cc;
            std::cin >> cc;
            active_phase = false;
        }

        //continue;

        fsVec3d v = hi.getPos();
        int e[6];
        hi.getEnc(e);
        fsVec3d thetas = hi.getBodyAngles() * (180 / 3.141592);

        double dtheta = thetas.m_x - thetas_start.m_x;

        go = !go;

        // Alternatively, set actual current in Amperes to respective motor (a,b,c)
        //fsVec3d c(0.000, 0, 0);
        //hi.setCurrent(c);
        hi.setForce(fsVec3d(0,0,0));

        continue;

        int ma[3];
        hi.getLatestCommandedMilliamps(ma);

        std::cout << "dtheta; " << dtheta << " P: " << toString(v)
                  << " enc: " << e[0] << " " << e[1] << " " << e[2] << " "
                  << e[3] << " " << e[4] << " " << e[5]
                  << " t: " << toString(thetas)
                  << " ma: " << ma[0] << " " << ma[1] << " " << ma[2]
                  << "!!!\n";

        std::this_thread::sleep_for(100 * one_millisecond);
    }


    high_resolution_clock::time_point t1, t2;
    t1 = high_resolution_clock::now();
    for(int i=0;i<1000;i++){

        fsVec3d v = hi.getPos(true);

    }
    t2 = high_resolution_clock::now();

    //elapsed = (double)(end.tv_sec - begin.tv_sec) * 1000.0;
    //elapsed += (double)(end.tv_usec - begin.tv_usec) / 1000.0;
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    double elapsed = time_span.count();

    cout << "elased: " <<  elapsed << " ms";


    cout << "Goodbye World!" << endl;
    hi.close();
}
#endif

#ifdef LISTENER_EXAMPLE
/*
 *
 *  Haptic Listner example
 *
 */
class MyHapticListener : public HapticListener
{
public:
    void positionEvent(HapticValues &hv)
    {
        fsVec3d f = -100 * hv.position;
        hv.nextForce = f;
    }
    MyHapticListener()
    {
        t1 = high_resolution_clock::now();
        t2 = t1;
    }

    high_resolution_clock::time_point t1, t2;
};

int main()
{
    cout << "Hello World Listener Example!" << endl;

    HaptikfabrikenInterface hi(Kinematics::configuration::polhem_v3(),
                               HaptikfabrikenInterface::USB);
    hi.open();

    // std::this_thread::sleep_for(hundred_milliseconds);

    // Add our listener
    MyHapticListener *myHapticListener = new MyHapticListener();
    hi.addEventListener(myHapticListener);

    // Main loop
    bool running = true;
    while (running)
    {
        if (_kbhit())
            running = false;

        int enc[6];
        hi.getEnc(enc);

        std::cout << "Pos: " << toString(hi.getPos()) << " Force:" << toString(hi.getCurrentForce()) << " " << enc[0] << " " << enc[1] << " " << enc[2] << " "
                  << "\n";

        // Just wait for keyboard hit
        this_thread::sleep_for(std::chrono::microseconds(100000)); // 100ms
    }

    // Remove listener
    hi.removeEventListener(myHapticListener);
    delete myHapticListener;
    myHapticListener = nullptr;
    hi.close();

    char c;
    std::cin >> c;
}
#endif // LISTENER_EXAMPLE

#ifdef SENSORYA_PWM_EXPERIMENTATION
int SetPWM(uint board, uint ctr, uint ontime, uint offtime)
{
    S826_CounterPreloadWrite(board, ctr, 0, ontime);  // On time in us.
    S826_CounterPreloadWrite(board, ctr, 1, offtime); // Off time in us.
}

int CreatePWM(uint board, uint ctr, uint ontime, uint offtime)
{
    S826_CounterModeWrite(board, ctr,                              // Configure counter for PWM:
                                                                   //    S826_CM_K_1MHZ |                       //   clock = internal 1 MHz
                          S826_CM_K_50MHZ |                        //   clock = internal 1 MHz
                              S826_CM_UD_REVERSE |                 //   count down
                              S826_CM_PX_START | S826_CM_PX_ZERO | //   preload @startup and counts==0
                              S826_CM_BP_BOTH |                    //   use both preloads (toggle)
                              S826_CM_OM_PRELOAD |
                              S826_CM_OP_INVERT); //   assert ExtOut during preload0 interval
    SetPWM(board, ctr, ontime, offtime);          // Program initial on/off times.

    S826_CounterSnapshotConfigWrite(0, 1, S826_SSRMASK_ZERO, S826_BITWRITE);
}

int StartPWM(uint board, uint ctr)
{
    return S826_CounterStateWrite(board, ctr, 1); // Start the PWM generator.
}

int RouteCounterOutput(uint board, uint ctr, uint dio)
{
    uint data[2]; // dio routing mask
    if ((dio >= S826_NUM_DIO) || (ctr >= S826_NUM_COUNT))
        return S826_ERR_VALUE; // bad channel number
    if ((dio & 7) != ctr)
        return S826_ERR_VALUE; // counter output can't be routed to dio

    // Route counter output to DIO pin:
    S826_SafeWrenWrite(board, S826_SAFEN_SWE); // Enable writes to DIO signal router.
    S826_DioOutputSourceRead(board, data);     // Route counter output to DIO
    data[dio > 23] |= (1 << (dio % 24));       //   without altering other routes.
    S826_DioOutputSourceWrite(board, data);
    return S826_SafeWrenWrite(board, S826_SAFEN_SWD); // Disable writes to DIO signal router.
}

int main()
{
    // Open connection
    int status = S826_SystemOpen();
    if (status < 0)
        return status; // Error, could not open.

    S826_DacDataWrite(0, 3, 0xFFFF, 0); // Channel 3 pin 48 PCB VERSION 2

    // 10.000 = 5khz   available for data 80% = 8000 (12 bit is 4096)
    // 50.000 = 1khz
    // 25.000 = 2khz
    // Example: Configure counter0 and dio0 as a PWM generator; ontime = 900, offtime = 500 microseconds.
    const int pwm_period_ticks = 10000;
    CreatePWM(0, 1, 0.5 * pwm_period_ticks, 0.5 * pwm_period_ticks); // Configure counter0 as PWM.
    RouteCounterOutput(0, 1, 1);                                     // Route counter0 output to dio 1, e.g. j3 pin 45
    StartPWM(0, 1);                                                  // Start the PWM running.

    S826_SafeControlWrite(0, 0, 0); // No safemode!

    uint counts;

    int ontime = 100;
    int dio_channel = 24 - 1;
    bool on = false;
    while (!_kbhit())
    {

        //ontime+=100;
        //if(ontime>5000000) ontime=5000000; // 90% max, 10% min

        uint reason;
        S826_CounterSnapshotRead(0, 1, 0, 0, &reason, S826_WAIT_INFINITE);
        uint status;
        S826_CounterStatusRead(0, 1, &status);
        //if(!(status & (1<<16))) std::cout << "1..\n"; else std::cout << "0";
        //std::cout << reason << "\n";

        uint data[2];
        S826_DioOutputRead(0, data);
        //if(data[1]) std::cout << "data: " << data[1] << "\n";

        S826_CounterRead(0, 1, &counts);
        //std::cout << "coutns: " << counts << "\n";

        on = !on;
        data[0] = (1 << 0);
        if (on)
        {
            S826_DioOutputWrite(0, data, S826_BITSET);
            SetPWM(0, 1, 0.4 * pwm_period_ticks, 0.6 * pwm_period_ticks);
        }
        else
        {
            S826_DioOutputWrite(0, data, S826_BITCLR);
            SetPWM(0, 1, 0.5 * pwm_period_ticks, 0.5 * pwm_period_ticks);
        }

        /*
        if(status && (1<<16)) std::cout << "1..\n"; else cout << "0..\n";
        //S826_CounterSnapshotRead(0,1,0,0,0,S826_WAIT_INFINITE);
        S826_CounterStatusRead(0,1,&status);
        if(status && (1<<16)) std::cout << "2..\n";else cout << "0..\n";
        //S826_CounterSnapshotRead(0,1,0,0,0,S826_WAIT_INFINITE);
        S826_CounterStatusRead(0,1,&status);
        if(status && (1<<16)) std::cout << "3..\n";else cout << "0..\n";
*/

        /*
        if(status && (1<<16)){ // if preload1 is used "the long, 0 voltage", we can load new values while it finishes
            SetPWM(0, 1, 10000-ontime, ontime);     // Program initial on/off times.
        } else {
            // Wait until we are at the long
            //S826_CounterSnapshotRead(0,1,0,0,0,S826_WAIT_INFINITE);

            // verify
            S826_CounterStatusRead(0,1,&status);
            if(status && (1<<16)) std::cout << "error...\n";

            SetPWM(0, 1, 10000-ontime, ontime);     // Program initial on/off times.
        }
*/

        /*
//        uint data[] = {0x00FFFFFF,0x00FFFFFF}; // channels 0 to 23, 24 to 47
        dio_channel++;
        if(dio_channel>47) dio_channel = 24;

        dio_channel = 25; // = J2_PIN_45
        uint data[] = {0x00FFFFFF,(1<<(dio_channel-24))}; // channels 0 to 23, 24 to 47
        S826_DioOutputWrite(0,data,0);

        std::cout << "Pin: " << dio_channel << "\n";

        int i;
        uint pins[2];                // Buffer for pin states.
        S826_DioInputRead(0, pins);  // Read all DIO pin states into buffer.
        for (i = 0; i < 24; i++)     // Display states of DIOs 0-23.
          printf("dio%d = %d\n", i, (pins[0] >> i) & 1);
        for (i = 24; i < 48; i++)    // Display states of DIOs 24-47.
          printf("dio%d = %d\n", i, (pins[1] >> (i - 24)) & 1);
        */

        //std::cout << "Ontime %: " << 100*float(ontime)/10000 << "\n";
        std::this_thread::sleep_for(30 * one_millisecond);
    }

    SetPWM(0, 1, 0.1 * 25000, 0.9 * 25000); // Program initial on/off times.

    S826_DacDataWrite(0, 3, 0, 0); // Channel 3 pin 48 PCB VERSION 2
}
#endif

#ifdef OVERDRIVE_EXPERIMENT

int main()
{
    cout << "Hello World!" << endl;

    HaptikfabrikenInterface hi(false,
                               Kinematics::configuration::woodenhaptics_v2015(),
                               HaptikfabrikenInterface::DAQ);
    hi.open();

    std::this_thread::sleep_for(hundred_milliseconds);
    std::this_thread::sleep_for(hundred_milliseconds);
    hi.calibrate();

    /*




    // Add our listener
    MyHapticListener* myHapticListener = new MyHapticListener();
    hi.addEventListener(myHapticListener);

    // Main loop
    bool running=true;
    while(running){
        if(_kbhit()) running=false;

        //std::cout << "Pos: " << toString(hi.getPos()) << " Force:" << toString(hi.getCurrentForce()) << "\n";

        // Just wait for keyboard hit
        this_thread::sleep_for(std::chrono::microseconds(100000)); // 100ms
    }

    // Remove listener
    hi.removeEventListener(myHapticListener);
    delete myHapticListener;
    myHapticListener = nullptr;
    hi.close();

    return 0;

*/

    int printcount = 100;

    double t = 0;

    bool active_phase = true;

    fsVec3d thetas_start = hi.getBodyAngles() * (180 / 3.141592);

    int current = 1;
    while (active_phase)
    {
        if (_kbhit())
        {
            char cc;
            std::cin >> cc;
            active_phase = false;
        }

        //continue;

        fsVec3d v = hi.getPos();
        int e[6];
        hi.getEnc(e);
        fsVec3d thetas = hi.getBodyAngles() * (180 / 3.141592);

        double dtheta = thetas.m_x - thetas_start.m_x;

        double k = 1500;
        /*
         * Spring to the 0,0,0
         */
        /*
        fsVec3d x = v;
        fsVec3d f = -k*x;//fsVec3d(0,1,0);
        */

        fsVec3d f;
        if (v.m_y > 0.01)
            f.m_y = -k * (v.m_y - 0.01);

        // Set force
        //hi.setForce(f);

        // Alternatively, set actual current in Amperes to respective motor (a,b,c)

        fsVec3d c;
        c.zero();
        if (dtheta > 0)
        {
            c.m_x = -dtheta * 0.25; // 20 degrees = 5 amps

            if (c.m_x < -5)
                c.m_x = -5;
        }

        hi.setCurrent(c);

        int ma[3];
        hi.getLatestCommandedMilliamps(ma);

        if (printcount-- == 0)
        {
            t += 0.1;

            std::cout << "dtheta; " << dtheta << " P: " << toString(v)
                      << " enc: " << e[0] << " " << e[1] << " " << e[2] << " "
                      << e[3] << " " << e[4] << " " << e[5]
                      << " t: " << toString(thetas)
                      << " ma: " << ma[0] << " " << ma[1] << " " << ma[2] << " " << hi.getNumReceivedMessages() << " " << hi.getNumSentMessages() << " " << hi.getNumReceivedMessages() / t << " " << hi.getNumSentMessages() / t
                      << "\n";

            printcount = 100;
        }

        std::this_thread::sleep_for(1 * one_millisecond);
        current = !current;
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
#endif // EXAMPLE
