#include "haptikfabrikenapi.h"
#include "fshapticdevicethread.h"

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>


#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include <string>

#include <chrono>
#include <thread>

namespace haptikfabriken {

using boost::asio::ip::udp;
using namespace std;

enum { max_length = 1024 };

void FsHapticDeviceThread::server(boost::asio::io_service& io_service, unsigned short port)
{
  udp::socket sock(io_service, udp::endpoint(udp::v4(), port));
  for (;;)
  {

    unsigned char data[max_length];
    //for(int i=0;i<max_length;++i)
    //    data[i]='\0';


    udp::endpoint sender_endpoint;
    size_t length = sock.receive_from(
        boost::asio::buffer(data, max_length), sender_endpoint);


/*
    cout << "Got " << length << " bytes:\n";
    for(int i=0;i<length;++i){
        cout << i << ": " << int(data[i]) << "\n";

    }
*/

    int ch_a=0;
    int ch_b=0;
    int ch_c=0;

    fsVec3d pos;
    pos.zero();

    if (data[0]==0xA4 && length>8){
        in_msg* in = reinterpret_cast<in_msg*>(data);

        ch_a = in->getEnc(0);
        ch_b = in->getEnc(1);
        ch_c = in->getEnc(2);


        // Compute position
        pos = kinematics.computePosition(ch_a,ch_b,ch_c);
        int base[] = {ch_a, ch_b, ch_c};
        int rot[]  = {in->getEnc(3), in->getEnc(4), in->getEnc(5)};
        fsRot r = kinematics.computeRotation(base,rot);
        fsVec3d angles = kinematics.computeBodyAngles(base);
        mtx_pos.lock();
        latestBodyAngles = angles;
        latestPos = pos;
        latestRot = r;
        latestEnc[0]=ch_a;
        latestEnc[1]=ch_b;
        latestEnc[2]=ch_c;
        latestEnc[3]=rot[0];
        latestEnc[4]=rot[1];
        latestEnc[5]=rot[2];
        mtx_pos.unlock();
    }



    fsVec3d f;
    f.zero();


#define COMMANDED


#ifdef COMMANDED
    mtx_force.lock();
    f = nextForce;
    mtx_force.unlock();

#endif

#ifdef SPRING
    double k=100;
    f = k * (-pos); // Spring to middle
#endif

#ifdef WALL
    double k=2000;
//      if(pos.y() < 0) f = fsVec3d(0,-k*pos.y(),0);
//     if(ch_a<0)
#endif

#ifdef BOX
    double k=1000;
    double b=0.03;
    double x=pos.x();
    double y=pos.y();
    double z=pos.z();
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

    int enc[3] = {ch_a,ch_b,ch_c};
    fsVec3d amps = kinematics.computeMotorAmps(f,enc);

    if(useCurrentDirectly){
        mtx_force.lock();
        amps = nextCurrent;
        mtx_force.unlock();
    }

#ifdef INSTAB
    amps.zero();
    if(ch_a < 0)
    amps.m_x = -double(ch_a)/10.0;
#endif

    out_msg out;
    out.milliamps_motor_a = -int(amps.x()*1000.0);
    out.milliamps_motor_b = -int(amps.y()*1000.0);
    out.milliamps_motor_c = -int(amps.z()*1000.0);


#ifdef STATIC_CURRENT
    if(pos.y()>0){
        out.milliamps_motor_a = -1000;
        out.milliamps_motor_b = -1000;
        out.milliamps_motor_c = -1000;
    }
#endif




    // Cap at 2A since Escons 24/4 cant do more than that for 4s
    if(out.milliamps_motor_a >= max_milliamps) out.milliamps_motor_a = max_milliamps-1;
    if(out.milliamps_motor_b >= max_milliamps) out.milliamps_motor_b = max_milliamps-1;
    if(out.milliamps_motor_c >= max_milliamps) out.milliamps_motor_c = max_milliamps-1;

    if(out.milliamps_motor_a <= -max_milliamps) out.milliamps_motor_a = -max_milliamps;
    if(out.milliamps_motor_b <= -max_milliamps) out.milliamps_motor_b = -max_milliamps;
    if(out.milliamps_motor_c <= -max_milliamps) out.milliamps_motor_c = -max_milliamps;


    // Avoid 0?
    //if(out.milliamps_motor_a<=0) out.milliamps_motor_a--; else out.milliamps_motor_a++;
    //if(out.milliamps_motor_b<=0) out.milliamps_motor_b--; else out.milliamps_motor_b++;
    //if(out.milliamps_motor_c<=0) out.milliamps_motor_c--; else out.milliamps_motor_c++;

    //std::cout << "Force: " << f.x() << ", " << f.y() << ", " << f.z() << "\n";
    //std::cout << "mA: " << out.milliamps_motor_a << ", " << out.milliamps_motor_b << ", " << out.milliamps_motor_c << "\n";

    mtx_pos.lock();
    latestCommandedMilliamps[0] = out.milliamps_motor_a;
    latestCommandedMilliamps[1] = out.milliamps_motor_b;
    latestCommandedMilliamps[2] = out.milliamps_motor_c;
    mtx_pos.unlock();



    char* const buf = reinterpret_cast<char*>(&out);
/*
    boost::this_thread::sleep_for( boost::chrono::microseconds(500) );
    string out = "1 2 3";


    //sock.send_to(boost::asio::buffer(data, length), sender_endpoint);

*/
    sock.send_to(boost::asio::buffer(buf, sizeof(out)), sender_endpoint);
    currentForce = f;

    // Current force is now the set force
    //mtx_force.unlock();
    //mtx_force.lock();

}
}

FsHapticDeviceThread::FsHapticDeviceThread(Kinematics::configuration c):
    sem_force_sent(0),sem_getpos(0),kinematics(Kinematics(c))
{
    std::cout << "FsHapticDeviceThread::FsHapticDeviceThread()\n";
    app_start = chrono::steady_clock::now();

    for(int i=0;i<6;++i){
        latestEnc[i]=0;
        offset_encoders[i]=0;
    }

}

void FsHapticDeviceThread::thread()
{
  io_service = new boost::asio::io_service();

  try {
    server(*io_service, 47111);
  }
  catch (std::exception& e){
    std::cerr << "Exception in FsHapticDeviceThread::thread(): " << e.what() << "\n";
  }

}

void FsHapticDeviceThread::event_thread()
{
    using namespace std::chrono;
    std::chrono::duration<int, std::micro> microsecond{1};
    high_resolution_clock::time_point t1,t2;

    t1 = high_resolution_clock::now();

    high_resolution_clock::time_point listeners_t1,listeners_t2;
    high_resolution_clock::time_point total_t1,total_t2;
    double listeners_dt=0;
    int listeners_count=0;

    total_t1=high_resolution_clock::now();

    while(running){
        if(listeners.size() == 0){
            this_thread::sleep_for(1000*microsecond);
            continue; // Wait for added listener
        }

        HapticValues hv;
        hv.position = getPos(true);
        bool readyContinue=false;
        while(!readyContinue && running){
            t2 = high_resolution_clock::now();
            duration<double> dt = duration_cast<duration<double>>(t2 - t1);
            if(listeners.size() == 0){
                this_thread::sleep_for(1000*microsecond);
                continue; // Wait for added listener
            }

            listener_mutex.lock();

            for(auto listener : listeners){
                // Ready to call? (1/time since last time > hz)
                if(dt.count() > (1.0/listener->maxHapticListenerFrequency)){
                    hv.position = getPos();
                    hv.orientation = getRot();
                    hv.currentForce = getCurrentForce();
                    hv.nextForce = currentForce;

                    // stats
                    listeners_count++;
                    listeners_t1=high_resolution_clock::now();


                    listener->positionEvent(hv);

                    // stats cont.
                    listeners_t2=high_resolution_clock::now();
                    duration<double> time_span = duration_cast<duration<double>>(listeners_t2 - listeners_t1);
                    listeners_dt += time_span.count();

                    double gain = 1.0;
                    setForce(gain*hv.nextForce);
                    readyContinue=true;
                }
            }
            listener_mutex.unlock();

            if(listeners_count==10000){
                total_t2=high_resolution_clock::now();
                duration<double> total_time_span = duration_cast<duration<double>>(total_t2 - total_t1);

                std::cout << "\nCalled positionEvent() " << listeners_count
                          << " times in " << total_time_span.count()
                          << " s (" << listeners_count/total_time_span.count() << " hz) "
                          << " with avarage execution time of " << listeners_count*listeners_dt/listeners_count << " ms\n\n";
                listeners_count = 0;
                listeners_dt=0;
                total_t1 = total_t2=high_resolution_clock::now();;
            }



            // Add a tiny sleep if maxFrequency not reached
            if(!readyContinue){
                this_thread::sleep_for(10*microsecond); //100khz max
            }
        }
        t1=t2;
    }

}

}
