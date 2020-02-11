#ifndef FSHAPTICDEVICETHREAD_H
#define FSHAPTICDEVICETHREAD_H



#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <bitset>
#include <set>

#include "kinematics.h"
//#include <chai3d.h>
#include <boost/asio.hpp> // Note: pollutes namespace

#include <boost/thread/mutex.hpp>

// For sleeping
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <thread>

namespace  haptikfabriken {



using namespace std;
using boost::asio::ip::udp;





class FsHapticDeviceThread
{
public:
    FsHapticDeviceThread(Kinematics::configuration c=Kinematics::configuration::polhem_v1());
    void server(boost::asio::io_service& io_service, unsigned short port);
    virtual ~FsHapticDeviceThread() {}
    virtual void thread();
    void event_thread();
    virtual void close(){
        std::cout << "FsHapticDeviceThread close()\n";
        running=false;
        m_thread->join();
        sem_getpos.post(); // unlock potential block in event thread.
        m_event_thread->join();
    }
    virtual int open() {
        running=true;
        std::cout << "Open in fshapticdevicetherad\n";
        m_thread = new boost::thread(boost::bind(&FsHapticDeviceThread::thread, this));
        m_event_thread = new boost::thread(boost::bind(&FsHapticDeviceThread::event_thread, this));
        return 0;
    }
    virtual std::string getErrorCode() { return std::string("Something went wrong"); }

    boost::interprocess::interprocess_semaphore sem_force_sent;
    boost::interprocess::interprocess_semaphore sem_getpos;
    Kinematics kinematics;
    chrono::steady_clock::time_point app_start;

    bitset<5> latestSwitchesState; // Up to 5 switches supported.
    bool running;
    fsVec3d latestPos;
    fsVec3d latestBodyAngles;
    int latestCommandedMilliamps[3];
    int num_sent_messages=0;
    int num_received_messages=0;
    fsRot latestRot;
    int latestEnc[6];
    fsVec3d currentForce;
    fsVec3d nextForce;
    fsVec3d nextCurrent;
    bool useCurrentDirectly{false};

    int offset_encoders[6];
    virtual void calibrate() {}

    boost::mutex mtx_pos;
    boost::mutex mtx_force;
    boost::mutex mtx_getpos;
    int num_getpos{0};

    void addEventListener(HapticListener* listener){
        listener_mutex.lock();
        listeners.insert(listener);
        listener_mutex.unlock();
    }
    void removeEventListener(HapticListener* listener){
        listener_mutex.lock();
        listeners.erase(listener);
        listener_mutex.unlock();
    }

    inline void getEnc(int a[]){
        mtx_pos.lock();
        for(int i=0;i<6;++i)
            a[i]=latestEnc[i];
        mtx_pos.unlock();
    }
    inline fsVec3d getCurrentForce(){
        mtx_pos.lock();
        fsVec3d f = currentForce;
        mtx_pos.unlock();
        return f;
    }
    inline fsVec3d getBodyAngles(){
        mtx_pos.lock();
	fsVec3d t = latestBodyAngles;
        mtx_pos.unlock();
        return t;
    }
    inline void getLatestCommandedMilliamps(int ma[]){
        mtx_pos.lock();
        for(int i=0;i<3;++i)
            ma[i]=latestCommandedMilliamps[i];
        mtx_pos.unlock();
    }
    inline int getNumSentMessages(){
        int r;
        mtx_pos.lock();
        r = num_sent_messages;
        mtx_pos.unlock();
        return r;
    }
    inline int getNumReceivedMessages(){
        int r;
        mtx_pos.lock();
        r = num_received_messages;
        mtx_pos.unlock();
        return r;
    }

    inline fsVec3d getPos(bool blocking=false) {
        //blocking=false;
        if(blocking){
            //sem_getpos.wait();
            //while(sem_getpos.try_wait());
            while(num_getpos==0 && running){
                //using namespace std::chrono;
                //std::chrono::duration<int, std::micro> microsecond{1};
                //this_thread::sleep_for(100*microsecond);
                this_thread::yield();
            }
            num_getpos=0;
        }
        mtx_pos.lock();
        fsVec3d p = latestPos;
        mtx_pos.unlock();
        return p;
    }
    inline fsRot getRot() {
        mtx_pos.lock();
        fsRot r = latestRot;
        mtx_pos.unlock();
        return r;
    }
    inline std::bitset<5> getSwitchesState(){
        std::bitset<5> b;
        mtx_pos.lock();
        b = latestSwitchesState;
        mtx_pos.unlock();
        return b;
    }

    inline void setForce(fsVec3d f){
        useCurrentDirectly = false;
        mtx_force.lock();
        nextForce = f;
        mtx_force.unlock();
    }
    inline void setCurrent(fsVec3d amps){
        useCurrentDirectly = true;
        mtx_force.lock();
        nextCurrent = amps;
        mtx_force.unlock();
    }




    struct out_msg {
        unsigned char start = 0xa4;
        unsigned char number = 0x07;
        unsigned char number2 = 0x51;
        int milliamps_motor_a;
        int milliamps_motor_b;
        int milliamps_motor_c;
        unsigned char end = 0xa4;
        string toString() { stringstream ss; ss<<"mA: " << milliamps_motor_a << " "
                                                        << milliamps_motor_b << " "
                                                        << milliamps_motor_c << "";
                            return ss.str(); }
    };

    // A unsigned number in 2 bytes
    struct utwobyte {
        unsigned char low;
        unsigned char high;
        utwobyte(unsigned char low, unsigned char high):low(low),high(high){}
        utwobyte(){}
        int toInt(){
            return low+(high<<8);
        }
    };
    struct twobyte {
        unsigned char low;
        unsigned char high;
        twobyte(unsigned char low, unsigned char high):low(low),high(high){}
        twobyte(){}
        int toInt(){
            int ch = low+(high<<8);
            return (ch > 0x7FFF)? ch - 0xFFFF : ch;
        }
    };

    struct in_msg {
        unsigned char start = 0xa4;   // Static number for fun "synq byte"
        unsigned char number1 = 0x17; // Number of bytes in payload (17 bytes)
        unsigned char number2 = 0x42; // What this message is about. "Polhem encoders 6dof message"
        unsigned char encoder_abcdef[12]; // 2 bytes per channel encoder
        utwobyte timestamp; // 2 bytes timestamp
        utwobyte id;  // 2 bytes
        unsigned char checksum; // one byte


        int getEnc(int i) {
            int ch = encoder_abcdef[i*2]+(encoder_abcdef[i*2+1]<<8);
            return (ch > 0x7FFF)? ch - 0xFFFF : ch;
        }

        string toString() { stringstream ss; ss<<"enc: " << getEnc(0) << " "
                                                         << getEnc(1) << " "
                                                         << getEnc(2) << " "
                                                         << getEnc(3) << " "
                                                         << getEnc(4) << " "
                                                         << getEnc(5) << " "
                                               <<"ts_mbed: " <<  timestamp.toInt() << " "
                                               <<"id_mbed: " <<  id.toInt();
                            return ss.str(); }
    };


    double to_us(chrono::steady_clock::time_point ts) {
        auto diff = ts-app_start;
        return double(chrono::duration <double, micro> (diff).count());
    }

    int getPosCounter;
    int getPosOldCounter;
    fsVec3d oldPosition;












    boost::asio::io_service* io_service;

    short max_milliamps = 6000;




    boost::thread* m_thread = nullptr;
    boost::thread* m_event_thread = nullptr;

    std::set<HapticListener*> listeners;
    boost::mutex listener_mutex;













};

}

#endif // FSHAPTICDEVICETHREAD_H
