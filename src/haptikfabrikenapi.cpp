#include "haptikfabrikenapi.h"

#define UHAPTIK

#ifndef UHAPTIK
#include "fshapticdevicethread.h"
#ifdef USE_DAQ
#include "fsdaqhapticdevicethread.h"
#endif
#ifdef USE_USB_HID
#include "fsusbhapticdevicethread.h"
#endif

#include <sstream>
#include <iomanip>
#include <boost/thread/thread.hpp>
#endif

#ifdef UHAPTIK
#include "uhaptikfabriken.h"
#endif

#include <thread>
#include <mutex>
#include <set>

//typedef uhaptikfabriken::HaptikfabrikenInterface FsHapticDeviceThread;
class haptikfabriken::FsHapticDeviceThread {
public:
    uhaptikfabriken::HaptikfabrikenInterface uh;

    FsHapticDeviceThread() {
        std::cout << "haptikfabrikenapi.cpp FsHapticDeviceThread() constructor!\n";
        pelle();
        //std::thread a(&FsHapticDeviceThread::pelle,this);
        //a.join();
        m_event_thread = std::thread{&FsHapticDeviceThread::event_thread, this};
        //m_event_thread.join();
        std::cout << "after thread created!\n";
    }
    ~FsHapticDeviceThread(){
        std::cout << "Destructor!\n";
        running=false;
        m_event_thread.join();
    }

    void pelle(){
        std::cout << "Pelle!\n";

    }

    std::set<haptikfabriken::HapticListener*> listeners;
    std::mutex listener_mutex;
    std::thread m_event_thread;

    void addEventListener(haptikfabriken::HapticListener* listener){
        listener_mutex.lock();
        listeners.insert(listener);
        listener_mutex.unlock();
    }
    void removeEventListener(haptikfabriken::HapticListener* listener){
        listener_mutex.lock();
        listeners.erase(listener);
        listener_mutex.unlock();
    }
    void event_thread(){

        std::cout << "in event_thread\n";

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
                std::this_thread::sleep_for(1000*microsecond);
                continue; // Wait for added listener
            }
            //std::cout << "listeners>0\n";


            haptikfabriken::HapticValues hv;

            //uhaptikfabriken::fsVec3d p = uh.getPos();
            //hv.position = haptikfabriken::fsVec3d(p.m_x,p.m_y,p.m_z);
            bool readyContinue=false;
            while(!readyContinue && running){
                t2 = high_resolution_clock::now();
                duration<double> dt = duration_cast<duration<double>>(t2 - t1);
                if(listeners.size() == 0){
                    std::this_thread::sleep_for(1000*microsecond);
                    continue; // Wait for added listener
                }

                listener_mutex.lock();

                for(auto listener : listeners){
                    // Ready to call? (1/time since last time > hz)
                    if(dt.count() > (1.0/listener->maxHapticListenerFrequency)){

                        uhaptikfabriken::fsVec3d p = 1000*uh.getPos();
                        hv.position = haptikfabriken::fsVec3d(p.m_x,p.m_y,p.m_z);
                        //std::cout << "got x positon " << hv.position.m_x << "\n";

                        hv.orientation.set(uh.getRot().m);

                        uhaptikfabriken::fsVec3d f = uh.getCurrentForce();
                        hv.currentForce = haptikfabriken::fsVec3d(f.m_x,f.m_y,f.m_z);
                        hv.nextForce = hv.currentForce;

                        // stats
                        listeners_count++;
                        listeners_t1=high_resolution_clock::now();


                        listener->positionEvent(hv);

                        // stats cont.
                        listeners_t2=high_resolution_clock::now();
                        duration<double> time_span = duration_cast<duration<double>>(listeners_t2 - listeners_t1);
                        listeners_dt += time_span.count();

                        double gain = 0.001;
                        //std::cout << "sets force from listener: " << toString(hv.nextForce) << "\n";
                        uh.setForce(uhaptikfabriken::fsVec3d(gain*hv.nextForce.m_x,
                                                             gain*hv.nextForce.m_y,
                                                             gain*hv.nextForce.m_z));
                        readyContinue=true;
                    }
                }
                listener_mutex.unlock();

                if(listeners_count==10000){
                    total_t2=high_resolution_clock::now();
                    duration<double> total_time_span = duration_cast<duration<double>>(total_t2 - total_t1);

                    std::cout << "called positionEvent() " << listeners_count
                              << " times in " << total_time_span.count()
                              << " s (" << listeners_count/total_time_span.count() << " hz) "
                              << " with avarage execution time of " << listeners_count*listeners_dt/listeners_count << " ms\n\n";
                    listeners_count = 0;
                    listeners_dt=0;
                    total_t1 = total_t2=high_resolution_clock::now();;
                }



                // Add a tiny sleep if maxFrequency not reached
                if(!readyContinue){
                    std::this_thread::sleep_for(10*microsecond); //100khz max
                }
            }
            t1=t2;
        }
    }
    bool running{true};

};

namespace haptikfabriken {
void fsRot::identity() {
    double a[3][3] = { {1, 0, 0 },
                       {0, 1, 0 },
                       {0, 0, 1 }};
    set(a);
}
void fsRot::rot_x(double t){
    double a[3][3] = { {1,   0,       0    },
                       {0, cos(t), -sin(t) },
                       {0, sin(t), cos(t)  }};
    set(a);
}
void fsRot::rot_y(double t){
    double a[3][3] = { {cos(t),  0, sin(t) },
                       {   0,    1,   0    },
                       {-sin(t), 0, cos(t) }};
    set(a);
}
void fsRot::rot_z(double t){
    double a[3][3] = { {cos(t), -sin(t), 0 },
                       {sin(t), cos(t), 0 },
                       {0, 0, 1 }};
    set(a);
}

fsRot fsRot::transpose()
{
    double a[3][3] = { {m[0][0], m[1][0], m[2][0] },
                       {m[0][1], m[1][1], m[2][1] },
                       {m[0][2], m[1][2], m[2][2] }};
    fsRot r;
    r.set(a);
    return r;
}

fsVec3d operator*(const fsRot &m, const fsVec3d &v)
{
    fsVec3d r;

    r.m_x = m.m[0][0]*v.m_x + m.m[0][1]*v.m_y + m.m[0][2]*v.m_z;
    r.m_y = m.m[1][0]*v.m_x + m.m[1][1]*v.m_y + m.m[1][2]*v.m_z;
    r.m_z = m.m[2][0]*v.m_x + m.m[2][1]*v.m_y + m.m[2][2]*v.m_z;

    return r;
}
}


#ifndef UHAPTIK
//------------------------------------------------------------------------------
// https://www.ridgesolutions.ie/index.php/2012/12/13/boost-c-read-from-serial-port-with-timeout-example/
//------------------------------------------------------------------------------
//
// blocking_reader.h - a class that provides basic support for
// blocking & time-outable single character reads from
// boost::asio::serial_port.
//
// use like this:
//
// 	blocking_reader reader(port, 500);
//
//	char c;
//
//	if (!reader.read_char(c))
//		return false;
//
// Kevin Godden, www.ridgesolutions.ie
//

#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>

class blocking_reader
{
    boost::asio::serial_port &port;
    boost::asio::io_service &io;
    size_t timeout;
    char c;
    boost::asio::deadline_timer timer;
    bool read_error;

    // Called when an async read completes or has been cancelled
    void read_complete(const boost::system::error_code &error,
                       size_t bytes_transferred)
    {

        read_error = (error || bytes_transferred == 0);

        // Read has finished, so cancel the
        // timer.
        timer.cancel();
    }

    // Called when the timer's deadline expires.
    void time_out(const boost::system::error_code &error)
    {

        // Was the timeout was cancelled?
        if (error)
        {
            // yes
            return;
        }

        // no, we have timed out, so kill
        // the read operation
        // The read callback will be called
        // with an error
        port.cancel();
    }

public:
    // Constructs a blocking reader, pass in an open serial_port and
    // a timeout in milliseconds.
    blocking_reader(boost::asio::serial_port &port, boost::asio::io_service &io,
                    size_t timeout) : port(port), io(io), timeout(timeout),
                                      timer(io),
                                      read_error(true)
    {
    }

    // Reads a character or times out
    // returns false if the read times out
    bool read_char(char &val)
    {

        val = c = '\0';

        // After a timeout & cancel it seems we need
        // to do a reset for subsequent reads to work.
        //port.get_executor().context().reset();
        io.reset();

        // Asynchronously read 1 character.
        boost::asio::async_read(port, boost::asio::buffer(&c, 1),
                                boost::bind(&blocking_reader::read_complete,
                                            this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));

        // Setup a deadline time to implement our timeout.
        timer.expires_from_now(boost::posix_time::milliseconds(timeout));
        timer.async_wait(boost::bind(&blocking_reader::time_out,
                                     this, boost::asio::placeholders::error));

        // This will block until a character is read
        // or until the it is cancelled.
        io.run();

        if (!read_error)
            val = c;

        return !read_error;
    }
};
//------------------------------------------------------------------------------
#endif

/*
std::string haptikfabriken::HaptikfabrikenInterface::serialport_name;
unsigned int haptikfabriken::HaptikfabrikenInterface::findUSBSerialDevices()
{
#ifdef UHAPTIK
    return 1;
#else
    using namespace boost::asio;
    boost::asio::io_service io;
    for (int os = 0; os < 2; ++os)
    {
        for (int i = 0; i < 10; ++i)
        {
            std::string str = (os == 0 ? "/dev/ttyACM" : "COM") + std::to_string(i);
            try
            {
                cout << "Trying " << str << "... ";
                serial_port port(io, str);

                boost::asio::streambuf sb;
                blocking_reader reader(port, io, 500);

                char c;
                std::string rsp;

                write(port, boost::asio::buffer("0 0 0 0 0 0 0\n"));

                // read from the serial port until we get a
                // \n or until a read times-out (500ms)
                while (reader.read_char(c) && c != '\n')
                {
                    rsp += c;
                }

                if (c != '\n')
                {
                    // it must have timed out.
                    cout << "Read timed out!";
                }

                std::cout << "Read: " << rsp;
                if (rsp.length() > 10 && rsp.length() < 65)
                {
                    serialport_name = str;
                    port.close();
                    return 1;
                }

                //size_t num_bytes = read_until(port,sb,'\n');
            }
            catch (const std::exception &e)
            {
                cout << e.what() << "\n";
            }
        }
    }

    return 0;
#endif
}
*/
haptikfabriken::HaptikfabrikenInterface::HaptikfabrikenInterface(bool wait_for_next_message, haptikfabriken::Kinematics::configuration c,
                                                                 haptikfabriken::HaptikfabrikenInterface::Protocol protocol): kinematicModel(c), fsthread(nullptr)
{
#ifdef UHAPTIK
    fsthread = new FsHapticDeviceThread();
#else



    switch (protocol)
    {
    case DAQ:
#ifdef USE_DAQ
        fsthread = new FsDAQHapticDeviceThread(c);
#else
        std::cout << "Haptikfabriken compiled without DAQ support.\n";
#endif
        break;
    case USB:
#ifdef PURE_SERIAL
        HaptikfabrikenInterface::findUSBSerialDevices();
#endif
#ifdef USE_USB_HID
        fsthread = new FsUSBHapticDeviceThread(c, serialport_name);
#endif
        break;
    case UDP:
        fsthread = new FsHapticDeviceThread(c);
        break;
    }
#endif
}

//haptikfabriken::HaptikfabrikenInterface::HaptikfabrikenInterface(bool, haptikfabriken::Kinematics::configuration c,
//                haptikfabriken::HaptikfabrikenInterface::Protocol protocol):haptikfabriken::HaptikfabrikenInterface(c,protocol){}


haptikfabriken::HaptikfabrikenInterface::~HaptikfabrikenInterface()
{
    std::cout << "Haptifabriken destructor: " << this << "\n";
    if (fsthread)
        delete fsthread;
    fsthread = nullptr;
}

int haptikfabriken::HaptikfabrikenInterface::open()
{
    fsthread->uh.findUSBSerialDevices();
    return fsthread->uh.open();
}

void haptikfabriken::HaptikfabrikenInterface::close()
{
    std::cout << "Closing haptikfabrikeninterface " << this << "\n";
    if (fsthread)
        fsthread->uh.close();
}

std::string haptikfabriken::HaptikfabrikenInterface::getErrorCode()
{
    return "error";//fsthread->getErrorCode();
}

void haptikfabriken::HaptikfabrikenInterface::getEnc(int a[])
{
    //fsthread->getEnc(a);
}

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getBodyAngles()
{
    return fsVec3d();//fsthread->getBodyAngles();
}

void haptikfabriken::HaptikfabrikenInterface::getLatestCommandedMilliamps(int ma[])
{
    //fsthread->getLatestCommandedMilliamps(ma);
}

int haptikfabriken::HaptikfabrikenInterface::getNumSentMessages()
{
    return 0;//fsthread->getNumSentMessages();
}

int haptikfabriken::HaptikfabrikenInterface::getNumReceivedMessages()
{
    return 0;//fsthread->getNumReceivedMessages();
}

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getPos(bool blocking)
{
#ifdef UHAPTIK
    uhaptikfabriken::fsVec3d p = fsthread->uh.getPos();
    return fsVec3d(p.m_x,p.m_y,p.m_z);
#else
    if (std::abs(fsthread->oldPosition.x() - fsthread->getPos().x()) > 0.01)
    {
        fsthread->oldPosition = fsthread->getPos();
    }
    return fsthread->getPos(blocking);
#endif
}

haptikfabriken::fsRot haptikfabriken::HaptikfabrikenInterface::getRot()
{
    fsRot r;
    return r;//fsthread->uh.getRot();
}

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getCurrentForce()
{
    return fsVec3d();//fsthread->uh.getCurrentForce();
}

void haptikfabriken::HaptikfabrikenInterface::setForce(haptikfabriken::fsVec3d f)
{
    // Limit to 5N
    /*
    double magnitude = sqrt(f.m_x * f.m_x + f.m_y * f.m_y + f.m_z * f.m_z);
    if (magnitude > 5.0)
    {
        fsVec3d dir(f.m_x / magnitude, f.m_y / magnitude, f.m_z / magnitude);
        f = dir * 5.0;
    }*/
    std::cout << "sets force from setForce(): " << toString(f) << "\n";

    fsthread->uh.setForce(uhaptikfabriken::fsVec3d(f.m_x,f.m_y,f.m_z));
}

void haptikfabriken::HaptikfabrikenInterface::setCurrent(haptikfabriken::fsVec3d amps)
{
    //fsthread->setCurrent(amps);
}

std::bitset<5> haptikfabriken::HaptikfabrikenInterface::getSwitchesState()
{
    std::bitset<5> b;
    return b;//fsthread->getSwitchesState();
}

void haptikfabriken::HaptikfabrikenInterface::calibrate()
{
    //fsthread->calibrate();
}

void haptikfabriken::HaptikfabrikenInterface::addEventListener(haptikfabriken::HapticListener *listener)
{
    fsthread->addEventListener(listener);
}

void haptikfabriken::HaptikfabrikenInterface::removeEventListener(haptikfabriken::HapticListener *listener)
{
    fsthread->removeEventListener(listener);
}

std::string haptikfabriken::toString(const haptikfabriken::fsVec3d &r)
{

    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    ss << std::setw(6) << r.m_x << ", " << std::setw(6) << r.m_y << ", " << std::setw(6) << r.m_z;
    return ss.str();
}

std::string haptikfabriken::toString(const haptikfabriken::fsRot &r)
{

    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    ss << std::setw(6) << r.m[0][0] << ", " << std::setw(6) << r.m[0][1] << ", " << std::setw(6) << r.m[0][2] << ",\n";
    ss << std::setw(6) << r.m[1][0] << ", " << std::setw(6) << r.m[1][1] << ", " << std::setw(6) << r.m[1][2] << ",\n";
    ss << std::setw(6) << r.m[2][0] << ", " << std::setw(6) << r.m[2][1] << ", " << std::setw(6) << r.m[2][2] << "\n";
    return ss.str();
}
