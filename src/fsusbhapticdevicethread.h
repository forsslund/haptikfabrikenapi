#ifndef FSUSBHAPTICDEVICETHREAD_H
#define FSUSBHAPTICDEVICETHREAD_H

#include "fshapticdevicethread.h"
#ifdef USE_WEBSERV
#include "webserv.h"
#endif
#include <stdio.h> // for sscanf

// Uncomment to receive final axis data over serial (another controller)
//#define SERIAL_READ

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <thread>

namespace haptikfabriken
{

#pragma pack(push) // Keep these structures unoptimized (packed as is, not padded)
#pragma pack(1)    // Since we are transferring them as is.
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
        return sprintf(c, "%hd %hd %hd %hd %hd %hd %hd\n", encoder_a, encoder_b, encoder_c,
                       encoder_d, encoder_e, encoder_f, info);
    }
    void fromChars(const char *c)
    {
        sscanf(c, "%hd %hd %hd %hd %hd %hd %hd", &encoder_a, &encoder_b, &encoder_c,
               &encoder_d, &encoder_e, &encoder_f, &info);
    }
};

struct pc_to_hid_message
{ // 7*2 = 14 bytes + 1 inital byte always 0
    unsigned char reportid = 0;
    short current_motor_a_mA;
    short current_motor_b_mA;
    short current_motor_c_mA;
    short command; // e.g. reset encoders
    short command_attr0;
    short command_attr1;
    short command_attr2;
    int toChars(char *c) const
    {
        return sprintf(c, "%hd %hd %hd %hd %hd %hd %hd\n", current_motor_a_mA, current_motor_b_mA,
                       current_motor_c_mA, command, command_attr0, command_attr1, command_attr2);
    }
    void fromChars(const char *c)
    {
        sscanf(c, "%hd %hd %hd %hd %hd %hd %hd", &current_motor_a_mA, &current_motor_b_mA,
               &current_motor_c_mA, &command, &command_attr0, &command_attr1, &command_attr2);
    }
};
#pragma pack(pop)


#include <string>
using namespace std;
class SerialComm {
public:
    virtual void open(string portname) = 0;
    virtual void close() = 0;
    virtual void sendWakeupMessage(){pc_to_hid_message m; send(m);}
    virtual void send(const pc_to_hid_message& msg) = 0;
    virtual void receive(hid_to_pc_message& msg) = 0;
    virtual ~SerialComm() {}
};



class FsUSBHapticDeviceThread : public FsHapticDeviceThread
{
public:
    FsUSBHapticDeviceThread(Kinematics::configuration c = Kinematics::configuration::polhem_v3(),
                            std::string serialport_name = "");

    void thread();
    void close();
    int open();

    SerialComm* scom;

    ~FsUSBHapticDeviceThread()
    {
        std::cout << "Fsusb desctructor\n";
        if (running)
        {
            std::cout << "Haptikfabriken threads still running. Please call close() before destructing object.\nTrying to call close() now...\n";
            close();
            std::cout << "Close returned successfully.\n";
        }
#ifdef USE_WEBSERV
        if (w)
        {
            delete w;
            w = nullptr;
        }
#endif
    }

    void calibrate();
    void wakeup_thread();
    const std::string serialport_name;

private:
    int raw_enc[6];

    bool tell_hid_to_calibrate{false};
    bool firstMessage{false};

#ifdef USE_WEBSERV
    Webserv *w;
#endif

    // For pure serial (boost style)
    boost::thread *m_wakeup_thread;
    boost::asio::serial_port *port;
    bool got_message{false};

#ifdef SERIAL_READ
    void usb_serial_thread();
    boost::thread *m_thread_usb_serial = 0;
    boost::mutex mtx_serial_data;
    int serial_data_received = 0;
    int serial_data;
#endif
};
} // namespace haptikfabriken

#endif // FSUSBHAPTICDEVICETHREAD_H
