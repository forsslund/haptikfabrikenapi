#ifndef FSUSBHAPTICDEVICETHREAD_H
#define FSUSBHAPTICDEVICETHREAD_H

#include "fshapticdevicethread.h"
#include "webserv.h"

// Uncomment to use the second usb port for sending serial data.
// No improvement shown however. For experimentation only.
//#define SERIAL_READ

namespace haptikfabriken {

#pragma pack(push)  // Keep these structures unoptimized (packed as is, not padded)
#pragma pack(1)     // Since we are transferring them as is.
struct hid_to_pc_message { // 7*2 = 14 bytes
    short encoder_a;
    short encoder_b;
    short encoder_c;
    short encoder_d;
    short encoder_e;
    short encoder_f;
    short info;
};

struct pc_to_hid_message {  // 7*2 = 14 bytes + 1 inital byte always 0
    unsigned char reportid=0;
    short current_motor_a_mA;
    short current_motor_b_mA;
    short current_motor_c_mA;
    short command; // e.g. reset encoders
    short command_attr0;
    short command_attr1;
    short command_attr2;
};
#pragma pack(pop)

class FsUSBHapticDeviceThread : public FsHapticDeviceThread
{
public:
    FsUSBHapticDeviceThread(bool wait_for_next_message=false,
        Kinematics::configuration c=Kinematics::configuration::woodenhaptics_v2015());

    void thread();
    void close();
    int open();
#ifdef SERIAL_READ
    void usb_serial_thread();
#endif

    ~FsUSBHapticDeviceThread(){
        std::cout << "Fsusb desctructor\n";
    }

    void calibrate();


private:
    int raw_enc[6];

    bool tell_hid_to_calibrate;
    bool firstMessage{false};

    Webserv* w;


#ifdef SERIAL_READ
    boost::thread* m_thread_usb_serial = 0;
    boost::mutex mtx_serial_data;
    int serial_data_received = 0;
    short serial_data;
#endif

};
}

#endif // FSUSBHAPTICDEVICETHREAD_H
