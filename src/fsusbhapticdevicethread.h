#ifndef FSUSBHAPTICDEVICETHREAD_H
#define FSUSBHAPTICDEVICETHREAD_H

#include "fshapticdevicethread.h"
#include "../external/hidapi/hidapi.h"

#include "webserv.h"

namespace haptikfabriken {

struct hid_to_pc_message { // 7*2 = 14 bytes
    short encoder_a;
    short encoder_b;
    short encoder_c;
    short encoder_d;
    short encoder_e;
    short encoder_f;
    short info;
};

struct pc_to_hid_message {  // 7*2 = 14 bytes
    short current_motor_a_mA;
    short current_motor_b_mA;
    short current_motor_c_mA;
    short command; // e.g. reset encoders
    short command_attr0;
    short command_attr1;
    short command_attr2;
};


class FsUSBHapticDeviceThread : public FsHapticDeviceThread
{
public:
    FsUSBHapticDeviceThread(bool wait_for_next_message=false,
        Kinematics::configuration c=Kinematics::configuration::woodenhaptics_v2015());

    void thread();
    void close();

    ~FsUSBHapticDeviceThread(){
        std::cout << "Fsusb desctructor\n";
    }

    void calibrate();


private:
    hid_device_info *devs, *cur_dev;
    hid_device *handle;
    unsigned char buf[15];// 1 extra byte for the report ID
    int res;
    bool tell_hid_to_calibrate;

    hid_to_pc_message hid_to_pc;
    pc_to_hid_message pc_to_hid;
    int raw_enc[6];


    Webserv* w;

};
}

#endif // FSUSBHAPTICDEVICETHREAD_H
