#ifndef FSUSBHAPTICDEVICETHREAD_H
#define FSUSBHAPTICDEVICETHREAD_H

#include "fshapticdevicethread.h"
#include "webserv.h"
#include <stdio.h> // for sscanf

// Uncomment to receive final axis data over serial (another controller)
//#define SERIAL_READ




#ifdef WIN32
/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/


#ifndef SERIALPORT_H
#define SERIALPORT_H

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

class SerialPort
{
private:
    HANDLE handler;
    bool connected;
    COMSTAT status;
    DWORD errors;
public:
    SerialPort(const char *portName);
    ~SerialPort();

    int readSerialPort(char *buffer, unsigned int buf_size);
    bool writeSerialPort(char *buffer, unsigned int buf_size);
    bool isConnected();
    void closeSerial();
};

#endif // SERIALPORT_H
#endif // WIN32



















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
    int toChars(char *c){
        return sprintf(c,"%hd %hd %hd %hd %hd %hd %hd\n",encoder_a,encoder_b,encoder_c,
                                         encoder_d,encoder_e,encoder_f, info);
    }
    void fromChars(const char *c){
        sscanf(c,"%hd %hd %hd %hd %hd %hd %hd",&encoder_a,&encoder_b,&encoder_c,
                                          &encoder_d,&encoder_e,&encoder_f, &info);
    }
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
    int toChars(char *c){
        return sprintf(c,"%hd %hd %hd %hd %hd %hd %hd\n",current_motor_a_mA,current_motor_b_mA,
              current_motor_c_mA, command,command_attr0,command_attr1, command_attr2);
    }
    void fromChars(const char *c){
        sscanf(c,"%hd %hd %hd %hd %hd %hd %hd", &current_motor_a_mA,&current_motor_b_mA,
              &current_motor_c_mA, &command,&command_attr0,&command_attr1, &command_attr2);
    }
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

    ~FsUSBHapticDeviceThread(){
        std::cout << "Fsusb desctructor\n";
    }

    void calibrate();


private:
    int raw_enc[6];

    bool tell_hid_to_calibrate{false};
    bool firstMessage{false};

    Webserv* w;


#ifdef SERIAL_READ
    void usb_serial_thread();
    boost::thread* m_thread_usb_serial = 0;
    boost::mutex mtx_serial_data;
    int serial_data_received = 0;
    int serial_data;
#endif

};
}

#endif // FSUSBHAPTICDEVICETHREAD_H
