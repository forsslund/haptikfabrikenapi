#ifndef FSDAQHAPTICDEVICETHREAD_H
#define FSDAQHAPTICDEVICETHREAD_H

#include "fshapticdevicethread.h"
#include "webserv.h"

namespace haptikfabriken {

#define SERIAL_READ


class FsDAQHapticDeviceThread : public FsHapticDeviceThread
{
public:
    FsDAQHapticDeviceThread(bool wait_for_next_message=false,
        Kinematics::configuration c=Kinematics::configuration::woodenhaptics_v2015()):
        FsHapticDeviceThread::FsHapticDeviceThread(wait_for_next_message,c),w(0),
        m_ixthread{0,0,0,0,0,0},enable_calibration_button_ix0(false){}

    void thread();
    void close();
    void calibrate() { calibrate(false); }
    void calibrate(bool force=false);
    int open();

    void ixthread(int ixchan);

    ~FsDAQHapticDeviceThread(){
        std::cout << "Fsdaq desctructor\n";
    }

private:
    Webserv* w;
    boost::thread* m_ixthread[6];
    bool enable_calibration_button_ix0;


    int speedcheck_enc[3];

#ifdef SERIAL_READ
    void usb_serial_thread();
    boost::thread* m_thread_usb_serial = 0;
    boost::mutex mtx_serial_data;
    int serial_data_received = 0;
    short serial_data;
#endif



};
}
#endif // FSDAQHAPTICDEVICETHREAD_H
