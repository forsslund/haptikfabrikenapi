#ifndef FSDAQHAPTICDEVICETHREAD_H
#define FSDAQHAPTICDEVICETHREAD_H

#include "fshapticdevicethread.h"
#ifdef USE_WEBSERV
#include "webserv.h"
#endif

namespace haptikfabriken {

//#define SERIAL_READ


class FsDAQHapticDeviceThread : public FsHapticDeviceThread
{
public:
    FsDAQHapticDeviceThread(
        Kinematics::configuration c=Kinematics::configuration::woodenhaptics_v2015()):
        FsHapticDeviceThread::FsHapticDeviceThread(c),
        m_ixthread{nullptr},enable_calibration_button_ix0(false)
    #ifdef USE_WEBSERV
      ,w(0)
  #endif
    {}

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
#ifdef USE_WEBSERV
    Webserv* w;
#endif
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
