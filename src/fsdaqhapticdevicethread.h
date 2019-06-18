#ifndef FSDAQHAPTICDEVICETHREAD_H
#define FSDAQHAPTICDEVICETHREAD_H

#include "fshapticdevicethread.h"
#include "webserv.h"

namespace haptikfabriken {



class FsDAQHapticDeviceThread : public FsHapticDeviceThread
{
public:
    FsDAQHapticDeviceThread(bool wait_for_next_message=false,
        Kinematics::configuration c=Kinematics::configuration::woodenhaptics_v2015()):
        FsHapticDeviceThread::FsHapticDeviceThread(wait_for_next_message,c),w(0),
        m_ixthread{0,0,0,0,0,0},enable_calibration_button_ix0(false){}

    void thread();
    void close();
    void calibrate();
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


};
}
#endif // FSDAQHAPTICDEVICETHREAD_H
