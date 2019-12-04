
#include "fsdaqhapticdevicethread.h"
#include "../external/sensoray/826api.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <ratio>
#include <bitset>



#ifdef SERIAL_READ
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#endif


constexpr uint ENABLE_IX_FILTER = (1 << 31);
constexpr uint ENABLE_CK_FILTER = (1 << 30);

namespace haptikfabriken {


using namespace std;


// Helper functions for getPosition & setForce
//==============================================================================
int signedenc(const uint chan){
    constexpr unsigned int maxdata = 0xFFFFFFFF; // 32 bit
    uint counts;
    S826_CounterRead(0,chan,&counts);
    if(counts >= maxdata/2)
        return counts - maxdata;
    return int(counts);
}

void setVolt(double v, int motor){

#ifdef USE_DIO_PIN_TO_ENABLE_ANALOG_SIGNALS_TO_ESCON
    // Check if we are allowing ourselves to set volt to other than 0.
    // check dio23 (pin 1-2 of J3 (closest to computer back panel, top pins)
    uint pins[2];
    S826_DioInputRead(0,pins);
    std::bitset<32> b(pins[0]);
    const bool userEnable = b[23];

    //std::cout << "B23 is " << b[23] << "\n";

    if(!userEnable)
        v=0;
#endif
    if(v < 10 || v > -10) {
        // -10V to +10V is mapped from 0x0000 to 0xFFFF
        unsigned int signal = uint((v+10.0)/20.0 * 0xFFFF);
        //(uint)(volts * 0xFFFF / 20) + 0x8000;
        S826_DacDataWrite(0,motor,signal,0);
    } else
        printf("Volt outside +/- 10 Volt\n");
}

bool calibration_performed(){
    // Check counter state to see if we need to calibrate (if it is not running that is)
    //constexpr uint COUNTER_STATE_RUN   =  (1 << 29);
    //uint counter_status;
    //S826_CounterStatusRead(0,1,&counter_status);
    //bool running_counter = counter_status & COUNTER_STATE_RUN;
    uint vdata;
    S826_VirtualRead(0,&vdata);
    return vdata!=0;
}
void set_calibration_performed(bool performed){
    S826_VirtualWrite(0,performed,0);
}

void FsDAQHapticDeviceThread::thread()
{
#ifdef SERIAL_READ
    // Start receving usb serial thread
    m_thread_usb_serial = new boost::thread(boost::bind(&FsDAQHapticDeviceThread::usb_serial_thread, this));
#endif


    // Speed check
    speedcheck_enc[0]=signedenc(0);
    speedcheck_enc[1]=signedenc(1);
    speedcheck_enc[2]=signedenc(2);

#ifndef DISABLE_SAFEMODE_CHECK
    std::chrono::time_point<std::chrono::system_clock> speedcheck_time = std::chrono::system_clock::now();
    constexpr int speedcheck_timeout_ms = 40;
    constexpr int speedcheck_minimum_hz = 4000;
    constexpr int speedcheck_mincount = speedcheck_timeout_ms * (speedcheck_minimum_hz / 1000);
    int speedcheck_loop_counts = 0;
    bool speedcheck_running = true;
#endif

    bool use_serial_for_enc5 = false;
    while(running){

        //    std::cout << "Safemode on: " << safemode_settings << "\n";

        // *************** COMPUTE POSITION ***********
        // Compute position

        int enc5_serial=0;

#ifdef SERIAL_READ
        mtx_serial_data.lock();
        enc5_serial = serial_data;
        //receive_count_this_loop = serial_data_received;
        if(!use_serial_for_enc5 && serial_data_received>0)
            use_serial_for_enc5=true;
        serial_data_received=0;
        mtx_serial_data.unlock();
#endif


        const int ch_a=signedenc(0);
        const int ch_b=signedenc(1);
        const int ch_c=signedenc(2);
        fsVec3d pos = kinematics.computePosition(ch_a,ch_b,ch_c);
        int base[] = {ch_a, ch_b, ch_c};
        int rot[3];
        if(kinematics.m_config.variant>1){ // Wired in wrong order
            rot[0] = signedenc(5);
            rot[1] = signedenc(4);
            // Normally we get from daq
            // 2019-08-21 Get from Web..! (bluetooth proxy)
#ifdef USE_WEBSERV
            if(w->activeEnc5())
                rot[2] = w->getEnc5();
            else
#endif
            if(use_serial_for_enc5)
                rot[2] = enc5_serial;
            else
                rot[2] = signedenc(3);
        } else{
            rot[0] = signedenc(3);
            rot[1] = signedenc(4);
            rot[2] = signedenc(5);
        }

        // Check for speed and loop ok.
#if DISABLE_SAFEMODE_CHECK
        S826_WatchdogKick(0,0x5A55AA5A); // Kick the watchdog
#else
        uint safemode_settings;
        S826_SafeControlRead(0,&safemode_settings);
        if(!(safemode_settings & S826_CONFIG_SAF))
            speedcheck_running = true;
        if(speedcheck_loop_counts<=speedcheck_mincount) // avoid looping integer
            speedcheck_loop_counts++;
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - speedcheck_time);
        if(milliseconds.count() > speedcheck_timeout_ms && speedcheck_running){
            if( (std::abs(speedcheck_enc[0] - ch_a) < kinematics.m_config.cpr_encoder_a/2.0) &&
                (std::abs(speedcheck_enc[1] - ch_b) < kinematics.m_config.cpr_encoder_b/2.0) &&
                (std::abs(speedcheck_enc[2] - ch_c) < kinematics.m_config.cpr_encoder_c/2.0) &&
                speedcheck_loop_counts > speedcheck_mincount &&
                calibration_performed()){
                S826_WatchdogKick(0,0x5A55AA5A); // Kick the watchdog

                speedcheck_enc[0] = ch_a;
                speedcheck_enc[1] = ch_b;
                speedcheck_enc[2] = ch_c;
                speedcheck_loop_counts = 0;

            } else {
                std::cout << "Speed or loopcount "<< speedcheck_loop_counts << " (" << speedcheck_loop_counts*1000/milliseconds.count()<<") hz error! Watchdog not kicked!\n";
                std::cout << std::abs(speedcheck_enc[0] - ch_a) << " compared with " << kinematics.m_config.cpr_encoder_a/2.0 << "\n";
                std::cout << std::abs(speedcheck_enc[1] - ch_b) << " compared with " << kinematics.m_config.cpr_encoder_b/2.0 << "\n";
                std::cout << std::abs(speedcheck_enc[2] - ch_c) << " compared with " << kinematics.m_config.cpr_encoder_c/2.0 << "\n";
                speedcheck_running = false;

                S826_SafeWrenWrite(0,S826_SAFEN_SWE);  // Write-enable saefmode data registers
                S826_SafeControlWrite(0,S826_CONFIG_SAF,2); // GO SAFEMODE!
                S826_SafeWrenWrite(0,S826_SAFEN_SWD);
            }
            speedcheck_time = std::chrono::system_clock::now();
        }
#endif


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
        //num_received_messages += 1;
        mtx_pos.unlock();


        // Inform blocking calls to getPos() that we now have a new position
        sem_getpos.post();




        // *************** GET WHAT TO SEND *****
        mtx_force.lock();
        fsVec3d f = nextForce;
        mtx_force.unlock();

        const int enc[3] = { ch_a,
                       ch_b,
                       ch_c};
        fsVec3d amps = kinematics.computeMotorAmps(f,enc);

        if(useCurrentDirectly){
            mtx_force.lock();
            amps = nextCurrent;
            mtx_force.unlock();
        }

        // If not calibrated, go for 0 amps
        if(!calibration_performed())
            amps.zero();

        // Limit to maxium amps and check that it is within interval
        double small=0.001;
        // First axis on Polhem v2 is limited to 1.5 amps since it is so intensly powerful
        const bool polhemv2 = false;// NORMAL MOTOR NOW kinematics.m_config.variant==3;
        const double maxamps_first_axis = polhemv2? 1.5-small : max_milliamps*0.001-small;
        const double maxamps[] = {maxamps_first_axis, max_milliamps*0.001-small,max_milliamps*0.001-small};
        double a[3] = {amps.m_x,amps.m_y,amps.m_z};           

        for(int i=0;i<3;++i){
            a[i] = a[i] <  maxamps[i] ? a[i] :  maxamps[i];
            a[i] = a[i] > -maxamps[i] ? a[i] : -maxamps[i];

            double signal   = 10*a[i]/3.0; // signal 10V gives 3 amps
            if(i==0 && polhemv2) signal = 10*a[i]/1.5; // First axis have 10V gives 1.5 amps (since its so powerful)
            //std::cout << "Signal " << signal << " motor " << i << "\n";
            setVolt(signal, i);
        }

        mtx_pos.lock();
        latestCommandedMilliamps[0] = int(a[0] * 1000);
        latestCommandedMilliamps[1] = int(a[1] * 1000);
        latestCommandedMilliamps[2] = int(a[2] * 1000);
        num_sent_messages++;
        currentForce = f;
        mtx_pos.unlock();


        // Set webserver info
        #ifdef USE_WEBSERV
        stringstream ss;
        ss << "\"DeviceName\": \"" << kinematics.m_config.name << "\",\n";
        ss << "\"Encoders\": [" << enc[0] << ", " << enc[1] << ", " << enc[2] << ", " << rot[0]
           <<", " << rot[1] << ", " << rot[2] << "],\n";
        ss << "\"CommandedMilliamps\": [" << latestCommandedMilliamps[0] << ", "
           << latestCommandedMilliamps[1] << ", " << latestCommandedMilliamps[2] << "],\n";

        ss << "\"Position\": [" << toString(pos*1000.0) << "],\n";
        ss << "\"Orientation\": [\n" << toString(r) << "],\n";
        ss << "\"BodyAngles\": [" << toString(angles) << "],\n";
        ss << "\"Configuration\": " << toJSON(kinematics.m_config) << ",\n";
        ss << "\"CommandedForce\":   [" << f.x() << ", " << f.y() << ", " << f.z() << "],\n";
        for(int i=0;i<5;++i){
            ss << "\"Jacobian_" << i << "\": [\n" << toString(kinematics.debugJacobians[i]) << "],\n";
            ss << "\"Torque_" << i << "\": [" << toString(kinematics.debugTorques[i]) << "],\n";
        }
        ss << "\"NumSentMessages\": " << num_sent_messages << ",\n";
        ss << "\"NumReceivedMessages\": " << num_received_messages << "\n";
        if(w) w->setMessage(ss.str());
#endif





        // // 0.01ms
        //using namespace std::chrono;
        //duration<int, std::micro> dd{100};
        //std::this_thread::sleep_for(dd);


    }


}

void FsDAQHapticDeviceThread::close()
{
    FsHapticDeviceThread::close();
    cout << "Quitting FsDAQ...\n";
    //close
    // Disable power
    setVolt(0,0);
    setVolt(0,1);
    setVolt(0,2);
    S826_DacDataWrite(0,3,0x0,0); // Channel 3 = Pin 48
    S826_DacDataWrite(0,4,0x0,0); // Channel 4 Pin 41 PCB VERSION 1

    // Stop watchdog
    S826_SafeWrenWrite(0,S826_SAFEN_SWE);
    S826_WatchdogEnableWrite(0,0);
    S826_SafeWrenWrite(0,S826_SAFEN_SWD);

    cout << "0...\n";

    cout << "1...\n";
    for(int ix=0;ix<5;++ix){
        S826_CounterWaitCancel(0,ix);
        if(m_ixthread[ix]){
            m_ixthread[ix]->join();
            delete m_ixthread[ix];
        }
    }
    cout << "2...\n";

#ifdef USE_WEBSERV
    if(w)
        delete w;
    w=0;
#endif

    cout << "Done.\n";
}

void FsDAQHapticDeviceThread::ixthread(int ixchan)
{
    while(running){
        uint ctstamp;
        uint reason;
        uint counter;
        uint a = S826_CounterSnapshotRead(0,ixchan,&counter,&ctstamp,&reason,S826_WAIT_INFINITE);
        if(a != S826_ERR_NOTREADY){ // if not nothing in buffer
            if(reason==8 || reason==16){
//                std::cout << ixchan << " button " << reason << std::endl;
                mtx_pos.lock();
                int switchIndex=ixchan;

                // switch button 0 and 1 (since we have calibration on ix0)
                if(ixchan==0) switchIndex=1;
                if(ixchan==1) switchIndex=0;

                latestSwitchesState[switchIndex] = (reason==8)?1:0;
                mtx_pos.unlock();

                if(enable_calibration_button_ix0)
                    if(reason==8 && ixchan==0) calibrate(true); // To calibrate etc
            }
        }
    }
}

void FsDAQHapticDeviceThread::calibrate(bool force)
{
    if((!force) && calibration_performed()) return;
    //double stopsDeg[6] = {120.45,124.8,238.55,0,0,0};

    constexpr unsigned int maxdata = 0xFFFFFFFF; // 32 bit

    // Max point
    //uint counts[6] = {maxdata-8237,14200,maxdata-35249,0,0,0};

    // Nominal position
    //constexpr uint rightmost_first_axis = (120.45/360)*(87.5/14.4)*4096;
    //uint counts[6] = {rightmost_first_axis,maxdata-18447,27140,0,30,0};

    double calib[] = {kinematics.m_config.calibrate_enc_a,
                      kinematics.m_config.calibrate_enc_b,
                      kinematics.m_config.calibrate_enc_c,
                      kinematics.m_config.calibrate_enc_d,
                      kinematics.m_config.calibrate_enc_e,
                      kinematics.m_config.calibrate_enc_f};

    std::cout << "Calibrating encoders: ";

    for(int chan=0;chan<6;++chan){
        std::cout << calib[chan] << " ";
        uint counts = uint(calib[chan]<0? maxdata + calib[chan] : calib[chan]);
        S826_CounterPreloadWrite(0,chan,0,counts);
        S826_CounterPreload(0,chan,0,0);
    }
    std::cout << "\n";

    speedcheck_enc[0]=int(calib[0]);
    speedcheck_enc[1]=int(calib[1]);
    speedcheck_enc[2]=int(calib[2]);

    // Are we in safemode?
    uint safemode_settings;
    S826_SafeControlRead(0,&safemode_settings);
    if(safemode_settings & S826_CONFIG_SAF){
        std::cout << "Disabling safemode\n";
        S826_SafeWrenWrite(0,S826_SAFEN_SWE);  // Write-enable saefmode data registers
        S826_WatchdogEnableWrite(0,0); // Disable watchdog
        S826_SafeControlWrite(0,S826_CONFIG_SAF,1); // Reset safe mode to 0 (normal mode)
//        S826_WatchdogConfigWrite(0,1); // Enable watchdog
        S826_WatchdogEnableWrite(0,1); // Enable watchdog
        S826_SafeWrenWrite(0,S826_SAFEN_SWD);
        S826_WatchdogKick(0,0x5A55AA5A); // Kick the watchdog
    }

    set_calibration_performed(true);


}

int FsDAQHapticDeviceThread::open()
{
    // Start web server
#ifdef USE_WEBSERV
    if(w==0){
        w = new Webserv();
        w->initialize();
    }
#endif

    // Open connection
    int status = S826_SystemOpen();
    if(status<0) return status; // Error, could not open.

    // Set up of fail-safe states i.e. 0 voltage output on escons enables and set value
    S826_SafeWrenWrite(0,S826_SAFEN_SWE);  // Write-enable safemode data registers
    // Start in safe mode if calibration needed, otherwise ready to go normal mode.
    S826_SafeControlWrite(0,S826_CONFIG_SAF, calibration_performed()?1:2);
    for(int i=0;i<S826_NUM_DAC;i++){
        uint span = i<3 ? S826_DAC_SPAN_0_10 : S826_DAC_SPAN_0_5;
        S826_DacRangeWrite(0,i,span,1);
        S826_DacDataWrite(0,i,0,1);
    }
    // Set up watchdog, which will trigger safemode if not regularrily "kicked"
    const uint WD_MILLISECONDS = 100;
    uint wdtiming[] = {WD_MILLISECONDS * 50000, 1, 1, 0 ,0};
    S826_WatchdogConfigWrite(0,0x10,wdtiming);
    S826_WatchdogEnableWrite(0,1);
    S826_SafeWrenWrite(0,S826_SAFEN_SWD);

    // Initialize counters for channel 0,1,2 AND 3,4,5 (gimbal)
    for(unsigned int i=0;i<6;++i){

        constexpr uint multiples_of_20ns = 50000;// 65535; // 65535 = 1.3107 ms (maximum)

        //S826_CounterFilterWrite(0,i,0);
        //S826_CounterFilterWrite(0,i,multiples_of_20ns | ENABLE_IX_FILTER | ENABLE_CK_FILTER);
        S826_CounterFilterWrite(0,i,multiples_of_20ns | ENABLE_IX_FILTER );


        S826_CounterModeWrite(0,i,0x70 | S826_CM_UD_REVERSE); // REVERSE direction so we match
                                                              // direction of turning with positive
                                                              // currents signal
        //S826_CounterPreloadWrite(0,i,0,0); // Load 0
        //S826_CounterPreload(0,i,0,0);


        // Configure snapshot to occur when IX signal either rises or falls,
        // this is to get a callback when i.e. a button is pressed that we
        // have wired to IX channel (since we are not using the encoder's IX)
        S826_CounterSnapshotConfigWrite(0, i, S826_SSRMASK_IXRISE |
                                              S826_SSRMASK_IXFALL, S826_BITWRITE);

        // Enable
        S826_CounterStateWrite(0,i,1);
    }
    for(unsigned int i=0;i<3;++i){
        // And range of analoge signal to escon
        S826_DacRangeWrite(0,i,3,0); //-10 to 10 V
        setVolt(0,i);
    }
    // Enable ESCON driver by a digial signal, which in our case
    // is an analogue signal due to the fact that we only have the
    // analgoue and encoder breakout board of the S826.
    //
    S826_DacDataWrite(0,3,0xFFFF,0); // Channel 3 pin 48 PCB VERSION 2
    S826_DacDataWrite(0,4,0xFFFF,0); // Channel 4 Pin 41 PCB VERSION 1



    // Filter for DIO pins (200us filter)
    uint enabs[] = {23};
    S826_DioFilterWrite(0,50*200,enabs);

    setVolt(0,0);
    setVolt(0,1);
    setVolt(0,2);


    std::cout << "\n************************************\n";
    std::cout <<   "*  Welcome to HaptikfabrikenFsDAQ! *\n";
    std::cout <<   "*  Build: " << __DATE__ << " " __TIME__ << "     *\n";
    std::cout <<   "************************************\n\n";

    std::cout << "Opened DAQ Connection: " << status << std::endl;
    std::cout << "Calibration performed:" << calibration_performed() << "\n";

    if(kinematics.m_config.variant == 3)
        enable_calibration_button_ix0 = true;
    else
        enable_calibration_button_ix0 = false;



    // Enable buttons
    running = true; // ixthreads uses the same variable but it normally is
                    // not set until open() in the end.
    int numButtons = kinematics.m_config.variant==3 ? 5 : 0;
    for(int ix=0;ix<numButtons;ix++){
        m_ixthread[ix] = new boost::thread(boost::bind(&FsDAQHapticDeviceThread::ixthread, this, ix));
    }

    std::cout << "Responding to maximum " << numButtons << " switches. Using ix0 for calibration: " << (enable_calibration_button_ix0?"true" : "false") << "\n";
    std::cout << "Using kinematic model: " << kinematics.m_config.name
              << toJSON(kinematics.m_config);

    // Start normal thread
    FsHapticDeviceThread::open();

    return 0;
}



#ifdef SERIAL_READ
void FsDAQHapticDeviceThread::usb_serial_thread()
{

    std::cout << "Opening /dev/ttyUSB0\n";

    int serial_port = ::open("/dev/ttyUSB0", O_RDWR);
    char read_buf [100];



    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }



    string s;
    while (running) {
        memset(&read_buf, '\0', sizeof(read_buf));

        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
        if (num_bytes < 0) {
            printf("Error reading: %s", strerror(errno));
            break;
        }
        if (num_bytes == 0) continue;
        //cout << "read: " << read_buf << "\n";
        s+=read_buf[0];
        if(read_buf[0]=='\n'){


            //stringstream in(read_buf);
            mtx_serial_data.lock();
            sscanf(s.c_str(),"%hd",&serial_data);
            //receive_count_this_loop++;
            serial_data_received++;
            mtx_serial_data.unlock();
            s="";

            //cout << "got: [" << s << "]" << serial_data<< "\n";


        }

        if(num_bytes>14)
            printf("Read %i bytes. Received message: %s", num_bytes, read_buf);





    }
    ::close(serial_port);
}
#endif


}
