#include "fsusbhapticdevicethread.h"
#include "../external/hidapi/hidapi.h"

// For USB HID version
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>

#include <sstream>
#include <thread>
#include <chrono>

#ifndef UNIX
void usleep(int us){
    using namespace std::chrono;
    duration<int, std::micro> dd{us};
    std::this_thread::sleep_for(dd);
}
#endif

#ifdef SERIAL_READ
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#endif


constexpr int WOODENHAPTICS = 1;
constexpr int POLHEM_USB = 2;

constexpr int PCB = POLHEM_USB;

namespace haptikfabriken {

FsUSBHapticDeviceThread::FsUSBHapticDeviceThread(bool wait_for_next_message,
                                                 Kinematics::configuration c):
    FsHapticDeviceThread::FsHapticDeviceThread(wait_for_next_message,c),w(0){}

void FsUSBHapticDeviceThread::thread()
{
    // Start web server
    if(w==0){
        w = new Webserv();
        w->initialize();
    }

#ifdef SERIAL_READ
    // Start receving usb serial thread
    m_thread_usb_serial = new boost::thread(boost::bind(&FsUSBHapticDeviceThread::usb_serial_thread, this));
#endif

    // For debugging
    num_sent_messages = 0;
    num_received_messages = 0;

    // Set protocol 1=Old usb, 2=April 2018, and for POLHEM
    constexpr int protocol_version = PCB==WOODENHAPTICS ? 1 : 2;

    // Open the device using the VID, PID,
    // and optionally the Serial number.
    hid_device *handle = hid_open(0x1234, 0x6, nullptr);
    if (!handle) {
        std::cout << "unable to open device. Is it plugged in and you run with right permission (or as root?)\n";
        return;
    }
    std::cout << "\n************************************\n";
    std::cout <<   "*  Welcome to HaptikfabrikenFsUSB! *\n";
    std::cout <<   "*  Build: " << __DATE__ << " " __TIME__ << "     *\n";
    std::cout <<   "************************************\n\n";
    std::cout << "Opened USB Connection" << std::endl;

    tell_hid_to_calibrate = false;
    bool forced_inital_calibration = PCB==WOODENHAPTICS ? true : false;
    const int out_bytes = (protocol_version==2)? 15: 9;
    const int in_bytes = (protocol_version==2)? 15: 9;
    unsigned char in_buf[in_bytes];

    std::chrono::duration<int, std::micro> microsecond{1};

    hid_to_pc_message hid_to_pc;
    pc_to_hid_message pc_to_hid;

    using namespace std::chrono;

    high_resolution_clock::time_point t1,t2;

    bool use_serial_for_enc5 = false;
    while(running){
        t1 = high_resolution_clock::now();

        int receive_count_this_loop = 0;

        int enc5_serial=0;
#ifdef SERIAL_READ
        mtx_serial_data.lock();
        enc5_serial = serial_data;
        //receive_count_this_loop = serial_data_received;
        if(!use_serial_for_enc5 && serial_data_received>0){
            use_serial_for_enc5=true;
            std::cout << "Got serial data " << serial_data << "\n";
        }
        serial_data_received=0;
        mtx_serial_data.unlock();
#endif

        // **************** RECEIVE ***************
        for(int i=0;i<in_bytes;++i) in_buf[i]=0;
        int res=0;

        // Wait here for message
        hid_set_nonblocking(handle, 0);
        res = hid_read_timeout(handle, in_buf, in_bytes, 10);
        if(res==0) {
            if(!running) break; // we are quitting

            // Send a 0 to kick start the mbed to send
            std::cout << "Send inital report\n";
            pc_to_hid = {};
            unsigned char* msg_buf = reinterpret_cast<unsigned char*>(&pc_to_hid);
            int error = hid_write(handle,msg_buf,out_bytes);
            if(error!=out_bytes)
                std::cout << "initial hid_write return " << error << std::endl;
            continue; // new try receive
        }
        if(res==-1) std::cout << "Error reading from usb\n";
        receive_count_this_loop++;

        // Got message
        char* dataptr = reinterpret_cast<char*>(in_buf);
        hid_to_pc = *reinterpret_cast<hid_to_pc_message*>(dataptr);

        // If there are more messages in the queqe, skip
        // until the last one.
        hid_set_nonblocking(handle, 1);
        while((res=hid_read(handle, in_buf, sizeof(in_buf)))){
            if(res==-1) break;
            dataptr = reinterpret_cast<char*>(in_buf);
            hid_to_pc = *reinterpret_cast<hid_to_pc_message*>(dataptr);
            this_thread::sleep_for(1*microsecond);
            receive_count_this_loop++;
        }
        if(res==-1) std::cout << "Error reading from usb\n";

        if(receive_count_this_loop>1)
          std::cout << "additional read: " << receive_count_this_loop << " \n";


        // Check message
        if(hid_to_pc.info == 1){
             std::cout << "Calibration requested from hid! \n";
            tell_hid_to_calibrate = true;
        } else {
            tell_hid_to_calibrate = false;
        }

        // If encoder set by webserv, use that instead
        if(w->activeEnc5())
            hid_to_pc.encoder_d = short(w->getEnc5());
        else if(use_serial_for_enc5)
            hid_to_pc.encoder_d = enc5_serial;

        // *************** COMPUTE POSITION ***********
        // Compute position
        const int ch_a=PCB==POLHEM_USB?hid_to_pc.encoder_a:-hid_to_pc.encoder_a+offset_encoders[0]; // 2019-05-14 following new standard
        const int ch_b=PCB==POLHEM_USB?hid_to_pc.encoder_b:-hid_to_pc.encoder_b+offset_encoders[1];
        const int ch_c=PCB==POLHEM_USB?hid_to_pc.encoder_c:-hid_to_pc.encoder_c+offset_encoders[2];
        fsVec3d pos = kinematics.computePosition(ch_a,ch_b,ch_c);
        const int base[] = {ch_a, ch_b, ch_c};
        const int rot[]  = {PCB==POLHEM_USB?hid_to_pc.encoder_f:hid_to_pc.encoder_f+offset_encoders[5], // reverse order polhem
                            PCB==POLHEM_USB?hid_to_pc.encoder_e:hid_to_pc.encoder_e+offset_encoders[4],
                            PCB==POLHEM_USB?hid_to_pc.encoder_d:hid_to_pc.encoder_d+offset_encoders[3]}; // Encoder d,e,f here if recevied
        fsRot r = kinematics.computeRotation(base,rot);
        fsVec3d angles = kinematics.computeBodyAngles(base);
        mtx_pos.lock();
        raw_enc[0] = hid_to_pc.encoder_a;
        raw_enc[1] = hid_to_pc.encoder_b;
        raw_enc[2] = hid_to_pc.encoder_c;
        raw_enc[3] = hid_to_pc.encoder_d;
        raw_enc[4] = hid_to_pc.encoder_e;
        raw_enc[5] = hid_to_pc.encoder_f;
        latestBodyAngles = angles;
        latestPos = pos;
        latestRot = r;
        latestEnc[0]=ch_a;
        latestEnc[1]=ch_b;
        latestEnc[2]=ch_c;
        latestEnc[3]=rot[0];
        latestEnc[4]=rot[1];
        latestEnc[5]=rot[2];
        num_received_messages += receive_count_this_loop;
        mtx_pos.unlock();


        if(forced_inital_calibration){
            calibrate();
            forced_inital_calibration = false;
            continue;
        }


        firstMessage=true;



        // Inform blocking calls to getPos() that we now have a new position
        sem_getpos.post();

        //sem_setforce.wait();
        //while(sem_setforce.try_wait());



        // *************** GET WHAT TO SEND *****        
        // Compute amps to send
        fsVec3d amps;
        fsVec3d f;
        if(useCurrentDirectly){
            mtx_force.lock();
            amps = nextCurrent;
            mtx_force.unlock();
        } else {
            mtx_force.lock();
            f = nextForce;
            mtx_force.unlock();
            amps = kinematics.computeMotorAmps(f,base);
        }

        pc_to_hid={}; // Clear
        pc_to_hid.command = 0; // Default is send milliamps
        pc_to_hid.current_motor_a_mA = short(amps.x()*1000.0);
        pc_to_hid.current_motor_b_mA = short(amps.y()*1000.0);
        pc_to_hid.current_motor_c_mA = short(amps.z()*1000.0);

        max_milliamps=750;

        // Cap at 2A since Escons 24/4 cant do more than that for 4s
        if(pc_to_hid.current_motor_a_mA >= max_milliamps) pc_to_hid.current_motor_a_mA = max_milliamps-1;
        if(pc_to_hid.current_motor_b_mA >= max_milliamps) pc_to_hid.current_motor_b_mA = max_milliamps-1;
        if(pc_to_hid.current_motor_c_mA >= max_milliamps) pc_to_hid.current_motor_c_mA = max_milliamps-1;

        if(pc_to_hid.current_motor_a_mA <= -max_milliamps) pc_to_hid.current_motor_a_mA = -max_milliamps+1;
        if(pc_to_hid.current_motor_b_mA <= -max_milliamps) pc_to_hid.current_motor_b_mA = -max_milliamps+1;
        if(pc_to_hid.current_motor_c_mA <= -max_milliamps) pc_to_hid.current_motor_c_mA = -max_milliamps+1;


        if(tell_hid_to_calibrate){
            std::cout << "Tell hid to calibrate! \n";
            pc_to_hid.command = 1;
            pc_to_hid.command_attr0 = short(kinematics.m_config.calibrate_enc_a);
            pc_to_hid.command_attr1 = short(kinematics.m_config.calibrate_enc_b);
            pc_to_hid.command_attr2 = short(kinematics.m_config.calibrate_enc_c);
            pc_to_hid.current_motor_a_mA = short(kinematics.m_config.calibrate_enc_d);
            pc_to_hid.current_motor_b_mA = short(kinematics.m_config.calibrate_enc_e);
            pc_to_hid.current_motor_c_mA = short(kinematics.m_config.calibrate_enc_f);
        }


        // **************** SEND ***************
        unsigned char* msg_buf = reinterpret_cast<unsigned char*>(&pc_to_hid);
        res = hid_write(handle,msg_buf,out_bytes);
        if(res!=out_bytes)
            std::cout << "hid_write return " << res << std::endl;

        t2 = high_resolution_clock::now();

        mtx_pos.lock();
        if(pc_to_hid.command == 0){
            latestCommandedMilliamps[0] = pc_to_hid.current_motor_a_mA;
            latestCommandedMilliamps[1] = pc_to_hid.current_motor_b_mA;
            latestCommandedMilliamps[2] = pc_to_hid.current_motor_c_mA;
        }
        num_sent_messages++;
        currentForce = f;
        mtx_pos.unlock();


        // Set webserver info
        stringstream ss;
        ss << "\"DeviceName\": \"" << kinematics.m_config.name << "\",\n";
        ss << "\"Encoders\": [" << ch_a << ", " << ch_b << ", " << ch_c << ", " << rot[0]
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
    }


    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

    std::cout << "\n\nIt took me " << time_span.count() << " seconds.\n\n";

    //close HID device
    if(handle){
        hid_close(handle);
        hid_exit();
    }


}

void FsUSBHapticDeviceThread::close()
{
    FsHapticDeviceThread::close();
}

int FsUSBHapticDeviceThread::open()
{
    // Start normal thread
    FsHapticDeviceThread::open();

    std::chrono::duration<int, std::micro> microsecond{1};
    while(!firstMessage) this_thread::sleep_for(1000*microsecond);

    return 0;
}

#ifdef SERIAL_READ
void FsUSBHapticDeviceThread::usb_serial_thread()
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

void FsUSBHapticDeviceThread::calibrate()
{
    int calib[] = {int(kinematics.m_config.calibrate_enc_a),
                   int(kinematics.m_config.calibrate_enc_b),
                   int(kinematics.m_config.calibrate_enc_c),
                   int(kinematics.m_config.calibrate_enc_d),
                   int(kinematics.m_config.calibrate_enc_e),
                   int(kinematics.m_config.calibrate_enc_f)};

    if(PCB == POLHEM_USB)
        tell_hid_to_calibrate = true;
    else {
        // Since we are read-only, we only set it using offset.
        mtx_pos.lock();
        for(int i=0;i<6;++i)
            offset_encoders[i] = calib[i] + raw_enc[i];
        mtx_pos.unlock();
    }
}

}
