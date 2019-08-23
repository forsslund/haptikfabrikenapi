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

constexpr int WOODENHAPTICS = 1;
constexpr int POLHEM_USB = 2;

constexpr int PCB = POLHEM_USB;

namespace haptikfabriken {


FsUSBHapticDeviceThread::FsUSBHapticDeviceThread(bool wait_for_next_message,
                                                 Kinematics::configuration c):
    FsHapticDeviceThread::FsHapticDeviceThread(wait_for_next_message,c),w(0)
{

}

void FsUSBHapticDeviceThread::thread()
{
    // Start web server
    if(w==0){
        w = new Webserv();
        w->initialize();
    }


    // For debugging
    num_sent_messages = 0;
    num_received_messages = 0;

    // Set protocol 1=Old usb, 2=April 2018, and for POLHEM
    int protocol_version = PCB==WOODENHAPTICS ? 1 : 2;

    // Open connection
    devs = hid_enumerate(0x0, 0x0);
    cur_dev = devs;
    while (cur_dev) {
        printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
        printf("\n");
        printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
        printf("  Product:      %ls\n", cur_dev->product_string);
        printf("  Release:      %hx\n", cur_dev->release_number);
        printf("  Interface:    %d\n",  cur_dev->interface_number);
        printf("\n");
        cur_dev = cur_dev->next;
    }
    hid_free_enumeration(devs);

    // Open the device using the VID, PID,
    // and optionally the Serial number.
    handle = hid_open(0x1234, 0x6, nullptr);
    if (!handle) {
        std::cout << "unable to open device. Is it plugged in and you run as root?\n";
        //return 1;
    }
    // Set the hid_read() function to be non-blocking.
    if(handle)
        hid_set_nonblocking(handle, 1);

    if(handle)
        std::cout << "Opened USB Connection" << std::endl;


    bool forced_inital_calibration = PCB==WOODENHAPTICS ? true : false;
    tell_hid_to_calibrate = false;

    while(running){
        if(!handle) break;


        // **************** RECEIVE ***************
        int res=0;
        int count=0;
        for(int i=0;i<15;++i) buf[i]=0;
        while (res == 0) {
            res = hid_read(handle, buf, sizeof(buf));
            char* dataptr = reinterpret_cast<char*>(buf);
            if(res==14 || res==8) // Got a correct message
                hid_to_pc = *reinterpret_cast<hid_to_pc_message*>(dataptr);
            if(res==8){
                hid_to_pc.encoder_d=0;
                hid_to_pc.encoder_e=0;
                hid_to_pc.encoder_f=0;
            }
            usleep(5);
            count++;
            //if(!(count%1000)) std::cout << "(count: " << count << ")\n";
        }
        //std::cout << "count: " << count << "\n";

        int flush=0;
        while(hid_read(handle, buf, sizeof(buf)) && running){
            char* dataptr = reinterpret_cast<char*>(buf);
            if(res==14 || res==8) // Got a correct message
                hid_to_pc = *reinterpret_cast<hid_to_pc_message*>(dataptr);
            if(res==8){
                hid_to_pc.encoder_d=0;
                hid_to_pc.encoder_e=0;
                hid_to_pc.encoder_f=0;
            }
            ++flush;
        }
        //if(flush)
        //    std::cout << "Flushed " << flush << " messages." << std::endl;
        //lost_messages += flush;



        // Check message
        if(hid_to_pc.info == 1){
             std::cout << "Calibration requested from hid! \n";
            tell_hid_to_calibrate = true;
        } else {
            tell_hid_to_calibrate = false;
        }




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
        num_received_messages += 1 + flush;
        mtx_pos.unlock();



        if(forced_inital_calibration){
            calibrate();
            forced_inital_calibration = false;
            continue;
        }





        // *************** GET WHAT TO SEND *****
        mtx_force.lock();
        fsVec3d f = nextForce;
        mtx_force.unlock();

        fsVec3d amps = kinematics.computeMotorAmps(f,base);

        if(useCurrentDirectly){
            mtx_force.lock();
            amps = nextCurrent;
            mtx_force.unlock();
        }

        pc_to_hid.current_motor_a_mA = short(amps.x()*1000.0);
        pc_to_hid.current_motor_b_mA = short(amps.y()*1000.0);
        pc_to_hid.current_motor_c_mA = short(amps.z()*1000.0);

        // Cap at 2A since Escons 24/4 cant do more than that for 4s
        if(pc_to_hid.current_motor_a_mA >= max_milliamps) pc_to_hid.current_motor_a_mA = max_milliamps-1;
        if(pc_to_hid.current_motor_b_mA >= max_milliamps) pc_to_hid.current_motor_b_mA = max_milliamps-1;
        if(pc_to_hid.current_motor_c_mA >= max_milliamps) pc_to_hid.current_motor_c_mA = max_milliamps-1;

        if(pc_to_hid.current_motor_a_mA <= -max_milliamps) pc_to_hid.current_motor_a_mA = -max_milliamps+1;
        if(pc_to_hid.current_motor_b_mA <= -max_milliamps) pc_to_hid.current_motor_b_mA = -max_milliamps+1;
        if(pc_to_hid.current_motor_c_mA <= -max_milliamps) pc_to_hid.current_motor_c_mA = -max_milliamps+1;


        pc_to_hid.command = 0; // Default is send milliamps



        // **************** SEND ***************

        // Fake data
        //pc_to_hid.current_motor_a_mA=1234;
        //pc_to_hid.current_motor_b_mA=5678;
        //pc_to_hid.current_motor_c_mA=9012;
        //pc_to_hid.force_motor_a_N=1111;
        //pc_to_hid.force_motor_b_N=2222;
        //pc_to_hid.force_motor_c_N=3333;
        //pc_to_hid.debug=4444;

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

        unsigned char* msg_buf = reinterpret_cast<unsigned char*>(&pc_to_hid);


        int byte_length = 9;  // Old protocol
        if(protocol_version==2)
            byte_length = 15; // New protocol April 2018


        //Fill the report
        unsigned char out_buf[15]; // const max of 9 and 15
        out_buf[0] = 0;
        for (int i = 1; i < byte_length; i++) {
            out_buf[i] = msg_buf[i-1];
        }
        if(handle){
            //std::cout << "size: " << sizeof(out_buf) << "\n";
            int error = hid_write(handle,out_buf,sizeof(out_buf));
            if(error!=byte_length){
                std::cout << "hid_write return " << error << std::endl;
            }
        }

        mtx_pos.lock();
        if(pc_to_hid.command == 0){
            latestCommandedMilliamps[0] = pc_to_hid.current_motor_a_mA;
            latestCommandedMilliamps[1] = pc_to_hid.current_motor_b_mA;
            latestCommandedMilliamps[2] = pc_to_hid.current_motor_c_mA;
        }
        num_sent_messages++;
        mtx_pos.unlock();

        // SENT
        currentForce = f;


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
