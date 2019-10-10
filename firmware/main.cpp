/*
 *  Polhem USB HID interface based on the woodenhapitcs hid
 *  code that was updated and tested 2019-05-08 and commited
 *  to Github woody-experimental/mbed-code (and here WoodenHapticsHID)
 *
 *
 *  Relased as open source under GNU GPL v.3 or later
 *  Note: any distribution of binaries (or hardware)
 *  Requires this source file, including any modifications,
 *  as well as the attribution mentioned above.
 *
 *  Updated and tested 2019-10-10 by Jonas Forsslund.
 *  jonas@forsslundsystems.com
 *
 */
 
 
 // PCB Version
 #define PCB_WOODENHAPTICS_0_4
 //#define PCB_POLHEM_V3
  

//
// Uncomment to enable 6-DOF input, i.e. gimbal encoders.
// If you have no encoders connected, leave it comment out or 
// you may get random readings and it will take a lot of cpu.
//
//#define DOF6 

// Set the current which represent max pwm duty cycle. 
// Reducing this improves the resolution of the current. 
// SHOULD MATCH the current at 90% in the Escon config used.
//const float max_current = 6.0f; 
const float max_current = 3.0f; 
 
 
#include "mbed.h"
#include "USBHID.h"
#include "FastPWM.h"

// The LPC1768 built-in USB port configured as a 
// virtual com port for serial communication
// /dev/ttyACM0 in Linux, COM3 or similar in Windows.
// This is only used for debug by default
// Alternatively we can use it for outgoing traffic,
// however, tests 20190910 showed no real turn-around performance increase
Serial pc(USBTX, USBRX);

// The motherboard usb configured as HID
const int bytes_in = 14;
const int bytes_out = 14;
USBHID hid(bytes_out, bytes_in);
HID_REPORT send_report;
HID_REPORT recv_report;

#pragma pack(1)  // Since we are transferring them as is.
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

// Timeout watchdog if no message have arrived
Timeout msg_watchdog;

// LEDS for debug/status
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);

// Escon communication (pwm, enable) 
// Yipee no direction pin anymore!
#ifdef PCB_POLHEM_V3
DigitalOut enableEscons(p27,0); 
#endif
#ifdef PCB_WOODENHAPTICS_0_4
DigitalOut enableEscons(p14,0); 
#endif
PwmOut pwm[3]={p21,p22,p23};
void escon_timeout() {
    enableEscons = 0;
    for(int i=0;i<3;i++)
        pwm[i].write(0.5); //50% means 0
}

// Button/switches (used for calibration request)
InterruptIn  ix0_button(p28); // "TX" general IO9 on WoodenHaptics 0.4
bool ix0_button_flag = false;
void callback_ix0_button_fall(void) { ix0_button_flag = true; myled4=1;}

// Encoders pins
#ifdef PCB_POLHEM_V3
InterruptIn  encoder0_A(p5);
InterruptIn  encoder0_B(p6);
InterruptIn  encoder1_A(p10);
InterruptIn  encoder1_B(p9);
InterruptIn  encoder2_A(p8);
InterruptIn  encoder2_B(p7);

#ifdef DOF6
InterruptIn  encoder3_A(p14);
InterruptIn  encoder3_B(p13);
InterruptIn  encoder4_A(p12);
InterruptIn  encoder4_B(p11);
InterruptIn  encoder5_A(p29);
InterruptIn  encoder5_B(p30);
#endif 
#endif

#ifdef PCB_WOODENHAPTICS_0_4
InterruptIn  encoder0_A(p5);
InterruptIn  encoder0_B(p7);
InterruptIn  encoder1_A(p8);
InterruptIn  encoder1_B(p10);
InterruptIn  encoder2_A(p11);
InterruptIn  encoder2_B(p13);
#endif

// Encoder logic
int prev_state[] = {-1,-1,-1,-1,-1,-1};
bool encoder_raw[6][2] = {{false,false},{false,false},{false,false},
                          {false,false},{false,false},{false,false}};
int counter[] = {0,0,0,0,0,0};

// AB channel switch table
// 
// Pre  Cur  Dir  Dec   
// 0 0  0 1  +     1
// 0 0  1 0  -     2
// 0 1  1 1  +     7
// 0 1  0 0  -     4
// 1 1  1 0  +    14
// 1 1  0 1  -    13
// 1 0  0 0  +     8
// 1 0  1 1  -    11
//
//                        0  1 2 3 4 5 6  7  8 9 10 11 12 13 14 15
//const int stable[16] = {0,-1,1,0,1,0,0,-1,-1,0, 0, 1, 0, 1,-1, 0}; // Original 
const int stable[16]   = {0,1,-1,0,-1,0,0,1,1,0, 0, -1, 0, -1,1, 0}; // Reverse
void encoder_callback(int _encoder,int AB,bool value){
        int cur_state;
        encoder_raw[_encoder][AB]=value;
        cur_state = encoder_raw[_encoder][0] << 1 | encoder_raw[_encoder][1];
        if(prev_state[_encoder] < 0) prev_state[_encoder] = cur_state;        
        counter[_encoder] += stable[prev_state[_encoder] << 2 | cur_state];
        prev_state[_encoder]=cur_state;
}

// "callback stubs"
void callback_0_A_rise(void) { encoder_callback(0,0,true);}
void callback_0_A_fall(void) { encoder_callback(0,0,false);}
void callback_0_B_rise(void) { encoder_callback(0,1,true);}
void callback_0_B_fall(void) { encoder_callback(0,1,false);}

void callback_1_A_rise(void) { encoder_callback(1,0,true);}
void callback_1_A_fall(void) { encoder_callback(1,0,false);}
void callback_1_B_rise(void) { encoder_callback(1,1,true);}
void callback_1_B_fall(void) { encoder_callback(1,1,false);}

void callback_2_A_rise(void) { encoder_callback(2,0,true);}
void callback_2_A_fall(void) { encoder_callback(2,0,false);}
void callback_2_B_rise(void) { encoder_callback(2,1,true);}
void callback_2_B_fall(void) { encoder_callback(2,1,false);}

void callback_3_A_rise(void) { encoder_callback(3,0,true);}
void callback_3_A_fall(void) { encoder_callback(3,0,false);}
void callback_3_B_rise(void) { encoder_callback(3,1,true);}
void callback_3_B_fall(void) { encoder_callback(3,1,false);}

void callback_4_A_rise(void) { encoder_callback(4,0,true);}
void callback_4_A_fall(void) { encoder_callback(4,0,false);}
void callback_4_B_rise(void) { encoder_callback(4,1,true);}
void callback_4_B_fall(void) { encoder_callback(4,1,false);}

void callback_5_A_rise(void) { encoder_callback(5,0,true);}
void callback_5_A_fall(void) { encoder_callback(5,0,false);}
void callback_5_B_rise(void) { encoder_callback(5,1,true);}
void callback_5_B_fall(void) { encoder_callback(5,1,false);}


int main(void) {
    
    // Flash leds to indicate we are ready
    myled1 = 1;   
    myled2 = 1;
    myled3 = 1;
    myled4 = 1;
    wait_ms(500);     
    myled1 = 0;
    myled2 = 0;
    myled3 = 0;
    myled4 = 0;
    wait_ms(500); 
       
    // Setup everything
    pc.baud(115200);
    ix0_button.mode(PullUp);
    ix0_button.fall(&callback_ix0_button_fall);
    
    encoder0_A.rise(&callback_0_A_rise);
    encoder0_A.fall(&callback_0_A_fall);
    encoder0_B.rise(&callback_0_B_rise);
    encoder0_B.fall(&callback_0_B_fall);
    
    encoder1_A.rise(&callback_1_A_rise);
    encoder1_A.fall(&callback_1_A_fall);
    encoder1_B.rise(&callback_1_B_rise);
    encoder1_B.fall(&callback_1_B_fall);

    encoder2_A.rise(&callback_2_A_rise);
    encoder2_A.fall(&callback_2_A_fall);
    encoder2_B.rise(&callback_2_B_rise);
    encoder2_B.fall(&callback_2_B_fall);
    
#ifdef DOF6
    encoder3_A.rise(&callback_3_A_rise);
    encoder3_A.fall(&callback_3_A_fall);
    encoder3_B.rise(&callback_3_B_rise);
    encoder3_B.fall(&callback_3_B_fall);

    encoder4_A.rise(&callback_4_A_rise);
    encoder4_A.fall(&callback_4_A_fall);
    encoder4_B.rise(&callback_4_B_rise);
    encoder4_B.fall(&callback_4_B_fall);

    encoder5_A.rise(&callback_5_A_rise);
    encoder5_A.fall(&callback_5_A_fall);
    encoder5_B.rise(&callback_5_B_rise);
    encoder5_B.fall(&callback_5_B_fall);
#endif

    enableEscons = 0;
    
    for(int i=0;i<3;i++){
        pwm[i].period_us(500); // 500=2khz, 250=4khz 200=5khz
        pwm[i].write(0.5);  
    }           
    send_report.length = bytes_out; 
    
    hid_to_pc_message hid_to_pc;
    pc_to_hid_message pc_to_hid;
       
    Timer message_timeout;
    message_timeout.start();
    
    while (1) {        
        hid_to_pc.encoder_a = counter[0];
        hid_to_pc.encoder_b = counter[1];
        hid_to_pc.encoder_c = counter[2];
        hid_to_pc.encoder_d = counter[3];
        hid_to_pc.encoder_e = counter[4];
        hid_to_pc.encoder_f = counter[5];
        hid_to_pc.info = ix0_button_flag; // flagged until cleared (calibrated)
        
        unsigned char* out_buf = reinterpret_cast<unsigned char*>(&hid_to_pc);

        //Fill the report
        for (unsigned int i = 0; i < send_report.length; i++)
            send_report.data[i] = out_buf[i];
        
        // Send the report with HID. Blocking. USB standard dictates that 
        // the host (pc) will regularly retrieve data from device. 
        hid.send(&send_report);      

        //try to read a msg, timeout after 10x0.1ms 
        hid.read(&recv_report);
            
        // Make sure we read the latest if multiple messages have been sent
        while(hid.readNB(&recv_report));

        if(recv_report.length == bytes_in){ // It should always be!                

            msg_watchdog.detach();
            msg_watchdog.attach(&escon_timeout,0.5);
            enableEscons = 1;

            // Reinterprete the recevied byte array as a pc_to_hid object.
            // Since recv_report.data is defined as unsigned char we
            // have to cast to char* to avoid strict aliasing warning.
            char* dataptr = reinterpret_cast<char*>(recv_report.data);
            pc_to_hid = *reinterpret_cast<pc_to_hid_message*>(dataptr);            
            double f[] = {0,0,0};
            
            // Check for commands
            if(pc_to_hid.command == 1){
                counter[0] = pc_to_hid.command_attr0;
                counter[1] = pc_to_hid.command_attr1;
                counter[2] = pc_to_hid.command_attr2;
                counter[3] = pc_to_hid.current_motor_a_mA;
                counter[4] = pc_to_hid.current_motor_b_mA;
                counter[5] = pc_to_hid.current_motor_c_mA;
                // flag off switch
                ix0_button_flag = false;
                myled4=0;
            } else {                                       
                f[0] = pc_to_hid.current_motor_a_mA*0.001;
                f[1] = pc_to_hid.current_motor_b_mA*0.001;
                f[2] = pc_to_hid.current_motor_c_mA*0.001;
            }
            
            for(int i=0;i<3;i++)
                pwm[i].write(0.4*f[i]/max_current+0.5);                            
        }                        
    }
}
