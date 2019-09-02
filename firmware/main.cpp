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
 *  Updated and tested 2019-08-23 by Jonas Forsslund.
 *  jonas@forsslundsystems.com
 *
 */

#include "mbed.h"
#include "USBHID.h"
#include "FastPWM.h"

const int bytes_in = 14;
const int bytes_out = 14;

//We declare a USBHID device. Input out output reports have a length of x bytes
USBHID hid(bytes_out, bytes_in);
 
//This report will contain data to be sent
HID_REPORT send_report;
HID_REPORT recv_report;

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);

DigitalOut enableEscons(p27,0); 
 
InterruptIn  encoder0_A(p5);
InterruptIn  encoder0_B(p6);
InterruptIn  encoder1_A(p10);
InterruptIn  encoder1_B(p9);
InterruptIn  encoder2_A(p8);
InterruptIn  encoder2_B(p7);
InterruptIn  encoder3_A(p14);
InterruptIn  encoder3_B(p13);
InterruptIn  encoder4_A(p12);
InterruptIn  encoder4_B(p11);
InterruptIn  encoder5_A(p29);
InterruptIn  encoder5_B(p30);

InterruptIn  ix0_button(p28);
//DigitalIn ix0_button_DI(p28);


Timeout msg_watchdog;

int no_enc = 6;

int prev_state[] = {-1,-1,-1,-1,-1,-1};
bool encoder_raw[6][2] = {{false,false},{false,false},{false,false},
                          {false,false},{false,false},{false,false}};

PwmOut pwm[3]={p21,p22,p23};
PwmOut direction[3]={p24,p25,p26};

int counter[] = {0,0,0,0,0,0};

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
//                0  1 2 3 4 5 6  7  8 9 10 11 12 13 14 15
//int stable[16] = {0,-1,1,0,1,0,0,-1,-1,0, 0, 1, 0, 1,-1, 0}; // Original 
int stable[16]   = {0,1,-1,0,-1,0,0,1,1,0, 0, -1, 0, -1,1, 0}; // Reverse
 
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

bool ix0_button_flag = false;
void callback_ix0_button_fall(void) { ix0_button_flag = true; myled1=1; }
 
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

struct pc_to_hid_message {  // 7*2 = 14 bytes
    short current_motor_a_mA;
    short current_motor_b_mA;
    short current_motor_c_mA;
    short command; // e.g. reset encoders
    short command_attr0;
    short command_attr1;
    short command_attr2;
};

void escon_timeout() {
    enableEscons = 0;
    myled2 = 0;   
    myled3 = 0;
    myled4 = 0;
        
    for(int i=0;i<3;i++){
        pwm[i].write(0.1); //10% means 0  
        direction[i].write(0);
    }        
}
 
int main(void) {
    myled1 = 1;   // SETUP
    myled2 = 1;
    myled3 = 1;
    myled4 = 1;  
    
    wait_ms(500); 
    myled1 = 0;   // SETUP
    myled2 = 0;
    myled3 = 0;
    myled4 = 0;  

    ix0_button.mode(PullUp);
    
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

    ix0_button.fall(&callback_ix0_button_fall);


    enableEscons = 0;
    
    for(int i=0;i<3;i++){
        pwm[i].period_us(500);
        pwm[i].write(0.1);  
        direction[i].period_us(500);
        direction[i].write(0);
    }
           
    send_report.length = bytes_out; 
    
    hid_to_pc_message hid_to_pc;
    pc_to_hid_message pc_to_hid;
       
    Timer usb_timer;
    usb_timer.start();

    Timer debug_t;
    debug_t.start();
    
    Timer message_timeout;
    message_timeout.start();
    
    while (1) {
        //myled1 = 1;

        //try to read a msg
        if(hid.read(&recv_report)) {
            
            // Make sure we read the latest
            while(hid.readNB(&recv_report));
            myled3 = !myled3; // We got data            

            if(recv_report.length == bytes_in){ // It should always be!                

                msg_watchdog.detach();
                msg_watchdog.attach(&escon_timeout,0.5);
                enableEscons = 1;
                myled2 = 1;

                // Reinterprete the recevied byte array as a pc_to_hid object.
                // Since recv_report.data is defined as unsigned char we
                // have to cast to char* to avoid strict aliasing warning.
                char* dataptr = reinterpret_cast<char*>(recv_report.data);
                pc_to_hid = *reinterpret_cast<pc_to_hid_message*>(dataptr);
                
                float f[] = {0,0,0};
                
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
                } else {                                       
                    f[0] = pc_to_hid.current_motor_a_mA*0.001f;
                    f[1] = pc_to_hid.current_motor_b_mA*0.001f;
                    f[2] = pc_to_hid.current_motor_c_mA*0.001f;
                }
                
                for(int i=0;i<3;i++){
                    int dir = f[i] > 0 ? 1 : 0;
                    direction[i].write(dir);
                    float abs_val = f[i]<0? -f[i] : f[i];
                    
                    float pwm_90_percent = i==0? 1.0 : 2.0; // First axis 90%=1A 
                    
                    if(abs_val > pwm_90_percent)
                        pwm[i].write(0.9);
                    else
                        pwm[i].write(0.8*abs_val/pwm_90_percent+0.1);
                }
            }                
            //myled1 = 1;
        }
        
        // Read data and send!

        hid_to_pc.encoder_a = counter[0];
        hid_to_pc.encoder_b = counter[1];
        hid_to_pc.encoder_c = counter[2];
        hid_to_pc.encoder_d = counter[3];
        hid_to_pc.encoder_e = counter[4];
        hid_to_pc.encoder_f = counter[5];
        hid_to_pc.info = ix0_button_flag; // flagged until cleared (calibrated)
        
        //if(usb_timer.read() > 0.002) {
        //    usb_timer.reset();

            //hid_to_pc.debug++;
            unsigned char* out_buf = reinterpret_cast<unsigned char*>(&hid_to_pc);
    
            //Fill the report
            for (unsigned int i = 0; i < send_report.length; i++) {
                send_report.data[i] = out_buf[i];
            }


                
            //Send the report
            hid.sendNB(&send_report);      
            myled4 = !myled4;
            
        //wait(0.0005);
            
        //}
    }
}

