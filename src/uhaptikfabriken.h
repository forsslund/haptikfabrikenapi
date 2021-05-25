#ifndef UHAPTIKFABRIKEN_H
#define UHAPTIKFABRIKEN_H

#include <iostream>
#include <bitset>
#include <string>
//#ifdef LINUX
//#define USE_BT_PROXY
//#endif
//#define DUMMY_DEVICE
//#define VERBOSE
//#define USE_BT_PROXY_WIN
#define USE_BT_SOCKET




namespace uhaptikfabriken {
static const char* version = "0.2 2021-03-23";
constexpr int buf_len = 64;
}

// -----------------------------------------------------------------------------
// Haptikfabriken micro API v0.2
//
// This header-only micro API enables fast interaction with the Polhem haptic
// device developed by Haptikfabriken AB, over USB (and Blueetooth, via proxy app).
//
// In effect it opens a channel to the virtual serial port and sends forces
// (in clear text) to be rendered on the device and receives position and
// orientation data. This using the operating system's native calls.
// The communication code is currently verbatim from PJRC (Teensy) examples.
//
//Example usage
/*
        #include <iostream>
        #include "uhaptikfabriken.h"
        int main()
        {
            using namespace haptikfabriken;

            HaptikfabrikenInterface hfab;
            //hfab.serialport_name = "COM6";
            hfab.findUSBSerialDevices();  // Finds next free COM-port or /dev/ttySx
            hfab.open();

            while(true){
                fsVec3d p = hfab.getPos();
                std::cout << toString(p) << "\n";
                fsVec3d f = -200*p;
                hfab.setForce(f);
            }

            hfab.close();
        }
*/
// -----------------------------------------------------------------------------




// -----------------------------------------------------------------------------
// Bluetooth Socket Proxy (windows only version currently)
// -----------------------------------------------------------------------------
#ifdef  USE_BT_SOCKET
#include "SocketClient.h"
#endif



// -----------------------------------------------------------------------------
// Includes (from PJRC)
// -----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#if defined(MACOSX) || defined(LINUX)
#include <unistd.h> // usleep
#include <termios.h>
#include <sys/select.h>
#include <math.h>
#define PORTTYPE int
#define BAUD B115200
#if defined(LINUX)
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif
#elif defined(WINDOWS)
#ifndef _WINSOCKAPI_
#define DID_DEFINE_WINSOCKAPI
#define _WINSOCKAPI_
#endif
#include <windows.h>
#define PORTTYPE HANDLE
#define BAUD 115200
#else
#error "You must define the operating system\n"
#endif
// -----------------------------------------------------------------------------
#include <string>
#include <sstream>
#include <iomanip>
// -----------------------------------------------------------------------------


#ifdef USE_BT_PROXY
// ---------- Includes for unix sockets ------------
#include <sys/un.h>
#include <sys/socket.h>
#include <stdio.h>      /* Standard I/O functions */
#include <stdlib.h>     /* Prototypes of commonly used library functions,
                           plus EXIT_SUCCESS and EXIT_FAILURE constants */
#include <unistd.h>     /* Prototypes for many system calls */
#include <errno.h>      /* Declares errno and defines error constants */
// -------------------------------------------------
#endif








// -----------------------------------------------------------------------------
namespace uhaptikfabriken {

// ---- Socket hack code -----------
#define BUF_SIZE 10             /* Maximum size of messages exchanged
                                   between client and server */

#define SV_SOCK_PATH "/tmp/ud_ucase"

void errExit(const char *str) { printf("%s",str); exit(1);}
void fatal(const char *str) { printf("%s",str);  exit(1);}
void usageErr(const char *str) { printf("%s",str); exit(1);}


// ----------------------------------


// -----------------------------------------------------------------------------
// PJRC Code below
// -----------------------------------------------------------------------------
PORTTYPE open_port_and_set_baud_or_die(const char *name, long baud);
int transmit_bytes(PORTTYPE port, const char *data, int len);
int receive_bytes(PORTTYPE port, char *buf, int len);
void close_port(PORTTYPE port);
void delay(double sec);

void delay(double sec)
{
#if defined(MACOSX) || defined(LINUX)
    usleep(sec * 1000000);
#elif defined(WINDOWS)
    Sleep(DWORD(sec * 1000));
#endif
}

// wait for the Arduino board to boot up, since opening
// the board raises DTR, which resets the processor.
// as soon as it properly responds, we know it's running
void wait_online(PORTTYPE port)
{
    char buf[buf_len+1];
    int r;

    printf("waiting for board to be ready:\n");
    while (1) {
        delay(0.1);
        printf(".");
        fflush(stdout);
        buf[0] = '\n';
        r = transmit_bytes(port, buf, 1);
        r = receive_bytes(port, buf, 8);
        r = receive_bytes(port, buf, 8);
        r = receive_bytes(port, buf, 8);
        r = receive_bytes(port, buf, 8);
        r = receive_bytes(port, buf, 8);
        r = receive_bytes(port, buf, 8);
        r = receive_bytes(port, buf, 8);
        r = receive_bytes(port, buf, 8);
        if (r == 8) break; // success, device online
    }
    printf("ok\n");
}


PORTTYPE open_port_and_set_baud_or_die(const char *name, long baud)
{
    PORTTYPE fd;
#if defined(MACOSX)
    struct termios tinfo;
    fd = open(name, O_RDWR | O_NONBLOCK);
    if (fd < 0)printf("unable to open port %s\n", name);
    if (tcgetattr(fd, &tinfo) < 0)printf("unable to get serial parms\n");
    if (cfsetspeed(&tinfo, baud) < 0)printf("error in cfsetspeed\n");
    tinfo.c_cflag |= CLOCAL;
    if (tcsetattr(fd, TCSANOW, &tinfo) < 0)printf("unable to set baud rate\n");
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
#elif defined(LINUX)
    struct termios tinfo;
    struct serial_struct kernel_serial_settings;
    int r;
    fd = open(name, O_RDWR);
    if (fd < 0)printf("unable to open port %s\n", name);
    if (tcgetattr(fd, &tinfo) < 0)printf("unable to get serial parms\n");
    cfmakeraw(&tinfo);
    if (cfsetspeed(&tinfo, baud) < 0)printf("error in cfsetspeed\n");
    if (tcsetattr(fd, TCSANOW, &tinfo) < 0)printf("unable to set baud rate\n");
    r = ioctl(fd, TIOCGSERIAL, &kernel_serial_settings);
    if (r >= 0) {
        kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
        r = ioctl(fd, TIOCSSERIAL, &kernel_serial_settings);
        if (r >= 0) printf("set linux low latency mode\n");
    }
#elif defined(WINDOWS)
    COMMCONFIG cfg;
    COMMTIMEOUTS timeout;
    DWORD n;
    char portname[256];
    int num;
    if (sscanf_s(name, "COM%d", &num) == 1) {
        sprintf(portname, "\\\\.\\COM%d", num); // Microsoft KB115831
    } else {
        strncpy_s(portname, name, sizeof(portname)-1);
        //portname[n-1] = 0;
    }
    fd = CreateFileA(portname, GENERIC_READ | GENERIC_WRITE,
        0, 0, OPEN_EXISTING, 0, NULL);
    GetCommConfig(fd, &cfg, &n);
    cfg.dcb.BaudRate = baud;
    //cfg.dcb.BaudRate = 115200;
    cfg.dcb.fBinary = TRUE;
    cfg.dcb.fParity = FALSE;
    cfg.dcb.fOutxCtsFlow = FALSE;
    cfg.dcb.fOutxDsrFlow = FALSE;
    cfg.dcb.fOutX = FALSE;
    cfg.dcb.fInX = FALSE;
    cfg.dcb.fErrorChar = FALSE;
    cfg.dcb.fNull = FALSE;
    cfg.dcb.fRtsControl = RTS_CONTROL_ENABLE;
    cfg.dcb.fAbortOnError = FALSE;
    cfg.dcb.ByteSize = 8;
    cfg.dcb.Parity = NOPARITY;
    cfg.dcb.StopBits = ONESTOPBIT;
    cfg.dcb.fDtrControl = DTR_CONTROL_ENABLE;
    SetCommConfig(fd, &cfg, n);
    GetCommTimeouts(fd, &timeout);
    timeout.ReadIntervalTimeout = 0;
    timeout.ReadTotalTimeoutMultiplier = 0;
    timeout.ReadTotalTimeoutConstant = 1000;
    timeout.WriteTotalTimeoutConstant = 0;
    timeout.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(fd, &timeout);
#endif
    return fd;

}

int receive_bytes(PORTTYPE port, char *buf, int len)
{
    int count=0;
#if defined(MACOSX) || defined(LINUX)
    int r;
    int retry=0;
    //char buf[512];

    //if (len > sizeof(buf) || len < 1) return -1;
    // non-blocking read mode
    fcntl(port, F_SETFL, fcntl(port, F_GETFL) | O_NONBLOCK);
    while (count < len) {
        r = read(port, buf + count, len - count);
        //printf("read, r = %d (retry %d)\n", r, retry);
        if (r < 0 && errno != EAGAIN && errno != EINTR) return -1;
        else if (r > 0) count += r;
        else if(retry>500){
            //std::cout << "no data available right now, must wait\n";
            // no data available right now, must wait
            
            fd_set fds;
            struct timeval t;
            FD_ZERO(&fds);
            FD_SET(port, &fds);
            t.tv_sec = 0;
            t.tv_usec = 100;
            r = select(port+1, &fds, NULL, NULL, &t);
            //printf("select, r = %d\n", r);
            if (r < 0) return -1;
            if (r == 0) return count; // timeout
        }
        retry++;
        if (retry > 2000) return -100; // no input
    }
    fcntl(port, F_SETFL, fcntl(port, F_GETFL) & ~O_NONBLOCK);
#elif defined(WINDOWS)
    COMMTIMEOUTS timeout;
    DWORD n;
    BOOL r;
    int waiting=0;

    GetCommTimeouts(port, &timeout);
    timeout.ReadIntervalTimeout = MAXDWORD; // non-blocking
    timeout.ReadTotalTimeoutMultiplier = 0;
    timeout.ReadTotalTimeoutConstant = 0;
    SetCommTimeouts(port, &timeout);
    while (count < len) {
        r = ReadFile(port, buf + count, len - count, &n, NULL);
        if (n > 0) count += n;
        else {
            if (waiting) break;  // 1 sec timeout
            timeout.ReadIntervalTimeout = MAXDWORD;
            timeout.ReadTotalTimeoutMultiplier = MAXDWORD;
            timeout.ReadTotalTimeoutConstant = 1000;
            SetCommTimeouts(port, &timeout);
            waiting = 1;
        }
    }
#endif
    return count;
}


int transmit_bytes(PORTTYPE port, const char *data, int len)
{
#if defined(MACOSX) || defined(LINUX)
    return write(port, data, len);
#elif defined(WINDOWS)
    DWORD n;
    BOOL r;
    r = WriteFile(port, data, len, &n, NULL);
    if (!r) return 0;
    return n;
#endif
}


void close_port(PORTTYPE port)
{
#if defined(MACOSX) || defined(LINUX)
    close(port);
#elif defined(WINDOWS)
    CloseHandle(port);
#endif
}


// -----------------------------------------------------------------------------
// Haptikfabriken code
// -----------------------------------------------------------------------------
struct position_hid_to_pc_message {
    float x_mm{0};
    float y_mm{0};
    float z_mm{0};
    short a_ma{0};
    short b_ma{0};
    short c_ma{0};
    short info{0};
    float tA{0};
    float lambda{0};
    float tD{0};
    float tE{0};
    /*
    int toChars(char *c){
      // 012345678901234567890123456789012345678901234567890123456789123
      // -234.231 -234.121 -234.123 -1.2324 -1.1232 -1.1242 -1.2324 1
        int n = sprintf(c, "% 8.3f % 8.3f % 8.3f % 8.4f % 7.4f % 7.4f % 7.4f %hd\n", x_mm, y_mm, z_mm, tA, lambda, tD, tE, info);        while(n<62) c[n++] = ' ';
        c[62]='\n';
        return 63;
    }
    */

    int toChars(char *c){
         // 012345678901234567890123456789012345678901234567890123456789123
         //  -59.276   13.383 -148.002   0.0919 -1.8483  1.0370 -1.9512 0
         // -234.231 -234.121 -234.123 -1.2324 -1.1232 -1.1242 -1.2324 1
           int n = sprintf(c, "% 8.3f % 8.3f % 8.3f % 7.4f % 7.4f % 7.4f % 7.4f %hd\n", x_mm, y_mm, z_mm, tA, lambda, tD, tE, info);
            while(n<buf_len) c[n++] = ' ';
            c[buf_len-1]='\n';
            c[buf_len]=0;
            return buf_len;
       }


    void fromChars(const char *c){
        #ifdef WINDOWS 
            sscanf_s(c, "%f %f %f %f %f %f %f %hd", &x_mm, &y_mm, &z_mm, &tA, &lambda, &tD, &tE, &info);
        #else
            sscanf(c, "%f %f %f %f %f %f %f %hd", &x_mm, &y_mm, &z_mm, &tA, &lambda, &tD, &tE, &info);
        #endif
    }
    std::string toString() { char c[256]; for(int i=0;i<255;++i) c[i]=0; toChars(c); return std::string(c);}
};

struct pc_to_hid_message {  // 7*2 = 14 bytes + 1 inital byte always 0
  unsigned char reportid = 0;
  short current_motor_a_mA{0};
  short current_motor_b_mA{0};
  short current_motor_c_mA{0};
  short command{2}; // e.g. reset encoders
  short command_attr0{0};
  short command_attr1{0};
  short command_attr2{0};
  /*
  int toChars(char *c) const {
    int n = sprintf(c, "%hd %hd %hd %hd %hd %hd %hd", current_motor_a_mA, current_motor_b_mA,
                   current_motor_c_mA, command, command_attr0, command_attr1, command_attr2);
    while(n<62) c[n++] = ' ';
    c[62]='\n';
    return 63;
  }*/

  int toChars(char *c) const {
    int n = sprintf(c, "%hd %hd %hd %hd %hd %hd %hd", current_motor_a_mA, current_motor_b_mA,
                   current_motor_c_mA, command, command_attr0, command_attr1, command_attr2);
    while(n<buf_len) c[n++] = ' ';
    c[buf_len-1]='\n';
    c[buf_len]=0;
    return buf_len;
  }

  void fromChars(const char *c) {
#ifdef WINDOWS 
    sscanf_s(c, "%hd %hd %hd %hd %hd %hd %hd", &current_motor_a_mA, &current_motor_b_mA,
           &current_motor_c_mA, &command, &command_attr0, &command_attr1, &command_attr2);
#else
    sscanf(c, "%hd %hd %hd %hd %hd %hd %hd", &current_motor_a_mA, &current_motor_b_mA,
           &current_motor_c_mA, &command, &command_attr0, &command_attr1, &command_attr2);
#endif
  }
};


class PJRCSerialComm {// : public SerialComm {
public:
    void sendWakeupMessage(){pc_to_hid_message m; send(m);}
    void open(std::string portname);
    void close();
    void send(const pc_to_hid_message& msg);
    int receive(position_hid_to_pc_message& msg);
private:
    PORTTYPE fd;
    short last_info{0};
};

void PJRCSerialComm::open(std::string port) {
    fd = open_port_and_set_baud_or_die(port.c_str(), BAUD);
    wait_online(fd);
    char buf[1];
            int aa=0;
            int bb=0;
            while(bb=receive_bytes(fd, buf, 1)>0){aa+=bb;}
            printf("Flushed %d bytes\n",aa);
}

void PJRCSerialComm::close(){
    close_port(fd);
}

void printBuf(char* buf, std::string str, bool header=true){
    if(header)
        std::cout << str << " [0123456789012345678901234567890123456789012345678901234567890123]\n";
    std::cout << str << " [";
    int i;
    for(i=0;i<buf_len+1;++i){
        if(buf[i]==0) { std::cout << "#"; continue; };
        if(buf[i]=='\n') { std::cout << "N"; continue; }
        if(buf[i]=='\r') { std::cout << "R"; continue; }
        if(buf[i]==' ')  { std::cout << "_"; continue; }
        std::cout << buf[i];
    }
    std::cout << "] Size (including null character): " << i <<"\n";
}

void PJRCSerialComm::send(const pc_to_hid_message &msg)
{
    char buf[buf_len+1];
    memset(buf, '\0', buf_len+1);
    int len = msg.toChars(buf);
    //std::cout << len;
    //printBuf(buf, " send   ",false);
    //std::cout << "len " << len << "\n";
    transmit_bytes(fd, buf, len);
}

int PJRCSerialComm::receive(position_hid_to_pc_message &msg)
{
    constexpr int len=buf_len;
    char buf[len+1];
    memset(buf, '\0', len+1);
    int a = 0;
    int rr=0;
    int maxredo=10;
    for(a=0;a<64;a+=8){
        rr+=receive_bytes(fd, buf+a, 8);
        if(buf[a]==0){
            a-=8; // redo
            maxredo--;
            if(!maxredo){
                printf("Max redo!\n");
                // Search for \n
                int aa=0;
                int bb=0;
                while(bb=receive_bytes(fd, buf, 1)>0){aa+=bb;}
                printf("Flushed %d bytes!\n",aa);
                
                return 1;
            }
        }
    //printBuf(buf, " receive",false);
        }
    
    if(rr!=buf_len || buf[buf_len-1]!='\n' || buf[buf_len]!='\0'){
        std::cout << a;
       // printBuf(buf, " receive",false);

        if(buf[buf_len-1]!='\n'){
            // Search for \n
            int aa=0;
            int bb=0;
            while(bb=receive_bytes(fd, buf, 1)>0){aa+=bb;}
            printf("Flushed %d bytes!\n",aa);
        }

        return 1;
    }
    
    msg.fromChars(buf);
    if(msg.info != last_info){
        last_info=msg.info;
        std::cout << "New info: " << msg.info << "\n";
    }
    return 0;
}

struct fsVec3d {
    double m_x,m_y,m_z;
    fsVec3d(double x, double y, double z):m_x(x),m_y(y),m_z(z){}
    fsVec3d():m_x(0),m_y(0),m_z(0){}
    double x(){ return m_x; }
    double y(){ return m_y; }
    double z(){ return m_z; }
    void zero(){m_x=0;m_y=0;m_z=0;}
    double& operator[](int i) { if(i==0) return m_x; if(i==1) return m_y; return m_z;}
};
inline fsVec3d operator*(const double& d, const fsVec3d& v){ return fsVec3d(v.m_x*d,v.m_y*d,v.m_z*d); }
inline fsVec3d operator*(const fsVec3d& v, const double& d){ return fsVec3d(v.m_x*d,v.m_y*d,v.m_z*d); }
inline fsVec3d operator+(const fsVec3d& a, const fsVec3d& b){ return fsVec3d(a.m_x+b.m_x, a.m_y+b.m_y, a.m_z+b.m_z); }
inline fsVec3d operator-(const fsVec3d& a, const fsVec3d& b){ return fsVec3d(a.m_x-b.m_x, a.m_y-b.m_y, a.m_z-b.m_z); }


struct fsRot {
    double m[3][3];
    fsRot(){identity();}
    fsRot(double m[3][3]){set(m);}
    inline void set(double m[3][3]){
        for(int i=0;i<3;++i)
            for(int j=0;j<3;++j)
                this->m[i][j] = m[i][j];
    }
    fsRot(std::initializer_list<double> list){   // To initalize with list e.g.
        auto iter = list.begin();                //    fsRot r{1,2,3,
        for(int i=0;i<3;++i)                     //            4,5,6,
            for(int j=0;j<3;++j){                //            7,8,9};
                m[i][j]=*iter;
                iter++;
            }
    }
    void identity();
    void rot_x(double t);
    void rot_y(double t);
    void rot_z(double t);
    fsRot transpose();
};
fsVec3d operator*(const fsRot& m, const fsVec3d& v);
inline fsRot operator*(const fsRot& a, const fsRot& b) {
    fsRot c;
    int i,j,m;
    for(i=0;i<3;i++) {
      for(j=0;j<3;j++) {
        c.m[i][j] = 0;
        for(m=0;m<3;m++)
          c.m[i][j] += a.m[i][m]*b.m[m][j];
      }
    }
    return c;
}
inline fsRot operator*(const fsRot& a, const double& b) {
    fsRot c;
    int i,j;
    for(i=0;i<3;i++) {
      for(j=0;j<3;j++) {
         c.m[i][j] = a.m[i][j]*b;
      }
    }
    return c;
}
std::string toString(const fsVec3d& r);
std::string toString(const fsRot& r);

// Forward declare
class FsHapticDeviceThread;
class HaptikfabrikenInterface;

// Haptic values as passed to event handlers
struct HapticValues {
    fsVec3d position;
    fsRot orientation;
    fsVec3d currentForce;
    fsVec3d nextForce; // Will be output to device
};

/*
 *  Class for haptic event listener
 *  Example:
 *    class MyClass : public HapticListener {
 *           positionEvent(HapticValues hv){
 *               fsVec3d f = -100 * hv.position;
 *               hfab->setForce(f);
 *           }
 *
 *           MyClass(){
 *               HaptikfabrikenInterface* hfab = new HaptikfabrikenInterface();
 *               hfab->open();
 *               hfab->addEventListener(this);
 *           }
 *    }
 *
 */
class HapticListener {
public:
    virtual void positionEvent(HapticValues&) = 0;
    virtual ~HapticListener(){}
    int maxHapticListenerFrequency{15000};
};

class HaptikfabrikenInterface {
public:
    static std::string serialport_name;
    static std::string serialport_names[10];
    static unsigned int findUSBSerialDevices(); // sets serialport_name

    int open(std::string port=serialport_name); // Returns 0 if success
    void close();

#ifdef USE_BT_PROXY_WIN
    std::thread* t;
    bool run_t{ true };
    void httpProxyThread() {
        while (run_t) {
            try {
                auto res = cli->Get("/stylus1");
                if (res != nullptr) {
                    size_t offsett = 0;
                    bt_btn = stoi(res->body.substr(offsett), &offsett);
                    tF_count = stoi(res->body.substr(offsett), &offsett);
                }              
                //std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            catch(...) {

            }
        }
    }
#endif

    HaptikfabrikenInterface() {
#ifdef USE_BT_PROXY_WIN
        std::cout << "HaptikfabrikenInterface() created. \n";
        cli = new httplib::Client("http://192.168.1.49:8181");
        t = new std::thread(&HaptikfabrikenInterface::httpProxyThread, this);
#endif


    }



    // If error, get error message here
    std::string getErrorCode();


    // Get/set methods (thread safe, gets latest known values and sets next to be commanded)
    //void getEnc(int a[]);  // Array of 6 elements will be filled with signed encoder values
    //fsVec3d getBodyAngles();
    //void getLatestCommandedMilliamps(int ma[]); // Array of 3 elements
    //int getNumSentMessages(); // usb communications statistics
    //int getNumReceivedMessages();

    fsVec3d getPos();              // Get cartesian coords xyz using kineamtics model
    fsRot getRot();                // Get orientaion of manipulandum
    fsVec3d getCurrentForce();              // Get last applied force
    void setForce(fsVec3d f);      // Set force using kineamtics model, e.g. in xyz
    std::bitset<5> getSwitchesState(); // Get state of up to 5 manipulandum switches (via bt proxy)
    //void addEventListener(HapticListener* listener);
    //void removeEventListener(HapticListener* listener);

    position_hid_to_pc_message getMsg() const {
        return msg;
    }

private:
    PJRCSerialComm sc;
    position_hid_to_pc_message msg;
    fsVec3d lastSentForce;
    int get_minus_send_requests{ 0 };


#ifdef USE_BT_PROXY
    // Socket stuff
    struct sockaddr_un svaddr, claddr;
    int sfd, j;
    size_t msgLen;
    ssize_t numBytes;
    u_int8_t resp[BUF_SIZE];
#endif
#ifdef USE_BT_PROXY_WIN
    httplib::Client *cli;
    int bt_btn, tF_count;
#endif

#ifdef USE_BT_SOCKET
    SocketClient<uint16_t> btClient;
#endif
};

std::string HaptikfabrikenInterface::serialport_name;
std::string HaptikfabrikenInterface::serialport_names[10];

unsigned int HaptikfabrikenInterface::findUSBSerialDevices(){
#ifdef WINDOWS
    std::string candidates[]={"COM2","COM3","COM4","COM5",
                              "COM6","COM7","COM8","COM9","COM10"};
    int found_available_devices = 0;
    for (int i = 0; i < 9; ++i) {
        PORTTYPE fd = open_port_and_set_baud_or_die(candidates[i].c_str(), BAUD);
        if (fd != INVALID_HANDLE_VALUE) {
            serialport_name = candidates[i];
            close_port(fd);
            serialport_names[found_available_devices++] = serialport_name;
        }
    }
    return found_available_devices;
#else
    // TODO: Implement search
    serialport_name = "/dev/ttyACM0";
    serialport_names[0] = serialport_name;
    return 1;
#endif
    return 0;
}

std::string HaptikfabrikenInterface::getErrorCode() {
    return std::string("error");
}

int HaptikfabrikenInterface::open(std::string port){
#ifdef DUMMY_DEVICE
    std::cout << "Dummy device: open("<<port<<")\n";    
#else

    #ifdef WINDOWS
    if (port.length() == 0) {
        findUSBSerialDevices(); // Should update serialport_name
        port = serialport_name;
    }
    #endif

    sc.open(port);
    sc.sendWakeupMessage();
#endif

#ifdef USE_BT_SOCKET
    std::string addr = "/tmp/stylus-" + (port.find("/dev/") == std::string::npos? port : port.substr(5));

    btClient.Start(addr);
    std::cout <<"Listening for stylus on " << addr<<std::endl;
#endif
#ifdef USE_BT_PROXY    
    // Open BT
    sfd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sfd == -1)
        errExit("socket");
    memset(&claddr, 0, sizeof(struct sockaddr_un));
    claddr.sun_family = AF_UNIX;
    snprintf(claddr.sun_path, sizeof(claddr.sun_path),
            "/tmp/ud_ucase_cl.%ld", (long) getpid());
    if (bind(sfd, (struct sockaddr *) &claddr, sizeof(struct sockaddr_un)) == -1)
        errExit("bind");
    memset(&svaddr, 0, sizeof(struct sockaddr_un));
    svaddr.sun_family = AF_UNIX;
    strncpy(svaddr.sun_path, SV_SOCK_PATH, sizeof(svaddr.sun_path) - 1);
#endif   


    return 0;
}

void HaptikfabrikenInterface::close(){
    #ifdef DUMMY_DEVICE
    std::cout << "Dummy device: close()\n";
    return;
#endif

    sc.close();

#ifdef USE_BT_SOCKET
    btClient.Shutdown();
#endif
}

fsVec3d HaptikfabrikenInterface::getCurrentForce(){
    return fsVec3d();
}

fsVec3d HaptikfabrikenInterface::getPos(){
    #ifdef VERBOSE
        std::cout << "HaptikfabrikenInterface::getPos()... ";
    #endif

    // Check if device is waiting for a force command (ie. we have called getPos() twice)
    if (get_minus_send_requests>0) {
    #ifdef VERBOSE
        std::cout << "Note: get_minus_send_requests: " << get_minus_send_requests << "\n";
    #endif
        setForce(lastSentForce);
        get_minus_send_requests = 0;
    }
    get_minus_send_requests++;
    #ifdef DUMMY_DEVICE
        return fsVec3d(0.01, 0.02, 0.03);
    #endif
    position_hid_to_pc_message new_msg;
    if(sc.receive(new_msg)){ // Error reading, use 
        //pc_to_hid_message m;
        //position_hid_to_pc_message m;
        //msg = m;
        //sc.send(m);
        //getPos(); // try again
        std::cout << "in error reading\n";
    } else 
        msg = new_msg;
    #ifdef VERBOSE
        std::cout << "returns: " << msg.x_mm << " " << msg.y_mm << " " << msg.z_mm << "\n";
    #endif
    return fsVec3d(msg.x_mm*0.001,msg.y_mm*0.001,msg.z_mm*0.001);
}

std::bitset<5> HaptikfabrikenInterface::getSwitchesState(){
    bool btn = false;
#ifdef USE_BT_PROXY
    msgLen = 2;
    if (sendto(sfd, "ab", msgLen, 0, (struct sockaddr *) &svaddr,
        sizeof(struct sockaddr_un)) != msgLen)
        fatal("Could not access bt proxy server. Is it not started?\n");
    numBytes = recvfrom(sfd, resp, BUF_SIZE, 0, NULL, NULL);
    if (numBytes == -1)
        errExit("recvfrom");   
    btn = resp[0]&0x80;
#endif    
#ifdef USE_BT_PROXY_WIN
    btn = bt_btn;
#endif
#ifdef USE_BT_SOCKET
    uint16_t a = btClient.Get();
    btn = a & 0x80;
#endif
    std::bitset<5> switches;
    switches[0]=btn;
    return switches;
}

fsRot HaptikfabrikenInterface::getRot(){
    double tF = 0;
    // Get BT
#ifdef USE_BT_PROXY
    msgLen = 2;
    if (sendto(sfd, "ab", msgLen, 0, (struct sockaddr *) &svaddr,
        sizeof(struct sockaddr_un)) != msgLen)
        fatal("Could not access bt proxy server. Is it not started?\n");
    numBytes = recvfrom(sfd, resp, BUF_SIZE, 0, NULL, NULL);
    if (numBytes == -1)
        errExit("recvfrom");
   
    bool btn = resp[0]&0x80;
    int tF_count = (resp[0]&0x03) << 8 | resp[1]; // 10 bit rotation
    double tF = -2*3.1415926535897*tF_count / 1024;
    tF +=  3.1415926535897; // Define stylus button facing ceiling as up/default position
#else

#endif

#ifdef USE_BT_PROXY_WIN
    tF = -2 * 3.1415926535897 * tF_count / 1024;
    tF += 3.1415926535897; // Define stylus button facing ceiling as up/default position
#endif

#ifdef USE_BT_SOCKET    
    uint16_t value = btClient.Get();
    int tF_count = (value & 0x03) << 8 | (value >> 8) & 0xFF;
    tF = -2 * 3.1415926535897 * tF_count / 1024;
#endif

    //printf("Response %02x %02x n: %d btn: %d\n", resp[0],resp[1], tE_count, btn);

    fsRot rA, rC, rD, rE, rF;
    rA.rot_z(msg.tA);
    rC.rot_y(-msg.lambda);
    rD.rot_x(msg.tD);
    rE.rot_y(msg.tE);
    rF.rot_x(tF);

    return rA * rC * rD * rE * rF;
}

void HaptikfabrikenInterface::setForce(fsVec3d f){
    #ifdef VERBOSE
        std::cout << "HaptikfabrikenInterface::setForce(" << toString(f) << ")\n";
    #endif

    get_minus_send_requests--;
    if (get_minus_send_requests < 0) {
#ifdef VERBOSE
        std::cout << "Note, in set force: get_minus_send_requests: " << get_minus_send_requests << "\n";
#endif
        getPos();
    }
    lastSentForce = f;

#ifdef DUMMY_DEVICE
    return;
#endif
    pc_to_hid_message out;
    out.current_motor_a_mA = short(f.m_x*1000);
    out.current_motor_b_mA = short(f.m_y*1000);
    out.current_motor_c_mA = short(f.m_z*1000);
    sc.send(out);
}

void fsRot::identity() {
    double a[3][3] = { {1, 0, 0 },
                       {0, 1, 0 },
                       {0, 0, 1 }};
    set(a);
}
void fsRot::rot_x(double t){
    double a[3][3] = { {1,   0,       0    },
                       {0, cos(t), -sin(t) },
                       {0, sin(t), cos(t)  }};
    set(a);
}
void fsRot::rot_y(double t){
    double a[3][3] = { {cos(t),  0, sin(t) },
                       {   0,    1,   0    },
                       {-sin(t), 0, cos(t) }};
    set(a);
}
void fsRot::rot_z(double t){
    double a[3][3] = { {cos(t), -sin(t), 0 },
                       {sin(t), cos(t), 0 },
                       {0, 0, 1 }};
    set(a);
}

fsRot fsRot::transpose()
{
    double a[3][3] = { {m[0][0], m[1][0], m[2][0] },
                       {m[0][1], m[1][1], m[2][1] },
                       {m[0][2], m[1][2], m[2][2] }};
    fsRot r;
    r.set(a);
    return r;
}

fsVec3d operator*(const fsRot &m, const fsVec3d &v)
{
    fsVec3d r;

    r.m_x = m.m[0][0]*v.m_x + m.m[0][1]*v.m_y + m.m[0][2]*v.m_z;
    r.m_y = m.m[1][0]*v.m_x + m.m[1][1]*v.m_y + m.m[1][2]*v.m_z;
    r.m_z = m.m[2][0]*v.m_x + m.m[2][1]*v.m_y + m.m[2][2]*v.m_z;

    return r;
}

std::string toString(const haptikfabriken::fsVec3d &r)
{

    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    ss << std::setw(6) << r.m_x << ", " << std::setw(6) << r.m_y << ", " << std::setw(6) << r.m_z;
    return ss.str();
}

std::string toString(const haptikfabriken::fsRot &r)
{

    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    ss << std::setw(6) << r.m[0][0] << ", " << std::setw(6) << r.m[0][1] << ", " << std::setw(6) << r.m[0][2] << ",\n";
    ss << std::setw(6) << r.m[1][0] << ", " << std::setw(6) << r.m[1][1] << ", " << std::setw(6) << r.m[1][2] << ",\n";
    ss << std::setw(6) << r.m[2][0] << ", " << std::setw(6) << r.m[2][1] << ", " << std::setw(6) << r.m[2][2] << "\n";
    return ss.str();
}
// -----------------------------------------------------------------------------


} // Namespace uhaptikfabriken
#endif // UHAPTIKFABRIKEN_H
