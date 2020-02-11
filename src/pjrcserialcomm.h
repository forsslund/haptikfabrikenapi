#ifndef PJRCSERIALCOMM_H
#define PJRCSERIALCOMM_H

// One of these must be defined, usually via the Makefile
//#define MACOSX
//#define LINUX
//#define WINDOWS

// Based on code from latency_test.zip by Paul Stoffengren
// https://forum.pjrc.com/threads/7826-USB-to-digital-I-O-delay


//#define BOOST_ASIO
#define PJRC
//#define PJRC_LATENCY_TEST

#include "fsusbhapticdevicethread.h" // For SerialComm class interface and pc_to_hid_message



namespace haptikfabriken {




#ifdef PJRC
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <errno.h>


#if defined(MACOSX) || defined(LINUX)
#include <termios.h>
#include <sys/select.h>
#define PORTTYPE int
#define BAUD B115200
#if defined(LINUX)
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif
#elif defined(WINDOWS)
#include <windows.h>
#define PORTTYPE HANDLE
#define BAUD 115200
#else
#error "You must define the operating system\n"
#endif

// function prototypes
PORTTYPE open_port_and_set_baud_or_die(const char *name, long baud);
int transmit_bytes(PORTTYPE port, const char *data, int len);
int receive_bytes(PORTTYPE port, char *buf, int len);
void close_port(PORTTYPE port);
void delay(double sec);

// C++ timing
#include <chrono>
using namespace std;
using namespace std::chrono;




/************************************/
/*  Latency/Delay Test  Functions   */
/************************************/

double do_test(PORTTYPE port)
{
    high_resolution_clock::time_point t1, t2;
    constexpr int len=64;
    char buf[65];
    //struct timeval begin, end;
    double elapsed;
    int r;

    memset(buf, '0', len);
    buf[len - 1] = '\n'; // end of packet marker
    t1 = high_resolution_clock::now();

    // send the data
    r = transmit_bytes(port, buf, len);

    // receive the reply
    r = receive_bytes(port, buf, 64);  // response is always 4 bytes
    buf[64]='\0';

    //std::cout << "[" << buf << "]\n";//hid_to_pc.fromChars(buf);
    //for(int i=0;i<64;++i){
    //    std::cout << i << ": " << int(buf[i]) <<  " " << char(buf[i])  << "\n";
    //}

    // test end
    //gettimeofday(&end, NULL);
    t2 = high_resolution_clock::now();

    //elapsed = (double)(end.tv_sec - begin.tv_sec) * 1000.0;
    //elapsed += (double)(end.tv_usec - begin.tv_usec) / 1000.0;
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    elapsed = time_span.count()*1000;

    //printf("  len=%d, elased: %.2f ms\n", len, elapsed);
    return elapsed;
}


void do_test_100_times(PORTTYPE port)
{
    const int num = 10000;
    double ms, total=0, max=0;
    int i;

    printf("latency @ 64 bytes: \n");
    for (i=0; i<num; i++) {
        ms = do_test(port);
        total += ms;
        if (ms > max) max = ms;
    }
    printf("%.2f ms average, ", total / num);
    printf("%.2f maximum\n", max);
}



// wait for the Arduino board to boot up, since opening
// the board raises DTR, which resets the processor.
// as soon as it properly responds, we know it's running
void wait_online(PORTTYPE port)
{
    char buf[64];
    int r;

    printf("waiting for board to be ready:\n");
    while (1) {
        delay(0.1);
        printf(".");
        fflush(stdout);
        buf[0] = '\n';
        r = transmit_bytes(port, buf, 1);
        r = receive_bytes(port, buf, 64);
        std::cout << r << std::endl;
        if (r == 64) break; // success, device online
    }
    printf("ok\n");
}





class PJRCSerialComm : public SerialComm {
    void open(string portname);
    void close();
    void send(const pc_to_hid_message& msg);
    void receive(hid_to_pc_message& msg);
private:
    PORTTYPE fd;
};



void PJRCSerialComm::open(string port) {
    fd = open_port_and_set_baud_or_die(port.c_str(), BAUD);
    wait_online(fd);
}

void PJRCSerialComm::close(){
    close_port(fd);
}

void PJRCSerialComm::send(const pc_to_hid_message &msg)
{
    int len=64;
    char buf[64];
    memset(buf, '0', len);
    len = msg.toChars(buf);
    transmit_bytes(fd, buf, len);
}

void PJRCSerialComm::receive(hid_to_pc_message &msg)
{
    constexpr int len=64;
    char buf[len];
    memset(buf, '0', len);
    receive_bytes(fd, buf, len);
    msg.fromChars(buf);
}


#ifdef MAINTEST

int main(int argc, char **argv)
{
    /*
    PORTTYPE fd;

    fd = open_port_and_set_baud_or_die(argv[1], BAUD);
    printf("port %s opened.. \n", argv[1]);

    wait_online(fd);
    //do_test(fd);

    do_test_100_times(fd);


    close_port(fd);
    */

    SerialComm* sc = new PJRCSerialComm();
    sc->open(argv[1]);
    sc->sendWakeupMessage();
    hid_to_pc_message msg;
    sc->receive(msg);
    char c[64];
    msg.toChars(c);
    std::cout << string(c,64);


    return 0;
}
#endif



/**********************************/
/*  Serial Port Functions         */
/**********************************/


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
    if (sscanf(name, "COM%d", &num) == 1) {
        sprintf(portname, "\\\\.\\COM%d", num); // Microsoft KB115831
    } else {
        strncpy(portname, name, sizeof(portname)-1);
        portname[n-1] = 0;
    }
    fd = CreateFileA(portname, GENERIC_READ | GENERIC_WRITE,
        0, 0, OPEN_EXISTING, 0, NULL);
    GetCommConfig(fd, &cfg, &n);
    //cfg.dcb.BaudRate = baud;
    cfg.dcb.BaudRate = 115200;
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
        //printf("read, r = %d\n", r);
        if (r < 0 && errno != EAGAIN && errno != EINTR) return -1;
        else if (r > 0) count += r;
        else {
            // no data available right now, must wait
            fd_set fds;
            struct timeval t;
            FD_ZERO(&fds);
            FD_SET(port, &fds);
            t.tv_sec = 1;
            t.tv_usec = 0;
            r = select(port+1, &fds, NULL, NULL, &t);
            //printf("select, r = %d\n", r);
            if (r < 0) return -1;
            if (r == 0) return count; // timeout
        }
        retry++;
        if (retry > 1000) return -100; // no input
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


/**********************************/
/*  Misc. Functions               */
/**********************************/

void delay(double sec)
{
#if defined(MACOSX) || defined(LINUX)
    usleep(sec * 1000000);
#elif defined(WINDOWS)
    Sleep(sec * 1000);
#endif
}

#endif












}
















































#ifdef PJRC_LATENCY_TEST
#define WINDOWS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <errno.h>

// One of these must be defined, usually via the Makefile
//#define MACOSX
//#define LINUX
//#define WINDOWS

#if defined(MACOSX) || defined(LINUX)
#include <termios.h>
#include <sys/select.h>
#define PORTTYPE int
#define BAUD B115200
#if defined(LINUX)
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif
#elif defined(WINDOWS)
#include <windows.h>
#define PORTTYPE HANDLE
#define BAUD 115200
#else
#error "You must define the operating system\n"
#endif

// function prototypes
PORTTYPE open_port_and_set_baud_or_die(const char *name, long baud);
int transmit_bytes(PORTTYPE port, const char *data, int len);
int receive_bytes(PORTTYPE port, char *buf, int len);
void close_port(PORTTYPE port);
void delay(double sec);

// C++ timing
#include <chrono>
using namespace std;
using namespace std::chrono;


high_resolution_clock::time_point t1, t2;


/************************************/
/*  Latency/Delay Test  Functions   */
/************************************/

double do_test(PORTTYPE port, int len)
{
    char buf[8192];
    //struct timeval begin, end;
    double elapsed;
    int r;

    if (len > sizeof(buf) || len < 1) return 1000000;
    memset(buf, '0', len);
    buf[len - 1] = 'x'; // end of packet marker
    // test begin
    //gettimeofday(&begin, NULL);
    t1 = high_resolution_clock::now();

    // send the data
    r = transmit_bytes(port, buf, len);
    //printf("write, r = %d\n", r);
    // receive the reply
    r = receive_bytes(port, buf, 4);  // response is always 4 bytes
    // test end
    //gettimeofday(&end, NULL);
    t2 = high_resolution_clock::now();

    //elapsed = (double)(end.tv_sec - begin.tv_sec) * 1000.0;
    //elapsed += (double)(end.tv_usec - begin.tv_usec) / 1000.0;
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    elapsed = time_span.count()*1000;

    //printf("  len=%d, elased: %.2f ms\n", len, elapsed);
    return elapsed;
}

void do_test_100_times(PORTTYPE port, int len)
{
    const int num = 1000;
    double ms, total=0, max=0;
    int i;

    for (i=0; i<num; i++) {
        ms = do_test(port, len);
        total += ms;
        if (ms > max) max = ms;
    }
    printf("latency @ %d bytes: ", len);
    printf("%.2f ms average, ", total / num);
    printf("%.2f maximum\n", max);
}

// wait for the Arduino board to boot up, since opening
// the board raises DTR, which resets the processor.
// as soon as it properly responds, we know it's running
void wait_online(PORTTYPE port)
{
    char buf[8];
    int r;

    printf("waiting for board to be ready:\n");
    while (1) {
        delay(0.1);
        printf(".");
        fflush(stdout);
        buf[0] = 'x';
        r = transmit_bytes(port, buf, 1);
        r = receive_bytes(port, buf, 4);
        if (r == 4) break; // success, device online
    }
    printf("ok\n");
}


int main(int argc, char **argv)
{
    PORTTYPE fd;

    fd = open_port_and_set_baud_or_die(argv[1], BAUD);
    printf("port %s opened\n", argv[1]);

    wait_online(fd);
    do_test_100_times(fd, 1);
    do_test_100_times(fd, 2);
    do_test_100_times(fd, 12);
    do_test_100_times(fd, 30);
    do_test_100_times(fd, 64);
    do_test_100_times(fd, 71);
    do_test_100_times(fd, 128);
    do_test_100_times(fd, 500);
    do_test_100_times(fd, 1000);
    do_test_100_times(fd, 2000);
    do_test_100_times(fd, 4000);
    do_test_100_times(fd, 8000);

    close_port(fd);
    return 0;
}


/**********************************/
/*  Serial Port Functions         */
/**********************************/


PORTTYPE open_port_and_set_baud_or_die(const char *name, long baud)
{
    PORTTYPE fd;
#if defined(MACOSX)
    struct termios tinfo;
    fd = open(name, O_RDWR | O_NONBLOCK);
    if (fd < 0) printf("unable to open port %s\n", name);
    if (tcgetattr(fd, &tinfo) < 0) printf("unable to get serial parms\n");
    if (cfsetspeed(&tinfo, baud) < 0) printf("error in cfsetspeed\n");
    tinfo.c_cflag |= CLOCAL;
    if (tcsetattr(fd, TCSANOW, &tinfo) < 0) printf("unable to set baud rate\n");
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
#elif defined(LINUX)
    struct termios tinfo;
    struct serial_struct kernel_serial_settings;
    int r;
    fd = open(name, O_RDWR);
    if (fd < 0) printf("unable to open port %s\n", name);
    if (tcgetattr(fd, &tinfo) < 0) printf("unable to get serial parms\n");
    if (cfsetspeed(&tinfo, baud) < 0) printf("error in cfsetspeed\n");
    if (tcsetattr(fd, TCSANOW, &tinfo) < 0) printf("unable to set baud rate\n");
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
    if (sscanf(name, "COM%d", &num) == 1) {
        sprintf(portname, "\\\\.\\COM%d", num); // Microsoft KB115831
    } else {
        strncpy(portname, name, sizeof(portname)-1);
        portname[n-1] = 0;
    }
    fd = CreateFileA(portname, GENERIC_READ | GENERIC_WRITE,
        0, 0, OPEN_EXISTING, 0, NULL);
    GetCommConfig(fd, &cfg, &n);
    //cfg.dcb.BaudRate = baud;
    cfg.dcb.BaudRate = 115200;
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

    if (len > sizeof(buf) || len < 1) return -1;
    // non-blocking read mode
    fcntl(port, F_SETFL, fcntl(port, F_GETFL) | O_NONBLOCK);
    while (count < len) {
        r = read(port, buf + count, len - count);
        //printf("read, r = %d\n", r);
        if (r < 0 && errno != EAGAIN && errno != EINTR) return -1;
        else if (r > 0) count += r;
        else {
            // no data available right now, must wait
            fd_set fds;
            struct timeval t;
            FD_ZERO(&fds);
            FD_SET(port, &fds);
            t.tv_sec = 1;
            t.tv_usec = 0;
            r = select(port+1, &fds, NULL, NULL, &t);
            //printf("select, r = %d\n", r);
            if (r < 0) return -1;
            if (r == 0) return count; // timeout
        }
        retry++;
        if (retry > 1000) return -100; // no input
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


/**********************************/
/*  Misc. Functions               */
/**********************************/

void delay(double sec)
{
#if defined(MACOSX) || defined(LINUX)
    usleep(sec * 1000000);
#elif defined(WINDOWS)
    Sleep(sec * 1000);
#endif
}

#endif





#ifdef BOOST_ASIO
#include <iostream>
#include <boost/asio.hpp>
#include <chrono>

using namespace std;
using namespace std::chrono;

int main()
{
    cout << "Hello World!" << endl;


    using namespace boost::asio;
    boost::asio::io_service io;
    boost::asio::serial_port *port = new serial_port(io, "COM9");


    high_resolution_clock::time_point t1, t2;
    t1 = high_resolution_clock::now();


    // Send inital
    write(*port, boost::asio::buffer("0x"));

    constexpr int trips = 1000;
    for(int i=0;i<trips;++i){
        boost::asio::streambuf sb;

        // Receive
        read_until(*port, sb, 'x');

        //string sbs((std::istreambuf_iterator<char>(&sb)), std::istreambuf_iterator<char>());
        //cout << sbs.c_str();

        // Send
        write(*port, boost::asio::buffer("0x"));
    }

    t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "\n\nIt took me " << time_span.count() << " seconds. ("<< trips/time_span.count() << " hz)\n\n";

    return 0;
}
#endif















#include "fsusbhapticdevicethread.h"

/*
namespace haptikfabriken {
class PJRCSerialComm : public SerialComm {
    void open(string portname);
    void close();
    void send(const pc_to_hid_message& msg);
    void receive(hid_to_pc_message& msg);
private:
    PORTTYPE fd;
};
}
*/

#endif // PJRCSERIALCOMM_H
