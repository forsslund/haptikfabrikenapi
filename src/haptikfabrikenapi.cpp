#include "haptikfabrikenapi.h"
#include "fshapticdevicethread.h"
#ifdef USE_DAQ
#include "fsdaqhapticdevicethread.h"
#endif
#ifdef USE_USB_HID
#include "fsusbhapticdevicethread.h"
#endif

#include <sstream>
#include <iomanip>
#include <boost/thread/thread.hpp>

//------------------------------------------------------------------------------
// https://www.ridgesolutions.ie/index.php/2012/12/13/boost-c-read-from-serial-port-with-timeout-example/
//------------------------------------------------------------------------------
//
// blocking_reader.h - a class that provides basic support for
// blocking & time-outable single character reads from
// boost::asio::serial_port.
//
// use like this:
//
// 	blocking_reader reader(port, 500);
//
//	char c;
//
//	if (!reader.read_char(c))
//		return false;
//
// Kevin Godden, www.ridgesolutions.ie
//

#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>

class blocking_reader
{
    boost::asio::serial_port &port;
    boost::asio::io_service &io;
    size_t timeout;
    char c;
    boost::asio::deadline_timer timer;
    bool read_error;

    // Called when an async read completes or has been cancelled
    void read_complete(const boost::system::error_code &error,
                       size_t bytes_transferred)
    {

        read_error = (error || bytes_transferred == 0);

        // Read has finished, so cancel the
        // timer.
        timer.cancel();
    }

    // Called when the timer's deadline expires.
    void time_out(const boost::system::error_code &error)
    {

        // Was the timeout was cancelled?
        if (error)
        {
            // yes
            return;
        }

        // no, we have timed out, so kill
        // the read operation
        // The read callback will be called
        // with an error
        port.cancel();
    }

public:
    // Constructs a blocking reader, pass in an open serial_port and
    // a timeout in milliseconds.
    blocking_reader(boost::asio::serial_port &port, boost::asio::io_service &io,
                    size_t timeout) : port(port), io(io), timeout(timeout),
                                      timer(io),
                                      read_error(true)
    {
    }

    // Reads a character or times out
    // returns false if the read times out
    bool read_char(char &val)
    {

        val = c = '\0';

        // After a timeout & cancel it seems we need
        // to do a reset for subsequent reads to work.
        //port.get_executor().context().reset();
        io.reset();

        // Asynchronously read 1 character.
        boost::asio::async_read(port, boost::asio::buffer(&c, 1),
                                boost::bind(&blocking_reader::read_complete,
                                            this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));

        // Setup a deadline time to implement our timeout.
        timer.expires_from_now(boost::posix_time::milliseconds(timeout));
        timer.async_wait(boost::bind(&blocking_reader::time_out,
                                     this, boost::asio::placeholders::error));

        // This will block until a character is read
        // or until the it is cancelled.
        io.run();

        if (!read_error)
            val = c;

        return !read_error;
    }
};
//------------------------------------------------------------------------------

std::string haptikfabriken::HaptikfabrikenInterface::serialport_name;
unsigned int haptikfabriken::HaptikfabrikenInterface::findUSBSerialDevices()
{
    using namespace boost::asio;
    boost::asio::io_service io;
    for (int os = 0; os < 2; ++os)
    {
        for (int i = 0; i < 10; ++i)
        {
            std::string str = (os == 0 ? "/dev/ttyACM" : "COM") + std::to_string(i);
            try
            {
                cout << "Trying " << str << "... ";
                serial_port port(io, str);

                boost::asio::streambuf sb;
                blocking_reader reader(port, io, 500);

                char c;
                std::string rsp;

                write(port, boost::asio::buffer("0 0 0 0 0 0 0\n"));

                // read from the serial port until we get a
                // \n or until a read times-out (500ms)
                while (reader.read_char(c) && c != '\n')
                {
                    rsp += c;
                }

                if (c != '\n')
                {
                    // it must have timed out.
                    cout << "Read timed out!";
                }

                std::cout << "Read: " << rsp;
                if (rsp.length() > 10 && rsp.length() < 65)
                {
                    serialport_name = str;
                    port.close();
                    return 1;
                }

                //size_t num_bytes = read_until(port,sb,'\n');
            }
            catch (const std::exception &e)
            {
                cout << e.what() << "\n";
            }
        }
    }

    return 0;
}

haptikfabriken::HaptikfabrikenInterface::HaptikfabrikenInterface(haptikfabriken::Kinematics::configuration c,
                                                                 haptikfabriken::HaptikfabrikenInterface::Protocol protocol): kinematicModel(c), fsthread(nullptr)
{
    switch (protocol)
    {
    case DAQ:
#ifdef USE_DAQ
        fsthread = new FsDAQHapticDeviceThread(c);
#else
        std::cout << "Haptikfabriken compiled without DAQ support.\n";
#endif
        break;
    case USB:
#ifdef PURE_SERIAL
        HaptikfabrikenInterface::findUSBSerialDevices();
#endif
#ifdef USE_USB_HID
        fsthread = new FsUSBHapticDeviceThread(c, serialport_name);
#endif
        break;
    case UDP:
        fsthread = new FsHapticDeviceThread(c);
        break;
    }
}

haptikfabriken::HaptikfabrikenInterface::HaptikfabrikenInterface(bool, haptikfabriken::Kinematics::configuration c,
                haptikfabriken::HaptikfabrikenInterface::Protocol protocol):haptikfabriken::HaptikfabrikenInterface(c,protocol){}


haptikfabriken::HaptikfabrikenInterface::~HaptikfabrikenInterface()
{
    std::cout << "Haptifabriken destructor: " << this << "\n";
    if (fsthread)
        delete fsthread;
    fsthread = nullptr;
}

int haptikfabriken::HaptikfabrikenInterface::open()
{
    return fsthread->open();
}

void haptikfabriken::HaptikfabrikenInterface::close()
{
    std::cout << "Closing haptikfabrikeninterface " << this << "\n";
    if (fsthread)
        fsthread->close();
}

std::string haptikfabriken::HaptikfabrikenInterface::getErrorCode()
{
    return fsthread->getErrorCode();
}

void haptikfabriken::HaptikfabrikenInterface::getEnc(int a[])
{
    fsthread->getEnc(a);
}

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getBodyAngles()
{
    return fsthread->getBodyAngles();
}

void haptikfabriken::HaptikfabrikenInterface::getLatestCommandedMilliamps(int ma[])
{
    fsthread->getLatestCommandedMilliamps(ma);
}

int haptikfabriken::HaptikfabrikenInterface::getNumSentMessages()
{
    return fsthread->getNumSentMessages();
}

int haptikfabriken::HaptikfabrikenInterface::getNumReceivedMessages()
{
    return fsthread->getNumReceivedMessages();
}

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getPos(bool blocking)
{
    if (std::abs(fsthread->oldPosition.x() - fsthread->getPos().x()) > 0.01)
    {
        fsthread->oldPosition = fsthread->getPos();
    }
    return fsthread->getPos(blocking);
}

haptikfabriken::fsRot haptikfabriken::HaptikfabrikenInterface::getRot()
{
    return fsthread->getRot();
}

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getCurrentForce()
{
    return fsthread->getCurrentForce();
}

void haptikfabriken::HaptikfabrikenInterface::setForce(haptikfabriken::fsVec3d f)
{
    // Limit to 5N
    double magnitude = sqrt(f.m_x * f.m_x + f.m_y * f.m_y + f.m_z * f.m_z);
    if (magnitude > 5.0)
    {
        fsVec3d dir(f.m_x / magnitude, f.m_y / magnitude, f.m_z / magnitude);
        f = dir * 5.0;
    }

    fsthread->setForce(f);
}

void haptikfabriken::HaptikfabrikenInterface::setCurrent(haptikfabriken::fsVec3d amps)
{
    fsthread->setCurrent(amps);
}

std::bitset<5> haptikfabriken::HaptikfabrikenInterface::getSwitchesState()
{
    return fsthread->getSwitchesState();
}

void haptikfabriken::HaptikfabrikenInterface::calibrate()
{
    fsthread->calibrate();
}

void haptikfabriken::HaptikfabrikenInterface::addEventListener(haptikfabriken::HapticListener *listener)
{
    fsthread->addEventListener(listener);
}

void haptikfabriken::HaptikfabrikenInterface::removeEventListener(haptikfabriken::HapticListener *listener)
{
    fsthread->removeEventListener(listener);
}

std::string haptikfabriken::toString(const haptikfabriken::fsVec3d &r)
{

    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    ss << std::setw(6) << r.m_x << ", " << std::setw(6) << r.m_y << ", " << std::setw(6) << r.m_z;
    return ss.str();
}

std::string haptikfabriken::toString(const haptikfabriken::fsRot &r)
{

    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);
    ss << std::setw(6) << r.m[0][0] << ", " << std::setw(6) << r.m[0][1] << ", " << std::setw(6) << r.m[0][2] << ",\n";
    ss << std::setw(6) << r.m[1][0] << ", " << std::setw(6) << r.m[1][1] << ", " << std::setw(6) << r.m[1][2] << ",\n";
    ss << std::setw(6) << r.m[2][0] << ", " << std::setw(6) << r.m[2][1] << ", " << std::setw(6) << r.m[2][2] << "\n";
    return ss.str();
}
