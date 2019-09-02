#include "haptikfabrikenapi.h"
#include "fshapticdevicethread.h"
#include "fsdaqhapticdevicethread.h"
#ifdef USE_USB_HID
#include "fsusbhapticdevicethread.h"
#endif

#include <sstream>
#include <iomanip>

haptikfabriken::HaptikfabrikenInterface::HaptikfabrikenInterface(bool wait_for_next_message,
       haptikfabriken::Kinematics::configuration c, Protocol protocol):kinematicModel(c),fsthread(0)
{
    switch(protocol){
    case DAQ:
        fsthread = new FsDAQHapticDeviceThread(wait_for_next_message,c);
        break;
    case USB:
#ifdef USE_USB_HID
        fsthread = new FsUSBHapticDeviceThread(wait_for_next_message,c);
#endif
        break;
    case UDP:
        fsthread = new FsHapticDeviceThread(wait_for_next_message,c);
        break;
    }
}

haptikfabriken::HaptikfabrikenInterface::~HaptikfabrikenInterface()
{
    std::cout<< "Haptifabriken destructor: " << this << "\n"; if(fsthread) delete fsthread; fsthread=0;
}

int haptikfabriken::HaptikfabrikenInterface::open()
{
    return fsthread->open();
}

void haptikfabriken::HaptikfabrikenInterface::close()
{
    std::cout << "Closing haptikfabrikeninterface "  << this << "\n";
    if(fsthread)
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

haptikfabriken::fsVec3d haptikfabriken::HaptikfabrikenInterface::getPos()
{
    if(std::abs(fsthread->oldPosition.x() - fsthread->getPos().x())>0.01){
        fsthread->oldPosition = fsthread->getPos();

    }
    return fsthread->getPos();
}

haptikfabriken::fsRot haptikfabriken::HaptikfabrikenInterface::getRot()
{
    return fsthread->getRot();
}

void haptikfabriken::HaptikfabrikenInterface::setForce(haptikfabriken::fsVec3d f)
{
    // Limit to 5N
    double magnitude = sqrt(f.m_x*f.m_x+f.m_y*f.m_y+f.m_z*f.m_z);
    if(magnitude > 5.0){
            fsVec3d dir(f.m_x/magnitude,f.m_y/magnitude,f.m_z/magnitude);
            f = dir*5.0;
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
