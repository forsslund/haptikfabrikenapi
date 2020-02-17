//////////////////////////////////////////////////////////////////////////////
//    Copyright 2019, SenseGraphics AB
//
//    This file is part of HAPI.
//
//    HAPI is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    HAPI is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with HAPI; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at
//    www.sensegraphics.com for more information.
//
//
/// \file HaptikfabrikenHapticsDevice.h
/// \brief Header file for HaptikfabrikenHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HaptikfabrikenHapticsDevice_H__
#define __HaptikfabrikenHapticsDevice_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_HAPTIKFABRIKENAPI
#include <haptikfabrikenapi.h>

namespace HAPI
{

/// \ingroup HapticsDevices
/// \class HaptikfabrikenHapticsDevice
/// \brief Interface to the WoodenHaptics and Polhem devices through the open source library
/// haptikfabrikenApi. Works on linux.
class HAPI_API HaptikfabrikenHapticsDevice : public HAPIHapticsDevice
{
public:
  /// Constructor. device_index is the index of falcon device
  /// connected.
  HaptikfabrikenHapticsDevice(unsigned int device_index = 0);

  /// Destructor.
  virtual ~HaptikfabrikenHapticsDevice();

  /// Returns the index of the falcon device the instance of this class
  /// refers to.
  inline unsigned int getDeviceIndex()
  {
    return index;
  }

  /// Register this renderer to the haptics renderer database.
  static HapticsDeviceRegistration device_registration;

protected:
  /// Implementation of updateDeviceValues using API to get the values.
  virtual void updateDeviceValues(HAPIHapticsDevice::DeviceValues &dv,
                                  HAPITime dt);

  /// Implementation of sendOutput using API to send forces.
  virtual void sendOutput(DeviceOutput &dv,
                          HAPITime dt);

  /// Initialize the haptics device. Use the HapticThread class in Threads.h
  /// as the thread for haptic rendering.
  /// \param _thread_frequency is the desired haptic frequency.
  /// 1000 is the maximum allowed frequency that can be specified. Setting
  /// this parameter to -1 means run as fast as possible. It is recommended
  /// to use the default value for most users.
  virtual bool initHapticsDevice(int _thread_frequency = 1000);

  /// Releases all resources allocated in initHapticsDevice.
  virtual bool releaseHapticsDevice();

  /// The index of the device that the instance of the class
  /// refers to.
  unsigned int index;

  //------------------------------------------------------------------------------
  // From HaptikfabrikenAPI
  //------------------------------------------------------------------------------
  std::auto_ptr<haptikfabriken::HaptikfabrikenInterface> hfab;
  //------------------------------------------------------------------------------
};
} // namespace HAPI

#endif //__HAVE_HAPTIKFABRIKENAPI__
#endif //__HaptikfabrikenHapticsDevice_H__