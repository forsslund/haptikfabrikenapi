//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at
//    www.sensegraphics.com for more information.
//
//
/// \file HaptikfabrikenDevice.cpp
/// \brief cpp file for HaptikfabrikenDevice
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/HaptikfabrikenDevice.h>
#include <HAPI/HaptikfabrikenHapticsDevice.h>

using namespace H3D;

H3DNodeDatabase HaptikfabrikenDevice::database("HaptikfabrikenDevice",
                                               &(newInstance<HaptikfabrikenDevice>),
                                               typeid(HaptikfabrikenDevice),
                                               &H3DHapticsDevice::database);
namespace HaptikfabrikenDeviceInternals
{
FIELDDB_ELEMENT(HaptikfabrikenDevice, kinematicsType, INITIALIZE_ONLY)
}

/// Constructor.
HaptikfabrikenDevice::HaptikfabrikenDevice(
    Inst<SFVec3f> _devicePosition,
    Inst<SFRotation> _deviceOrientation,
    Inst<TrackerPosition> _trackerPosition,
    Inst<TrackerOrientation> _trackerOrientation,
    Inst<SFMatrix4f> _positionCalibration,
    Inst<SFRotation> _orientationCalibration,
    Inst<SFVec3f> _proxyPosition,
    Inst<WeightedProxy> _weightedProxyPosition,
    Inst<SFFloat> _proxyWeighting,
    Inst<SFBool> _mainButton,
    Inst<SFBool> _secondaryButton,
    Inst<SFInt32> _buttons,
    Inst<SFVec3f> _force,
    Inst<SFVec3f> _torque,
    Inst<SFInt32> _inputDOF,
    Inst<SFInt32> _outputDOF,
    Inst<SFInt32> _hapticsRate,
    Inst<SFInt32> _desiredHapticsRate,
    Inst<SFNode> _stylus,
    Inst<SFHapticsRendererNode> _hapticsRenderer,
    Inst<MFVec3f> _proxyPositions,
    Inst<SFBool> _followViewpoint,
    Inst<SFString> _kinematicsType) : H3DHapticsDevice(_devicePosition, _deviceOrientation, _trackerPosition,
                                                       _trackerOrientation, _positionCalibration,
                                                       _orientationCalibration, _proxyPosition,
                                                       _weightedProxyPosition, _proxyWeighting, _mainButton,
                                                       _secondaryButton, _buttons, _force, _torque, _inputDOF,
                                                       _outputDOF, _hapticsRate, _desiredHapticsRate, _stylus,
                                                       _hapticsRenderer, _proxyPositions, _followViewpoint),
                                      kinematicsType(_kinematicsType)
{

  type_name = "HaptikfabrikenDevice";
  database.initFields(this);
  hapi_device.reset(0);
  //kinematicsType->setValue("WoodenHaptics 1.5");
}

void HaptikfabrikenDevice::initialize()
{
  H3DHapticsDevice::initialize();

#ifdef HAVE_HAPTIKFABRIKENAPI
  hapi_device.reset(new HAPI::HaptikfabrikenHapticsDevice(0)); // TODO: change to kinematicstype for example
#else
  Console(LogLevel::Error) << "Cannot use HaptikfabrikenDevice. HAPI compiled without"
                           << " Haptikfabriken support. Recompile HAPI with "
                           << "HAVE_HAPTIKFABRIKENAPI defined"
                           << " in order to use it." << endl;
#endif // HAVE_HAPTIKFABRIKENAPI
}

H3DHapticsDevice::ErrorCode HaptikfabrikenDevice::initDevice()
{
  HAPI::HAPIHapticsDevice::ErrorCode e = H3DHapticsDevice::initDevice();
#ifdef HAVE_HAPTIKFABRIKENAPI

  /*
   * TODO: Get information from device and expose to x3d
  HAPI::FalconHapticsDevice *pd =
    dynamic_cast< HAPI::FalconHapticsDevice * >(hapi_device.get() );
  if( e == HAPI::HAPIHapticsDevice::SUCCESS && pd ) {
    deviceModelType->setValue( pd->getDeviceModel(), id );
    HAPI::Vec3 max, min;
    pd->getWorkspaceDimensions( min, max );
    maxWorkspaceDimensions->setValue( 0, (Vec3f)min, id ); 
    maxWorkspaceDimensions->setValue( 1, (Vec3f)max, id ); 
  }
  */
#endif
  return e;
}