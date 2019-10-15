
# Haptikfabriken API can be built either as an executable for direct experimentation
# or as a static or dynamic library (default). 
#
# For Windows library it is recommended do build statically and linking your application
# with boost and sensoray.
#
# For testing/experimentation uncomment (remove #) these lines (and run qmake)...
#TEMPLATE = app
#SOURCES += src/main.cpp

# ...and comment these lines out:
TEMPLATE = lib
CONFIG += dynamiclib

# For Windows we recommend instead static build for now:
#CONFIG += staticlib

# Alternatively: to make a dynamic library in windows, we have to follow
# these guidlines in order to generate a proper .lib file
# https://doc.qt.io/qt-5/sharedlibrary.html
# Hint: one can see the exported functions using (from vs command prompt)
# dumpbin -headers haptikfabrikenapi.lib

# This disables safemode check, which may be needed using WoodenHaptics
#DEFINES += DISABLE_SAFEMODE_CHECK

# Using a serial-based controller?
#DEFINES += PURE_SERIAL


# Rest of configuration comes here
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

TARGET = haptikfabrikenapi
linux: target.path = /usr/local/lib
win32: target.path = c:/chai3d/external/haptikfabrikenapi
header_files.files = src/haptikfabrikenapi.h
linux: header_files.path = /usr/local/include
win32: header_files.path = c:/chai3d/external/haptikfabrikenapi
INSTALLS += target
INSTALLS += header_files


# Choose if you are using USB HID edition (currently only linux) or UDP (default) by commenting this line
CONFIG += use_usb_hid
CONFIG += use_webserv
CONFIG += use_sensoray
CONFIG += polhemv2

polhemv2 {
    HEADERS += ../fs_polhem/haptikfabrikenapi-polhem/polhem.h
    linux: HEADERS += src/polhem.h
    DEFINES += SUPPORT_POLHEMV2
}

# In windows, specify your boost folder
BOOST = F:\boost_1_62_0



CONFIG(debug,debug|release): message("Building debug.")
CONFIG(release,debug|release): message("Building release.")


SOURCES +=   src/kinematics.cpp \
    src/fshapticdevicethread.cpp \
    src/haptikfabrikenapi.cpp

HEADERS += src/kinematics.h \
    src/fshapticdevicethread.h \
    src/haptikfabrikenapi.h \
    src/json.hpp # https://github.com/nlohmann/json/releases v. 3.7.0



use_usb_hid {
    SOURCES += src/fsusbhapticdevicethread.cpp \
               external/hidapi/hid.c
    HEADERS += src/fsusbhapticdevicethread.h \
               external/hidapi/hidapi.h
    DEFINES += USE_USB_HID
}

use_webserv {
    SOURCES += src/webserv.cpp
    HEADERS += src/webserv.h src/client_http.h src/server_http.h
}

use_sensoray {
    SOURCES += src/fsdaqhapticdevicethread.cpp
    HEADERS += src/fsdaqhapticdevicethread.h
    DEFINES += USE_DAQ

    unix: LIBS += -l826_64
    unix: INCLUDEPATH += $$PWD/external/sensoray
    unix: DEPENDPATH += $$PWD/external/sensoray
#    unix:!macx: PRE_TARGETDEPS += $$PWD/external/sensoray/sdk_826_linux_3.3.11/middleware/lib826_64.a
}


DEFINES += BOOST_COROUTINES_NO_DEPRECATION_WARNING
DEFINES += BOOST_COROUTINE_NO_DEPRECATION_WARNING

win32{
    DR = "debug"
    CONFIG(release, debug|release): DR = release
    VS_VER = 14
    INCLUDEPATH += $${BOOST}
    A = -L$${BOOST}\bin.v2\libs
    B = \build\msvc-$${VS_VER}.0\\$${DR}\address-model-64\link-static\threading-multi
    LIBS += $${A}\system$${B}
    LIBS += $${A}\date_time$${B}
    LIBS += $${A}\regex$${B}
    LIBS += $${A}\context$${B}
    LIBS += $${A}\coroutine$${B}
    LIBS += $${A}\thread$${B}
    LIBS += $${A}\chrono$${B}

    LIBS += -lsetupapi
}

unix {
    DEFINES += UNIX
    DEFINES += LINUX
    LIBS += -lpthread
    LIBS +=  -lrt -lboost_system -lboost_date_time -lboost_regex -lboost_context -lboost_coroutine -lboost_thread -lboost_chrono

    use_usb_hid: LIBS += -lusb-1.0 -ludev
}






win32: LIBS += -L$$PWD/external/sensoray/s826_3.3.9/api/x64/ -ls826
#win32: INCLUDEPATH += $$PWD/external/sensoray/s826_3.3.9/api
#win32: DEPENDPATH += $$PWD/external/sensoray/s826_3.3.9/api
