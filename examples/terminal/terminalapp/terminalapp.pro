TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp


unix:!macx: LIBS += -L$$PWD/../../../ -lhaptikfabrikenapi
win32: LIBS += -L$$PWD/../../../../build-haptikfabrikenapi-Desktop_Qt_5_9_1_MSVC2015_64bit-Release/release -lhaptikfabrikenapi




INCLUDEPATH += $$PWD/../../../src
DEPENDPATH += $$PWD/../../../src

unix: DEFINES += LINUX


# Apparently needed...
unix: LIBS += -lboost_system

BOOST = F:\boost_1_62_0
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
}

win32: LIBS += -L$$PWD/../../../../build-haptikfabrikenapi-Desktop_Qt_5_9_1_MSVC2015_64bit-Release/release/ -lhaptikfabrikenapi

INCLUDEPATH += $$PWD/../../../src
DEPENDPATH += $$PWD/../../../src


# If haptikfabriken.lib is built as static lib we will need to link with sensoray
win32: LIBS += -L$$PWD/../../../external/sensoray/s826_3.3.9/api/x64/ -ls826
INCLUDEPATH += $$PWD/../../../external/sensoray/s826_3.3.9/api
DEPENDPATH += $$PWD/../../../external/sensoray/s826_3.3.9/api

win32: LIBS += setupapi.lib
