TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp


unix:!macx: LIBS += -lhaptikfabrikenapi
win32: LIBS += -L$$PWD/../../../lib -lhaptikfabrikenapi
INCLUDEPATH += $$PWD/../../../lib
DEPENDPATH += $$PWD/../../../lib

unix: DEFINES += LINUX


BOOST = F:\boost_1_62_0
unix: LIBS += -lboost_system
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


# If haptikfabriken.lib is built as static lib we will need to link with sensoray (if we use DAQ)
#win32: LIBS += -L$$PWD/../../../external/sensoray/s826_3.3.9/api/x64/ -ls826
#INCLUDEPATH += $$PWD/../../../external/sensoray/s826_3.3.9/api
#DEPENDPATH += $$PWD/../../../external/sensoray/s826_3.3.9/api

win32: LIBS += setupapi.lib
