TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

DEFINES -= UNICODE

win32:INCLUDEPATH += unix_adapt
win32:DEFINES += DISABLE_TIMEZONE_STRUCT_REDEFINITION

DEFINES += _REENTRANT
LIBS += -lpthread

# If using Visual Studio compiler for Qt, you will need to install also Pthreads-win32
# from http://www.ensta-bretagne.fr/lebars/Share/pthreads-win32-msvc.zip.
win32-msvc:contains(QMAKE_HOST.arch, x86):INCLUDEPATH += "C:\Program Files (x86)\pthreads-w32-2.9.1-msvc"
win32-msvc:contains(QMAKE_HOST.arch, x86_64):INCLUDEPATH += "C:\Program Files\pthreads-w32-2.8.0-msvc"
win32-msvc:DEFINES += HAVE_STRUCT_TIMESPEC
win32-msvc:LIBS -= -lpthread
win32-msvc:contains(QMAKE_HOST.arch, x86):LIBS += -L"C:\Program Files (x86)\pthreads-w32-2.9.1-msvc"
win32-msvc:contains(QMAKE_HOST.arch, x86_64):LIBS += -L"C:\Program Files\pthreads-w32-2.8.0-msvc"
win32-msvc:contains(QMAKE_HOST.arch, x86):LIBS += -lpthreadVC2
win32-msvc:contains(QMAKE_HOST.arch, x86_64):LIBS += -lpthreadVC2_x64

SOURCES += \
    Example.cpp \
    RazorAHRS.cpp

HEADERS += \
    RazorAHRS.h
