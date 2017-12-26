TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

DEFINES -= UNICODE

win32:INCLUDEPATH += unix_adapt
win32:DEFINES += DISABLE_TIMEZONE_STRUCT_REDEFINITION

DEFINES += _REENTRANT
LIBS += -lpthread

# Uncomment if using Visual Studio compiler for Qt, install also Pthreads-win32 2.9.1
# from http://www.ensta-bretagne.fr/lebars/Share/pthreads-win32-msvc.zip (only in 32 bit).
#win32:INCLUDEPATH += "C:\Program Files (x86)\pthreads-w32-2.9.1-msvc"
#win32:DEFINES += HAVE_STRUCT_TIMESPEC
#win32:LIBS -= -lpthread
#win32:LIBS += -L"C:\Program Files (x86)\pthreads-w32-2.9.1-msvc"
#win32:LIBS += -lpthreadVC2

SOURCES += \
    Example.cpp \
    RazorAHRS.cpp

HEADERS += \
    RazorAHRS.h
