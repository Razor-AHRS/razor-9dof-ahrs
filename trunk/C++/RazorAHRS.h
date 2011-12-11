/*************************************************************************************
* Mac OSX / Unix / Linux C++ Interface for Razor AHRS v1.3.3
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun 9DOF Razor IMU
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
*************************************************************************************/

#ifndef RAZORAHRS_H
#define RAZORAHRS_H

#include <string>
#include <memory>
#include <tr1/functional>
#include <stdexcept>
#include <sstream>
#include <unistd.h>  // for write(), close(), ...
#include <termios.h> // for cfsetispeed(), ...
#include <fcntl.h>   // for open(), ...
#include <sys/time.h>
 
using namespace std;
using namespace std::tr1;

#ifndef _REENTRANT
#error You need to compile with _REENTRANT defined since this uses threads!
#endif

// Razor AHRS tracker
class RazorAHRS
{
  public:    
    typedef function<void(float[])> DataCallbackFunc;
    typedef function<void(const string&)> ErrorCallbackFunc;

    RazorAHRS(const string& port, DataCallbackFunc data_func, ErrorCallbackFunc error_func,
        int connect_timeout_ms = 5000, speed_t speed = B57600);
    ~RazorAHRS();

  private:
    // serial port helpers
    bool _open_serial_port(const char *port);
    bool _set_blocking_io();
    bool _set_nonblocking_io();
    bool _is_io_blocking();

    bool _read_token(const string& token, char c);
    bool _init_razor();
    
    // timing
    long elapsed_ms(struct timeval start, struct timeval end)
    {
      return (long) ((end.tv_sec - start.tv_sec) * 1000 + (end.tv_usec - start.tv_usec) / 1000);
    }

    size_t _input_pos;
    char _input_buf[12];
    int _connect_timeout_ms;
    int _serial_port;

    // callbacks    
    DataCallbackFunc data;
    ErrorCallbackFunc error;

    /* threading stuff */
    pthread_t _thread_id;
    void* _thread(void*);  // thread main function
    volatile bool _stop_thread; // thred stop flag

    // start the tracking thread
    void _start_io_thread()
    {
      // create thread
      pthread_create(&_thread_id , NULL, _thread_starter, this);
    }
  
    // stop the tracking thread
    void _stop_io_thread()
    {
      void *thread_exit_status; // dummy
      _stop_thread = true;
      pthread_join(_thread_id , &thread_exit_status);
    }

    static void* _thread_starter(void *arg)
    {
      return reinterpret_cast<RazorAHRS*> (arg)->_thread(NULL);
    }
    
    string to_str(int i)
    {
      stringstream ss;
      ss << i;
      return ss.str();
    }
};

#endif // RAZORAHRS_H
