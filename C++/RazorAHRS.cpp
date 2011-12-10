/*************************************************************************************
* Mac OSX / Unix / Linux C++ Interface for Razor AHRS v1.3.2
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


#include "RazorAHRS.h"

RazorAHRS::RazorAHRS(const string& port, DataCallbackFunc data_func, ErrorCallbackFunc error_func,
    int connect_timeout_ms, speed_t speed)
    : _input_pos(0)
    , _connect_timeout_ms(connect_timeout_ms)
    , data(data_func)
    , error(error_func)
    , _thread_id(0)
    , _stop_thread(false)
{  
  // open serial port
  if (port == "")
    throw std::runtime_error("No port specified!");
  if (!_open_serial_port(port.c_str()))
    throw std::runtime_error("Could not open serial port!");  
      
  // get port attributes
  struct termios tio;
  if (int errorID = tcgetattr(_serial_port, &tio))
    throw std::runtime_error("Could not get serial port attributes! Error # " + to_str(errorID));
  
  /* see http://www.easysw.com/~mike/serial/serial.html */
  /* and also http://linux.die.net/man/3/tcsetattr */
  // basic raw/non-canonical setup
  cfmakeraw(&tio);
  
  // enable reading and ignore control lines
  tio.c_cflag |= CREAD | CLOCAL;

  // set 8N1
  tio.c_cflag &= ~PARENB; // no parity bit
  tio.c_cflag &= ~CSTOPB; // only one stop bit
  tio.c_cflag &= ~CSIZE;  // clear data bit number
  tio.c_cflag |= CS8;     // set 8 data bits
  
  // no hardware flow control
  tio.c_cflag &= ~CRTSCTS;
  // no software flow control  
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  
  // poll() is broken on OSX, so we set VTIME and use read(), which is ok since
  // we're reading only one port anyway
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 10; // 10 * 100ms = 1s
  
  // set port speed
  if (int errorID = cfsetispeed(&tio, speed))
    throw std::runtime_error(" " + to_str(errorID)
        + ": Could not set new serial port input speed to "
        + to_str(speed) + ".");
  if (int errorID = cfsetospeed(&tio, speed))
    throw std::runtime_error(" " + to_str(errorID)
        + ": Could not set new serial port output speed to "
        + to_str(speed) + ".");

  // set port attributes
  // must be done after setting speed!
  if (int errorID = tcsetattr(_serial_port, TCSANOW, &tio))
  {
    throw std::runtime_error(" " + to_str(errorID)
        + ": Could not set new serial port attributes.");
  }

  // start input/output thread
  _start_io_thread();
}

RazorAHRS::~RazorAHRS()
{
  // if thread was started, stop thread
  if (_thread_id) _stop_io_thread();
  close(_serial_port);
}

bool
RazorAHRS::_read_synch_token(char c)
{
  static const std::string _synch_token = "#SYNCH\r\n";

  if (c == _synch_token[_input_pos++])
  {
    if (_input_pos == _synch_token.length())
    {
      // synch token found
      _input_pos = 0;
      return true;
    }
  }
  else
  {
    _input_pos = 0;
  }
  
  return false;
}

bool
RazorAHRS::_init_razor()
{
  char in;
  int result;
  struct timeval t0, t1, t2;

  // start time
  gettimeofday(&t0, NULL);

  // request synch token to see if Razor is really present
  write(_serial_port, "#s", 2);
  gettimeofday(&t1, NULL);

  // set non-blocking I/O
  if (!_set_nonblocking_io()) return false;

  /* look for tracker */
  while (true)
  {
    // check timeout
    gettimeofday(&t2, NULL);
    if (elapsed_ms(t1, t2) > 200)
    {
      // 200ms elapsed since last request and no answer -> request synch again
      // (this happens when DTR is connected and Razor resets on connect)
      write(_serial_port, "#s", 2);
      t1 = t2;
    }
    if (elapsed_ms(t0, t2) > _connect_timeout_ms)
      // timeout -> tracker not present
      throw std::runtime_error("Can not init: tracker does not answer.");  
    
    // try to read one byte from the port
    result = read(_serial_port, &in, 1);
    
    // one byte read
    if (result > 0)
    {
      if (_read_synch_token(in))
        break;
    }
    // no data available
    else if (result == 0)
      usleep(1000); // sleep 1ms
    // error
    else
      throw std::runtime_error("Can not read from serial port.");  
  }
  
  
  /* configure tracker */
  // set binary output mode, enable continuous streaming, disable errors and
  // request synch token. So we're good, no matter what state the tracker
  // currently is in.
  string razor_config = "#ob#o1#oe0#s";
  write(_serial_port, razor_config.data(), razor_config.length());
  
  // set blocking I/O
  // (actually semi-blocking, because VTIME is set)
  if (!_set_blocking_io()) return false;

  while (true)
  {    
    // try to read one byte from the port
    result = read(_serial_port, &in, 1);
    
    // one byte read
    if (result > 0)
    {
      if (_read_synch_token(in))
        break;  // alrighty
    }
    // error
    else
      throw std::runtime_error("Can not read from serial port.");  
  }
  
  // set semi-blocking I/O blocking again
  if (_set_nonblocking_io() == -1)
    return false;
  
  return true;
}

bool
RazorAHRS::_open_serial_port(const char *port)
{
  // O_NDELAY allows open even with no carrier detect (e.g. needed for Razor)
  if ((_serial_port = open(port, O_RDWR | O_NOCTTY | O_NDELAY)) != -1)
  {
    // make I/O blocking again
    if (_set_blocking_io()) return true;
  }
  
  // something didn't work
  close(_serial_port);
  return false;
}

bool
RazorAHRS::_set_blocking_io()
{
  int flags;
  
  // clear O_NDELAY to make I/O blocking again
  // in fact this is semi-blocking, since we set VTIME on the port
  if (((flags = fcntl(_serial_port, F_GETFL, 0)) != -1) &&
      (fcntl(_serial_port, F_SETFL, flags & ~O_NDELAY)) != -1)
  {
    return true;
  }
  
  return false;
}

bool
RazorAHRS::_set_nonblocking_io()
{
  int flags;

  // set O_NDELAY to make I/O non-blocking
  if (((flags = fcntl(_serial_port, F_GETFL, 0)) != -1) &&
      (fcntl(_serial_port, F_SETFL, flags | O_NDELAY)) != -1)
  {
    return true;
  }
  
  return false;
}

bool
RazorAHRS::_is_io_blocking()
{
  return (fcntl(_serial_port, F_GETFL, 0) & O_NDELAY);
}

void*
RazorAHRS::_thread(void *arg)
{
  char c;
  int result;
  
  try
  {
    if (!_init_razor())
    {
      error("Tracker init failed.");
      return arg;
    }
  }
  catch(std::runtime_error& e)
  {
    error(string("Tracker init failed: ") + string(e.what()));
    return arg;
  }
  
  while (!_stop_thread)
  {
    if ((result = read(_serial_port, &c, 1)) > 0) // blocks only for VTIME before returning
    {
      // read binary stream
      _input_buf[_input_pos++] = c;
      if (_input_pos == 12)
      {
        // we received a full frame
        // forward
        data((float*) _input_buf);
        _input_pos = 0;
      }
    }
    else if (result < 0)
    {
      error("Can not read from serial port.");
      return arg;
    }
    // else if error was 0, no data was available
  }

  return arg;
}
