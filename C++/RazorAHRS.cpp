/******************************************************************************************
* Mac OSX / Unix / Linux C++ Interface for Razor AHRS v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports,contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
******************************************************************************************/

#include "RazorAHRS.h"
#include <cassert>

RazorAHRS::RazorAHRS(const std::string &port, DataCallbackFunc data_func, ErrorCallbackFunc error_func,
    Mode mode, int connect_timeout_ms, speed_t speed)
    : _mode(mode)
    , _input_pos(0)
    , _connect_timeout_ms(connect_timeout_ms)
    , data(data_func)
    , error(error_func)
    , _thread_id(0)
    , _stop_thread(false)
{  
  // check data type sizes
  assert(sizeof(char) == 1);
  assert(sizeof(float) == 4);

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
RazorAHRS::_read_token(const std::string &token, char c)
{
  if (c == token[_input_pos++])
  {
    if (_input_pos == token.length())
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
  const std::string synch_token = "#SYNCH";
  const std::string new_line = "\r\n";

  // start time
  gettimeofday(&t0, NULL);

  // request synch token to see if Razor is really present
  const std::string contact_synch_id = "00"; 
  const std::string contact_synch_request = "#s" + contact_synch_id; 
  const std::string contact_synch_reply = synch_token + contact_synch_id + new_line;
  write(_serial_port, contact_synch_request.data(), contact_synch_request.length());
  gettimeofday(&t1, NULL);

  // set non-blocking I/O
  if (!_set_nonblocking_io()) return false;

  /* look for tracker */
  while (true)
  {
    // try to read one byte from the port
    result = read(_serial_port, &in, 1);
    
    // one byte read
    if (result > 0)
    {
      if (_read_token(contact_synch_reply, in))
        break;
    }
    // no data available
    else if (result == 0)
      usleep(1000); // sleep 1ms
    // error?
    else
    {
      if (errno != EAGAIN && errno != EINTR)
        throw std::runtime_error("Can not read from serial port (1).");
    }

    // check timeout
    gettimeofday(&t2, NULL);
    if (elapsed_ms(t1, t2) > 200)
    {
      // 200ms elapsed since last request and no answer -> request synch again
      // (this happens when DTR is connected and Razor resets on connect)
      write(_serial_port, contact_synch_request.data(), contact_synch_request.length());
      t1 = t2;
    }
    if (elapsed_ms(t0, t2) > _connect_timeout_ms)
      // timeout -> tracker not present
      throw std::runtime_error("Can not init: tracker does not answer.");
  }
  
  
  /* configure tracker */
  // set correct binary output mode, enable continuous streaming, disable errors and
  // request synch token. So we're good, no matter what state the tracker
  // currently is in.
  const std::string config_synch_id = "01";
  const std::string config_synch_reply = synch_token + config_synch_id + new_line;

  std::string config = "#o1#oe0#s" + config_synch_id;
  if (_mode == YAW_PITCH_ROLL) config = "#ob" + config;
  else if (_mode == ACC_MAG_GYR_RAW) config = "#osrb" + config;
  else if (_mode == ACC_MAG_GYR_CALIBRATED) config = "#oscb" + config;
  else throw std::runtime_error("Can not init: unknown 'mode' parameter.");  

  write(_serial_port, config.data(), config.length());
  
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
      if (_read_token(config_synch_reply, in))
        break;  // alrighty
    }
    // error?
    else
    {
      if (errno != EAGAIN && errno != EINTR)
        throw std::runtime_error("Can not read from serial port (2).");
    }
  }
  
  // we keep using blocking I/O
  //if (_set_blocking_io() == -1)
  //  return false;
  
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
    error("Tracker init failed: " + std::string(e.what()));
    return arg;
  }
  
  while (!_stop_thread)
  {
    if ((result = read(_serial_port, &c, 1)) > 0) // blocks only for VTIME before returning
    {
      // read binary stream
      // (type-punning: aliasing with char* is ok)
      (reinterpret_cast<char*> (&_input_buf))[_input_pos++] = c;
      
      if (_mode == YAW_PITCH_ROLL) {  // 3 floats
        if (_input_pos == 12) // we received a full frame
        {
          // convert endianess if necessary
          if (_big_endian())
          {
            _swap_endianess(_input_buf.ypr, 3);
          }
          
          // invoke callback
          data(_input_buf.ypr);
          
          _input_pos = 0;
        }
      } else { // raw or calibrated sensor data (9 floats)
        if (_input_pos == 36) // we received a full frame
        {
          // convert endianess if necessary
          if (_big_endian())
          {
            _swap_endianess(_input_buf.amg, 9);
          }
          
          // invoke callback
          data(_input_buf.amg);
          
          _input_pos = 0;
        }
      }
    }
    // error?
    else if (result < 0)
    {
      if (errno != EAGAIN && errno != EINTR)
      {
        error("Can not read from serial port (3).");
        return arg;
      }
    }
    // else if result is 0, no data was available
  }

  return arg;
}
