/*************************************************************************************
* Test Program: Mac OSX / Unix / Linux C++ Interface for Razor AHRS v1.4.0
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
*************************************************************************************/

#include <iostream>   // cout()
#include <stdexcept>  // runtime_error
#include <cstdio>    // getchar()
#include "RazorAHRS.h"

using namespace std;


// Set  your serial port here!
const string serial_port_name = "/dev/tty.FireFly-6162-SPP"; 
//const string serial_port_name = "/dev/tty.usbserial-A700eEhN";


// Razor error callback handler
// Will be called from (and in) Razor background thread!
void on_error(const string &msg)
{
  cout << "  " << "ERROR: " << msg << endl;
  
  // NOTE: make a copy of the message if you want to save it or send it to another thread. do not
  // save or pass the reference itself, it will not be valid after this function returns! 
}

// Razor data callback handler
// Will be called from (and in) Razor background thread!
void on_data(const float ypr[])
{
  cout << "  " << "Yaw = " << ypr[0] << ", Pitch = " << ypr[1] << ", Roll = " << ypr[2] << endl;

  // NOTE: make a copy of the yaw/pitch/roll data if you want to save it or send it to another
  // thread. do not save or pass the pointer itself, it will not be valid after this function
  // returns! 
}

RazorAHRS *razor;
int main()
{
  cout << endl;
  cout << "  " << "Razor AHRS C++ test" << endl;
  cout << "  " << "Press RETURN to connect to tracker. When you're done press RETURN again to quit." << endl;
  getchar();  // wait RETURN
  cout << "  " << "Connecting..." << endl << endl;
  
  try
  {
    // Create Razor AHRS object. Serial I/O will run in background thread and report
    // errors and data updates using the callbacks on_data() and on_error().
    razor = new RazorAHRS(serial_port_name, on_data, on_error);
    
    // NOTE: If these callback functions were members of a class and not global
    // functions, you would have to bind them before passing. Like this:
    
    // class Callback
    // {
    //   public:
    //     void on_data(const float ypr[]) { }
    //     void on_error(const string &msg) { }
    // };
    
    // Callback c;
    
    // razor = new RazorAHRS(serial_port_name,
    //    bind(&Callback::on_data, &c, placeholders::_1),
    //    bind(&Callback::on_error, &c, placeholders::_1));
    
    // If you're calling from inside of "c" you would of course use "this" instead of "&c".
  }
  catch(runtime_error &e)
  {
    cout << "  " << (string("Could not create tracker: ") + string(e.what())) << endl;
    cout << "  " << "Did you set your serial port in Example.cpp?" << endl;
    return 0;
  }
  
  getchar();  // wait RETURN
  return 0;
}

