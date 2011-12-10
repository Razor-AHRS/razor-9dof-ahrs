/*************************************************************************************
* Test Program: Mac OSX / Unix / Linux C++ Interface for Razor AHRS v1.3.3
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

#include <iostream>   // cout()
#include <stdexcept>  // runtime_error
#include <stdio.h>    // getchar()
#include "RazorAHRS.h"

using namespace std;


//const string serial_port_name = "/dev/tty.FireFly-6162-SPP"; 
const string serial_port_name = "/dev/tty.usbserial-A700eEhN";


// Razor error callback handler
// Will be called from other thread!
void on_error(const string& msg)
{
  cout << "  " << "ERROR: " << msg << endl;
}

// Razor data callback handler
// Will be called from other thread!
void on_data(float* ypr)
{
  cout << "  " << "Yaw = " << ypr[0] << ", Pitch = " << ypr[1] << ", Roll = " << ypr[2] << endl;
}

RazorAHRS* razor;
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
    
    // NOTE: If these callback functions were members of a class and no global
    // functions, you would have to bind them before passing. Like this:
    
    // class Callback
    // {
    //   public:
    //     void on_data(float* ypr) { }
    //     void on_error(const string& msg) { }
    // };
    
    // Callback c;
    
    // razor = new RazorAHRS(serial_port_name,
    //    bind(&Callback::on_data, c, placeholders::_1),
    //    bind(&Callback::on_data, c, placeholders::_1));
    
    // If you're calling from inside of "c" you would of course use "*this" instead of "c".

  }
  catch(runtime_error& e)
  {
    cout << "  " << (string("Could not create tracker: ") + string(e.what())) << endl;
    cout << "  " << "Did you set your serial port in Example.cpp?" << endl;
    return 0;
  }
  
  getchar();  // wait RETURN
  return 0;
}

