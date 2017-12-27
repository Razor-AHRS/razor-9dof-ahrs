Razor AHRS
---

**9 Degree of Measurement Attitude and Heading Reference System** for Sparkfun *9DOF Razor IMU* (SEN-10125, SEN-10736 and SEN-14001) and SparkFun *9DOF Sensor Stick* (SEN-10183, SEN-10321 and SEN-10724)

Infos, updates, bug reports, contributions and feedback: https://github.com/ptrbrtz/razor-9dof-ahrs

Download
---

Clone the [repository on GitHub](https://github.com/lebarsfa/razor-9dof-ahrs) or [download a specific release](https://github.com/lebarsfa/razor-9dof-ahrs/releases).

Tutorial
---

You find a [detailed tutorial in the Wiki](https://github.com/ptrbrtz/razor-9dof-ahrs/wiki/Tutorial).  

Note: For SEN-14001 (*9DoF Razor IMU M0*), you will need to follow the same instructions as for the default firmware on https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide and use an updated version of SparkFun_MPU-9250-DMP_Arduino_Library from https://github.com/lebarsfa/SparkFun_MPU-9250-DMP_Arduino_Library (an updated version of the default firmware is also available on https://github.com/lebarsfa/9DOF_Razor_IMU).

Quick setup
---

### Razor AHRS Firmware and *Processing* Test Sketch

Select your hardware in `Arduino/Razor_AHRS/Razor_AHRS.ino` under `"USER SETUP AREA"` / `"HARDWARE OPTIONS"`.
Upload the firmware using *Arduino*.  
Run `Processing/Razor_AHRS_test/Razor_AHRS_test.pde` using *Processing*.

### Optional: Mac OS X / Unix / Linux / Windows C++ Interface

Use the provided Qt project (check Projects\Run Settings\Run in terminal to force your application to run inside a separate terminal) or compile test program from the command line (add `-Iunix_adapt -DDISABLE_TIMEZONE_STRUCT_REDEFINITION -DENABLE_O_NDELAY_WORKAROUND` for MinGW/MSYS):

    g++ Example.cpp RazorAHRS.cpp -Wall -D_REENTRANT -lpthread -o example

Run it:

    ./example

Note: To use the provided Visual Studio 2017 project, you will need to install Pthreads-win32 from http://www.ensta-bretagne.fr/lebars/Share/pthreads-win32-msvc.zip.

### Optional: Android Interface

About Razor AHRS and Android Bluetooth: Bluetooth seems to be even more picky on Android than it is anyway. Be sure to have a look at the section about Android Bluetooth in the [tutorial](https://github.com/ptrbrtz/razor-9dof-ahrs/wiki/Tutorial).

Compiling and running the test app: Open up your Android-ready *Eclipse* and import both projects using `File` → `Import...` → `General` → `Existing Projects into Workspace` with the root folder being `Android/`. Then build and run the test app from *Eclipse*.

Building your own app:

* Your app needs to target Android 2.0 (API Level 5) or later. The RazorAHRS *Library Project* has to be present in your Workspace. Add the library to your app under `Project Properties` → `Android` → `Library`.

* In case you want yaw/heading to reference "true north" and not just magnetic north, you can use the included `DeclinationHelper` class to find out about declination at your current position.
  
* You have to specify these uses-permissions in the AndroidManifest.xml:  
`android.permission.BLUETOOTH` and `android.permission.BLUETOOTH_ADMIN`
        
* If you want to use the DeclinationHelper class you also need:  
`android.permission.ACCESS_FINE_LOCATION` and `android.permission.ACCESS_COARSE_LOCATION`

### Optional: ROS Interface

See http://wiki.ros.org/razor_imu_9dof.
