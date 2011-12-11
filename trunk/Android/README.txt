Android Interface for Razor AHRS v1.3.3

Released under GNU GPL (General Public License) v3.0
Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin


Infos, updates, bug reports and feedback:
	http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs

You can find a tutorial on the tracker itself at:
	http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs/wiki/Tutorial



About Razor AHRS and Android Bluetooth:

  Bluetooth seems to be even more picky on Android than it is anyway. Be sure to have a look at the section about Android Bluetooth in the tutorial at:
    http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs


Compiling and running the test app:

  Open up your Android-ready Eclipse and import both projects into Eclipse using "File -> Import... -> General -> Existing Projects into Workspace". The root folder is the folder that contains this readme. Then build and run the test app.
    
  
Building your own app:

  Your app needs to target Android 2.0 (API Level 5) or later.
  
  The RazorAHRS Library Project has to be present in your Workspace. Add the library to your app under Project Properties -> Android -> Library.

  In case you want yaw/heading to reference "true north" and not just magnetic north, you can use the included DeclinationHelper class to find out about declination at your current position.
  
  You have to specify these uses-permissions in the AndroidManifest.xml: 
    android.permission.BLUETOOTH and android.permission.BLUETOOTH_ADMIN
  If you want to use the DeclinationHelper class you also need:
    android.permission.ACCESS_FINE_LOCATION and android.permission.ACCESS_COARSE_LOCATION  