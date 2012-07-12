/******************************************************************************************
* Android Java Interface for Razor AHRS v1.4.1
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
******************************************************************************************/

package de.tuberlin.qu.razorahrs;

import java.io.IOException;

/**
 * Interface definition for callbacks to be invoked when Razor AHRS events occur.
 * 
 * @author Peter Bartz
 */
public interface RazorListener {
	
	/**
	 * Invoked when updated yaw/pitch/roll angles are available.
	 * @param yaw
	 * @param pitch
	 * @param roll
	 */
	void onAnglesUpdate(float yaw, float pitch, float roll);
		
	/**
	 * Invoked when an IOException occurred.
	 * RazorAHRS will be disconnected already.
	 * 
	 * @param e The IOException that was thrown
	 */
	void onIOExceptionAndDisconnect(IOException e);
	
	/**
	 * Invoked when making an attempt to connect.Because connecting via Bluetooth often fails,
	 * multiple attempts can be made.
	 * See {@link RazorAHRS#RazorAHRS(android.bluetooth.BluetoothDevice, RazorListener, int)}.
	 * 
	 * @param attempt Current attempt
	 * @param maxAttempts Maximum number of attempts to be made
	 */
	void onConnectAttempt(int attempt, int maxAttempts);
	
	/**
	 * Invoked when connecting completed successfully.
	 */
	void onConnectOk();
	
	/**
	 * Invoked when connecting failed.
	 * 
	 * @param e	Exception that was thrown
	 */
	void onConnectFail(Exception e);

}
