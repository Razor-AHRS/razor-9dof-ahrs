*/*****************************************************************************************
* Android Java Interface for Razor AHRS v1.4.1
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
******************************************************************************************/

package de.tuberlin.qu.razorahrs;

import java.util.List;

import android.content.Context;
import android.hardware.GeomagneticField;
import android.location.Location;
import android.location.LocationManager;

/**
 * Helper class to simplify retrieval of declination at given location. For getCurrentLocation() to
 * work, the caller needs to have ACCESS_COARSE_LOCATION and ACCESS_FINE_LOCATION permissions. Also
 * location tracking (i.e. GPS or Network) has to be turned on by the user.
 * <p>
 * You can also find out about declination values here:
 * Overview: http://www.ngdc.noaa.gov/geomag/WMM/data/WMM2010/WMM2010_D_MERC.pdf
 * Calculator: http://www.ngdc.noaa.gov/geomagmodels/IGRFWMM.jsp
 */
public class DeclinationHelper {
	/**
	 * Returns current (last known) location of the system. If the context is missing permissions
	 * (ACCESS_COARSE_LOCATION, ACCESS_FINE_LOCATION) or location tracking is disabled by user, this
	 * will fail and return null.
	 * 
	 * @param context
	 *            Application context of caller
	 * @return Current location or null if we could not retrieve current location
	 */
	public static Location getCurrentLocation(Context context) {
		LocationManager lm = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
		List<String> providers = lm.getProviders(true);

		// Find most accurate last known location
		Location location = null;
		for (int i = providers.size() - 1; i >= 0; i--) {
			location = lm.getLastKnownLocation(providers.get(i));
			if (location != null)
				break;
		}

		return location;
	}
	
	/**
	 * Returns declination in degrees at given location
	 * 
	 * @param location
	 * @return Declination in degrees
	 */
	public static float getDeclinationAt(Location location) {
		return getDeclinationAt((float) location.getLatitude(), (float) location.getLongitude(),
				(float) location.getAltitude());
	}
	
	/**
	 * Returns declination in degrees at given location
	 * 
	 * @param location
	 * @return Declination in degrees
	 */
	public static float getDeclinationAt(float latitude, float longitude, float altitude) {
		GeomagneticField gf = new GeomagneticField(latitude, longitude, altitude, System.currentTimeMillis());
		return gf.getDeclination();
	}
}
