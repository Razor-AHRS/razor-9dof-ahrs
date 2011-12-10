/*************************************************************************************
* Test App: Android Java Interface for Razor AHRS v1.3.2
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

package de.tuberlin.qu;

import java.io.IOException;
import java.util.Set;

import android.app.Activity;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;
import de.tuberlin.qu.razorahrs.RazorAHRS;
import de.tuberlin.qu.razorahrs.RazorListener;

public class RazorExample extends Activity {
protected static final String TAG = "RazorExampleActivity";
	
	private BluetoothAdapter bluetoothAdapter;
	private RazorAHRS razor;
	
	private RadioGroup deviceListRadioGroup;
	private Button connectButton;
	private Button cancelButton;
	private TextView yawTextView;
	private TextView pitchTextView;
	private TextView rollTextView;
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		Log.d(TAG, "onCreate");
		
		// Set content view
		setContentView(R.layout.main);
		
		// Find views
		connectButton = (Button) findViewById(R.id.connect_button);
		cancelButton = (Button) findViewById(R.id.cancel_button);
		deviceListRadioGroup = (RadioGroup) findViewById(R.id.devices_radiogroup);
		yawTextView = (TextView) findViewById(R.id.yaw_textview);
		pitchTextView = (TextView) findViewById(R.id.pitch_textview);
		rollTextView = (TextView) findViewById(R.id.roll_textview);
		
		// Get Bluetooth adapter
		bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		if (bluetoothAdapter == null) {	// Ooops
			// Show message
			errorText("Your device does not seem to have Bluetooth, sorry.");
			return;
		}
		
		// Check whether Bluetooth is enabled
		if (!bluetoothAdapter.isEnabled()) {
			errorText("Bluetooth not enabled. Please enable and try again!");
			return;
		}
		
		// Get list of paired devices
		Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();
		
		// Add devices to radio group
	    for (BluetoothDevice device : pairedDevices) {
	        RadioButton rb = new RadioButton(this);
	        rb.setText(" " + device.getName());
	        rb.setTag(device);
	        deviceListRadioGroup.addView(rb);
	    }
	    
	    // Check if any paired devices found
	    if (pairedDevices.size() == 0) {
	    	errorText("No paired Bluetooth devices found. Please go to Bluetooth Settings and pair the Razor AHRS.");
	    } else {
	    	((RadioButton) deviceListRadioGroup.getChildAt(0)).setChecked(true);
	    	enableConnectButton();
	    }

	    // Connect button click handler
	    connectButton.setOnClickListener(new View.OnClickListener() {
	    	ProgressDialog progressDialog;
	    	
	    	public void onClick(View view) {
	    		disableOkButton();
	    		
	    		// Get selected Bluetooth device
	    		RadioButton rb = (RadioButton) findViewById(deviceListRadioGroup.getCheckedRadioButtonId());
	    		if (rb == null) {
	    			Toast.makeText(RazorExample.this, "You have select a device first.", Toast.LENGTH_LONG).show();
	    			return;
	    		}
	    		BluetoothDevice razorDevice = (BluetoothDevice) rb.getTag();
	    		
	    		// Create new razor instance and set listener
	    		razor = new RazorAHRS(razorDevice, new RazorListener() {
	    			@Override
	    			public void onConnectAttempt(int attempt, int maxAttempts) {
	    				Toast.makeText(RazorExample.this, "Connect attempt " + attempt + " of " + maxAttempts + "...", Toast.LENGTH_SHORT).show();
	    			}
	    			
	    			@Override
	    			public void onConnectOk() {
	    				Toast.makeText(RazorExample.this, "Connected!", Toast.LENGTH_LONG).show();
	    			}
	    			
	    			public void onConnectFail(Exception e) {
	    				enableConnectButton();
		    			Toast.makeText(RazorExample.this, "Connecting failed: " + e.getMessage() + ".", Toast.LENGTH_LONG).show();
	    			}

					@Override
					public void onAnglesUpdate(float yaw, float pitch, float roll) {
						yawTextView.setText(String.valueOf((int) (yaw + 0.5f)));
						pitchTextView.setText(String.valueOf((int) (pitch + 0.5f)));
						rollTextView.setText(String.valueOf((int) (roll + 0.5f)));
					}

					@Override
					public void onIOExceptionAndDisconnect(IOException e) {
	    				enableConnectButton();
		    			Toast.makeText(RazorExample.this, "Disconnected, an error occured: " + e.getMessage() + ".", Toast.LENGTH_LONG).show();
					}
	    		}, 5);
	    		
	    		// TODO
	    		razor.setDebugMessagesEnabled(true);
	    		
	    		// Connect asynchronously
	    		razor.asyncConnect();
	    	}
	    });
	    
	    // Cancel button click handler
	    cancelButton.setOnClickListener(new View.OnClickListener() {
	    	public void onClick(View view) {
	    		razor.asyncDisconnect(); // Also cancels pending connect 
	    		enableConnectButton();
	    	}
	    });
	}
	
	private void errorText(String text) {
    	TextView tv = new TextView(this);
    	tv.setText(text);
    	deviceListRadioGroup.addView(tv);
	}
	
	private void enableConnectButton() {
		// Enable connect button
		connectButton.setEnabled(true);
		connectButton.setText("Connect");
		
		// Disable cancel button
		cancelButton.setEnabled(false);
	}

	private void disableOkButton() {
		// Disable connect button and set text
		connectButton.setEnabled(false);
		connectButton.setText("Connecting...");
		
		// Enable cancel button
		cancelButton.setEnabled(true);
	}

	@Override
	protected void onStop() {
		super.onStop();
		Log.d(TAG, "onStop");
		
		if (razor != null)
			razor.asyncDisconnect();
	}
}