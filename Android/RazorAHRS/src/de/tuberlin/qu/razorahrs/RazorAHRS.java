/*************************************************************************************
* Android Java Interface for Razor AHRS v1.3.2
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

package de.tuberlin.qu.razorahrs;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Method;
import java.util.UUID;

import org.apache.http.util.EncodingUtils;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.Handler;
import android.os.Message;
import android.os.SystemClock;
import android.util.Log;

/**
 * Class to easily interface the Razor AHRS via Bluetooth.
 * <p>
 * Beware that the Bluetooth on Android seems to be buggy on most devices (we tested). E.g. on the
 * Galaxy S quite often connecting does not work for no apparent reason. If you can't connect: try
 * again. If you still can't connect, try switching Bluetooth off and on in the system settings
 * and/or reset your Bluetooth modem by power-cycling it.
 * <p>
 * List of other possible Bluetooth bugs:
 * <ul>
 * <li> The Android Bluetooth subsystem floods the log with messages that it overflows after a few
 * seconds.
 * <li> Canceling an in-progress connection sometimes blocks - although it shouldn't.
 * <li> Sometimes Android would half-connect, so that the connection-LED of the modem lights up, but
 * the app hangs on the connect in a non-cancelable way. Bad.
 * <li> TODO Wrong synch...
 * </ul>
 * <p>
 * TODO NOTE:
 * <ul> 
 * <li> Firmware default Bluetooth setting...
 * <li> Manifest permissions...
 * <li> Add library project...
 * </ul> 
 * <p>
 * TODOs:
 * <ul>
 * <li> Add support for USB OTG (Android device used as USB host), if using FTDI is possible.
 * </ul>
 * 
 * @author Peter Bartz
 */
public class RazorAHRS {
	private static final String TAG = "RazorAHRS";
	private static final byte[] SYNCH_TOKEN = EncodingUtils.getAsciiBytes("#SYNCH\r\n");
	
	// Timeout to init Razor AHRS after a Bluetooth connection has been established
	public static final int INIT_TIMEOUT_MS = 5000;
	
	// IDs passed to internal message handler
	private static final int MSG_ID__YPR_DATA = 0;
	private static final int MSG_ID__IO_EXCEPTION_AND_DISCONNECT = 1;
	private static final int MSG_ID__CONNECT_OK = 2;
	private static final int MSG_ID__CONNECT_FAIL = 3;
	private static final int MSG_ID__CONNECT_ATTEMPT = 4;
	
	private static final UUID UUID_SPP = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
	
	private enum ConnectionState {
		DISCONNECTED,
		CONNECTING,
		CONNECTED,
		USER_DISCONNECT_REQUEST
	}
	volatile private ConnectionState connectionState = ConnectionState.DISCONNECTED;
	
	volatile private BluetoothSocket btSocket;
	volatile private BluetoothDevice btDevice;
	volatile private InputStream inStream;
	volatile private OutputStream outStream;

	private RazorListener razorListener;
	volatile private boolean callbacksEnabled = true;
	// TODO remove debug messages
	volatile private boolean debugMessagesEnabled = false;
	
	BluetoothThread btThread;
	
	private int numConnectAttempts;
		
	// Object pools
	ObjectPool<float[]> float3Pool = new ObjectPool<float[]>(new ObjectPool.ObjectFactory<float[]>() {
		@Override
		public float[] newObject() {
			return new float[3];
		}
	});

	/**
	 * Constructor.
	 * Must be called from the thread where you want receive the RazorListener callbacks! So if you
	 * want to manipulate Android UI from the callbacks you have to call this from your main/UI 
	 * thread.
	 * 
	 * @param btDevice {@link android.bluetooth.BluetoothDevice BluetoothDevice} holding the Razor
	 * 		AHRS to connect to.
	 * @param razorListener {@link RazorListener} that will be notified of Razor AHRS events.
	 * @param numConnectAttempts Number of attempts when connecting via Bluetooth. Often connecting
	 * 		only works on the 3rd try or even later. Bluetooth hooray.
	 * @throws RuntimeException thrown if one of the parameters is null.
	 */
	public RazorAHRS(BluetoothDevice btDevice, RazorListener razorListener, int numConnectAttempts)
			throws RuntimeException {
		if (btDevice == null)
			throw new RuntimeException("BluetoothDevice can not be null.");
		this.btDevice = btDevice;
		
		if (razorListener == null)
			throw new RuntimeException("RazorListener can not be null.");
		this.razorListener = razorListener;
		this.numConnectAttempts = numConnectAttempts;
	}

	/**
	 * @return <code>true</code> if listener callbacks are currently enabled, <code>false</code> else.
	 */
	public boolean getCallbacksEnabled() {
		return callbacksEnabled;
	}
	
	/**
	 * Enables/disables listener callbacks.
	 * @param enabled
	 */
	public void setCallbacksEnabled(boolean enabled) {
		callbacksEnabled = enabled;
	}
	
	/**
	 * @return <code>true</code> if debug output is currently enabled, <code>false</code> else.
	 */
	public boolean getDebugMessagesEnabled() {
		return debugMessagesEnabled;
	}
	
	/**
	 * Enables/disables debug output.
	 * @param enabled
	 */
	public void setDebugMessagesEnabled(boolean enabled) {
		debugMessagesEnabled = enabled;
	}
	
	/**
	 * Connect and start reading. Both is done asynchronously. {@link RazorListener#onConnectOk()}
	 * or {@link RazorListener#onConnectFail(IOException)} callbacks will be invoked.
	 */
	public void asyncConnect() {
		debugMsg("asyncConnect()");
		// Disconnect and wait for running thread to end, if needed
		if (btThread != null) {
			asyncDisconnect();
			try {
				btThread.join();
			} catch (InterruptedException e) { }
		}
		
		// Bluetooth thread not running any more, we're definitely in DISCONNECTED state now

		// Create new thread to connect to Razor AHRS and read input
		connectionState = ConnectionState.CONNECTING;
		btThread = new BluetoothThread();
		btThread.start();
	}

	/**
	 * Disconnects from Razor AHRS. If still connecting this will also cancel the connection process.
	 */
	public void asyncDisconnect() {
		debugMsg("asyncDisconnect() BEGIN");
		synchronized (connectionState) {
			debugMsg("asyncDisconnect() SNYNCHRONIZED");
			// Don't go to USER_DISCONNECT_REQUEST state if we are disconnected already
			if (connectionState == ConnectionState.DISCONNECTED)
				return;
	
			// This is a wanted disconnect, so we force (blocking) I/O to break
			connectionState = ConnectionState.USER_DISCONNECT_REQUEST;
			closeSocketAndStreams();
		}
		debugMsg("asyncDisconnect() END");
	}
	
	/**
	 * Closes I/O streams and Bluetooth socket.
	 */
	private void closeSocketAndStreams() {
		debugMsg("closeSocketAndStreams() BEGIN");
		// Close Bluetooth socket => I/O operations immediately will throw exception
		try {
			if (btSocket != null)
				btSocket.close();
		} catch (IOException e) { }
		debugMsg("closeSocketAndStreams() BT SOCKET CLOSED");
		
		// Close streams
		try {
			if (inStream != null)
				inStream.close();
		} catch (IOException e) { }
		try {
			if (outStream != null)
				outStream.close();
		} catch (IOException e) { }
		debugMsg("closeSocketAndStreams() STREAMS CLOSED");

		// Do not set socket and streams null, because input thread might still access them
		//inStream = null;
		//outStream = null;
		//btSocket = null;
		
		debugMsg("closeSocketAndStreams() END");
	}

	/**
	 * Logs message if debug output is enabled.
	 * @param msg
	 */
	private void debugMsg(String msg) {
		if (debugMessagesEnabled) {
			Log.d(TAG, msg);
		}
	}

	/**
	 * Thread that handles connecting to and reading from Razor AHRS. 
	 */
	private class BluetoothThread extends Thread {
		byte[] inBuf = new byte[512];
		int inBufPos = 0;
		
		/**
		 * Blocks until it can read one byte of input, assumes we have a connection up and running.
		 * 
		 * @return One byte from input stream
		 * @throws IOException If reading input stream fails
		 */
		private byte readByte() throws IOException {
			int in = inStream.read();
			if (in == -1)
				throw new IOException("End of Stream");
			return (byte) in;
		}

		/**
		 * Writes out a string using ASCII encoding. Assumes we're connected. Does not handle
		 * exceptions itself.
		 * 
		 * @param text Text to send out
		 * @throws IOException
		 */
		private void write(String text) throws IOException {
			outStream.write(EncodingUtils.getAsciiBytes(text));
		}

		/**
		 * Parse input stream for synch token.
		 * @param in Next byte from input stream
		 * @return <code>true</code> if token was found
		 */
		private boolean readSynchToken(byte in) {
			if (in == SYNCH_TOKEN[inBufPos++]) {
				if (inBufPos == SYNCH_TOKEN.length) {
					// Synch token found
					inBufPos = 0;
					debugMsg("Synch token found");
					return true;
				}
			} else {
				inBufPos = 0;
			}

			return false;
		}

		/**
		 * Synches with Razor AHRS and sets parameters.
		 * @throws IOException
		 */
		private void initRazor() throws IOException {
			long t0, t1, t2;

			// Start time
			t0 = SystemClock.uptimeMillis();

			// Request synch token to see when Razor is up and running
			write("#s");
			t1 = SystemClock.uptimeMillis();

			/* See if Razor is there */
			while (true) {
				// Check timeout
				t2 = SystemClock.uptimeMillis();
				if (t2 - t1 > 200) {
					// 200ms elapsed since last request and no answer -> request synch again.
					// (This happens when DTR is connected and Razor resets on connect)
					write("#s");
					t1 = t2;
				}
				if (t2 - t0 > INIT_TIMEOUT_MS)
					// Timeout -> tracker not present
					throw new IOException("Can not init Razor: response timeout");

				// See if we can read something
				if (inStream.available() > 0) {
					// Synch token found?
					if (readSynchToken(readByte()))
						break;
				} else {
					// No data available, wait
					delay(5); // 5ms
				}
			}

			/* Configure tracker */
			// Set binary output mode, enable continuous streaming, disable errors and request synch
			// token. So we're good, no matter what state the tracker currently is in.
			write("#ob#o1#oe0#s");
			while (!readSynchToken(readByte())) { }
		}
		
		/**
		 * Opens Bluetooth connection to Razor AHRS.
		 * @throws IOException
		 */
		private void connect() throws IOException {		
			// Create Bluetooth socket
			// TODO
//			try {
//				debugMsg("reflection connect hack 1...");
//				Method m = btDevice.getClass().getMethod("createRfcommSocket", new Class[] { int.class });
//				btSocket = (BluetoothSocket) m.invoke(btDevice, Integer.valueOf(1));
//			} catch (Exception e) {
//				throw new IOException("Could not create Bluetooth socket using reflection");
//			}
			btSocket = btDevice.createRfcommSocketToServiceRecord(UUID_SPP);
			if (btSocket == null) {
				debugMsg("btSocket is null in connect()");
				throw new IOException("Could not create Bluetooth socket");
			}
			
			// Connect socket to Razor AHRS
			debugMsg("Canceling bt discovery");
			BluetoothAdapter.getDefaultAdapter().cancelDiscovery();	// Recommended
			debugMsg("Trying to connect() btSocket");
			btSocket.connect();
			
			// Get the input and output streams
			debugMsg("Trying to create streams");
			inStream = btSocket.getInputStream();
			outStream = btSocket.getOutputStream();
			if (inStream == null || outStream == null) {
				debugMsg("Could not create I/O stream(s) in connect()");
				throw new IOException("Could not create I/O stream(s)");
			}
		}

		/**
		 * Bluetooth I/O thread entry method.
		 */
		public void run() {
			debugMsg("Bluetooth I/O thread started");
			try {
				// Check if btDevice is set
				if (btDevice == null) {
					debugMsg("btDevice is null in run()");
					throw new IOException("Bluetooth device is null");
				}
				
				// Try to connect several times
				int i = 1;
				while (true) {
					debugMsg("Connect attempt " + i + " of " + numConnectAttempts);
					sendToParentThread(MSG_ID__CONNECT_ATTEMPT, i);
					try {
						connect();
						break;	// Alrighty!
					} catch (IOException e) {
						debugMsg("Attempt failed: " + e.getMessage());
						// Maximum number of attempts reached or cancel requested?
						if (i == numConnectAttempts || connectionState == ConnectionState.USER_DISCONNECT_REQUEST)
							throw e;
						
						// Wait one second
						delay(1000);
						
						i++;
					}
				}
				
				// Set Razor output mode
				debugMsg("Trying to set Razor output mode");
				initRazor();

				// We're connected and initialized (unless disconnect was requested)
				synchronized (connectionState) {
					if (connectionState == ConnectionState.USER_DISCONNECT_REQUEST) {
						closeSocketAndStreams();
						throw new IOException(); // Dummy exception to force disconnect
					}
					else connectionState = ConnectionState.CONNECTED;
				}

				// Tell listener we're ready
				sendToParentThread(MSG_ID__CONNECT_OK, null);
	
				// Keep reading inStream until an exception occurs
				debugMsg("Starting input loop");
				while (true) {
					// Read byte from input stream
					inBuf[inBufPos++] = (byte) readByte();
						
					if (inBufPos == 12) {
						// We received a full frame, forward input to parent thread handler
						float[] ypr = float3Pool.get();
						ypr[0] = Float.intBitsToFloat((inBuf[0] & 0xff) + ((inBuf[1] & 0xff) << 8) + ((inBuf[2] & 0xff) << 16) + ((inBuf[3] & 0xff) << 24));
						ypr[1] = Float.intBitsToFloat((inBuf[4] & 0xff) + ((inBuf[5] & 0xff) << 8) + ((inBuf[6] & 0xff) << 16) + ((inBuf[7] & 0xff) << 24));
						ypr[2] = Float.intBitsToFloat((inBuf[8] & 0xff) + ((inBuf[9] & 0xff) << 8) + ((inBuf[10] & 0xff) << 16) + ((inBuf[11] & 0xff) << 24));
						sendToParentThread(MSG_ID__YPR_DATA, ypr);
						
						// Rewind input buffer position
						inBufPos = 0;
					}
				}
			} catch (IOException e) {
				debugMsg("IOException in Bluetooth thread: " + e.getMessage());
				synchronized (connectionState) {
					// Don't forward exception if it was thrown because we broke I/O on purpose in
					// other thread when user requested disconnect
					if (connectionState != ConnectionState.USER_DISCONNECT_REQUEST) {
						// There was a true I/O error, cleanup and forward exception
						closeSocketAndStreams();
						debugMsg("Forwarding exception");
						sendToParentThread(MSG_ID__IO_EXCEPTION_AND_DISCONNECT, e);
					} else {
						// I/O error was caused on purpose, socket and streams are closed already
					}
					
					// I/O closed, thread done => we're disconnected now
					connectionState = ConnectionState.DISCONNECTED;
				}
			}
		}
		
		/**
		 * Sends a message to Handler assigned to parent thread.
		 * 
		 * @param msgId
		 * @param data
		 */
		private void sendToParentThread(int msgId, Object o) {
			if (callbacksEnabled)
				parentThreadHandler.obtainMessage(msgId, o).sendToTarget();
		}
		
		/**
		 * Sends a message to Handler assigned to parent thread.
		 * 
		 * @param msgId
		 * @param data
		 */
		private void sendToParentThread(int msgId, int i) {
			if (callbacksEnabled)
				parentThreadHandler.obtainMessage(msgId, i, -1).sendToTarget();
		}

		/**
		 * Wrapper for {@link Thread#sleep(long)};
		 * @param ms Milliseconds
		 */
		void delay(long ms) {
			try {
				sleep(5); // Sleep 5ms
			} catch (InterruptedException e) { }
		}
	}
		
	/**
	 * Handler that forwards messages to the RazorListener callbacks. This handler runs in the
	 * thread this RazorAHRS object was created in and receives data from the Bluetooth I/O thread.
	 */
	private Handler parentThreadHandler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
				switch (msg.what) {
				case MSG_ID__YPR_DATA:
					float[] ypr = (float[]) msg.obj;
					razorListener.onAnglesUpdate(ypr[0], ypr[1], ypr[2]);
					float3Pool.put(ypr);
					break;
				case MSG_ID__IO_EXCEPTION_AND_DISCONNECT:
					razorListener.onIOExceptionAndDisconnect((IOException) msg.obj);
					break;
				case MSG_ID__CONNECT_ATTEMPT:
					razorListener.onConnectAttempt(msg.arg1, numConnectAttempts);
					break;
				case MSG_ID__CONNECT_OK:
					razorListener.onConnectOk();
					break;
				case MSG_ID__CONNECT_FAIL:
					razorListener.onConnectFail((IOException) msg.obj);
				break;
			}
		}
	};
}
