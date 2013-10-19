/******************************************************************************************
* Android Java Interface for Razor AHRS v1.4.2
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

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
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
 * Bluetooth seems to be even more picky on Android than it is anyway. Be sure to have a look at the
 * section about Android Bluetooth in the tutorial at
 * <a href="https://github.com/ptrbrtz/razor-9dof-ahrs">
 * https://github.com/ptrbrtz/razor-9dof-ahrs</a>!
 * <p>
 * The app using this class has to
 * <ul>
 * <li>target Android 2.0 (API Level 5) or later.
 * <li>specify the uses-permissions <code>BLUETOOTH</code> and <code>BLUETOOTH_ADMIN</code> in it's
 * AndroidManifest.xml.
 * <li>add this Library Project as a referenced library (Project Properties -> Android -> Library)
 * </ul>
 * <p>
 * TODOs:
 * <ul>
 * <li>Add support for USB OTG (Android device used as USB host), if using FTDI is possible.
 * </ul>
 * 
 * @author Peter Bartz
 */
public class RazorAHRS {
	private static final String TAG = "RazorAHRS";
	private static final boolean DEBUG = false;
	private static final String SYNCH_TOKEN = "#SYNCH";
	private static final String NEW_LINE = "\r\n";

	// Timeout to init Razor AHRS after a Bluetooth connection has been established
	public static final int INIT_TIMEOUT_MS = 5000;

	// IDs passed to internal message handler
	private static final int MSG_ID__YPR_DATA = 0;
	private static final int MSG_ID__IO_EXCEPTION_AND_DISCONNECT = 1;
	private static final int MSG_ID__CONNECT_OK = 2;
	private static final int MSG_ID__CONNECT_FAIL = 3;
	private static final int MSG_ID__CONNECT_ATTEMPT = 4;
	private static final int MSG_ID__AMG_DATA = 5;

	private static final UUID UUID_SPP = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

	/**
	 * Razor output modes.
	 * Use <code>YAW_PITCH_ROLL_ANGLES</code> to receive yaw, pitch and roll in degrees. <br>
	 * Use <code>RAW_SENSOR_DATA</code> or <code>CALIBRATED_SENSOR_DATA</code> to read raw or
	 * calibrated xyz sensor data of the accelerometer, magnetometer and the gyroscope.
	 */
	public enum RazorOutputMode {
		YAW_PITCH_ROLL_ANGLES,
		RAW_SENSOR_DATA,
		CALIBRATED_SENSOR_DATA
	}
	private RazorOutputMode razorOutputMode;

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
	private boolean callbacksEnabled = true;

	BluetoothThread btThread;

	private int numConnectAttempts;

	// Object pools
	private ObjectPool<float[]> float3Pool = new ObjectPool<float[]>(new ObjectPool.ObjectFactory<float[]>() {
		@Override
		public float[] newObject() {
			return new float[3];
		}
	});
	private ObjectPool<float[]> float9Pool = new ObjectPool<float[]>(new ObjectPool.ObjectFactory<float[]>() {
		@Override
		public float[] newObject() {
			return new float[9];
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
	 * @throws RuntimeException thrown if one of the parameters is null.
	 */
	public RazorAHRS(BluetoothDevice btDevice, RazorListener razorListener)
			throws RuntimeException {
		this(btDevice, razorListener, RazorOutputMode.YAW_PITCH_ROLL_ANGLES);
	}

	/**
	 * Constructor.
	 * Must be called from the thread where you want receive the RazorListener callbacks! So if you
	 * want to manipulate Android UI from the callbacks you have to call this from your main/UI
	 * thread.
	 *
	 * @param btDevice {@link android.bluetooth.BluetoothDevice BluetoothDevice} holding the Razor
	 * 		AHRS to connect to.
	 * @param razorListener {@link RazorListener} that will be notified of Razor AHRS events.
	 * @param razorOutputMode {@link RazorOutputMode} that you desire.
	 * @throws RuntimeException thrown if one of the parameters is null.
	 */
	public RazorAHRS(BluetoothDevice btDevice, RazorListener razorListener, RazorOutputMode razorOutputMode)
			throws RuntimeException {
		if (btDevice == null)
			throw new RuntimeException("BluetoothDevice can not be null.");
		this.btDevice = btDevice;

		if (razorListener == null)
			throw new RuntimeException("RazorListener can not be null.");
		this.razorListener = razorListener;

		if (razorOutputMode == null)
			throw new RuntimeException("RazorMode can not be null.");
		this.razorOutputMode = razorOutputMode;
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
	 * Connect and start reading. Both is done asynchronously. {@link RazorListener#onConnectOk()}
	 * or {@link RazorListener#onConnectFail(IOException)} callbacks will be invoked.
	 * 
	 * @param numConnectAttempts Number of attempts to make when trying to connect. Often connecting
	 * 		only works on the 2rd try or later. Bluetooth hooray.
	 */
	public void asyncConnect(int numConnectAttempts) {
		if (DEBUG) Log.d(TAG, "asyncConnect() BEGIN");
		// Disconnect and wait for running thread to end, if needed
		if (btThread != null) {
			asyncDisconnect();
			try {
				btThread.join();
			} catch (InterruptedException e) { }
		}

		// Bluetooth thread not running any more, we're definitely in DISCONNECTED state now

		// Create new thread to connect to Razor AHRS and read input
		this.numConnectAttempts = numConnectAttempts;
		connectionState = ConnectionState.CONNECTING;
		btThread = new BluetoothThread();
		btThread.start();
		if (DEBUG) Log.d(TAG, "asyncConnect() END");
	}

	/**
	 * Disconnects from Razor AHRS. If still connecting this will also cancel the connection process.
	 */
	public void asyncDisconnect() {
		if (DEBUG) Log.d(TAG, "asyncDisconnect() BEGIN");
		synchronized (connectionState) {
			if (DEBUG) Log.d(TAG, "asyncDisconnect() SNYNCHRONIZED");
			// Don't go to USER_DISCONNECT_REQUEST state if we are disconnected already
			if (connectionState == ConnectionState.DISCONNECTED)
				return;

			// This is a wanted disconnect, so we force (blocking) I/O to break
			connectionState = ConnectionState.USER_DISCONNECT_REQUEST;
			closeSocketAndStreams();
		}
		if (DEBUG) Log.d(TAG, "asyncDisconnect() END");
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
	 * Closes I/O streams and Bluetooth socket.
	 */
	private void closeSocketAndStreams() {
		if (DEBUG) Log.d(TAG, "closeSocketAndStreams() BEGIN");
		// Try to switch off streaming output of Razor in preparation of next connect
		try {
			if (outStream != null)
				write("#o0");
		} catch (IOException e) { }

		// Close Bluetooth socket => I/O operations immediately will throw exception
		try {
			if (btSocket != null)
				btSocket.close();
		} catch (IOException e) { }
		if (DEBUG) Log.d(TAG, "closeSocketAndStreams() BT SOCKET CLOSED");
		
		// Close streams
		try {
			if (inStream != null)
				inStream.close();
		} catch (IOException e) { }
		try {
			if (outStream != null)
				outStream.close();
		} catch (IOException e) { }
		if (DEBUG) Log.d(TAG, "closeSocketAndStreams() STREAMS CLOSED");

		// Do not set socket and streams null, because input thread might still access them
		//inStream = null;
		//outStream = null;
		//btSocket = null;
		
		if (DEBUG) Log.d(TAG, "closeSocketAndStreams() END");
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
		 * Converts a buffer of bytes to an array of floats. This method does not do any error
		 * checking on parameter array sizes.
		 * @param byteBuf Byte buffer with length of at least <code>numFloats  * 4</code>.
		 * @param floatArr Float array with length of at least <code>numFloats</code>.
		 * @param numFloats Number of floats to convert
		 */
		private void byteBufferToFloatArray(byte[] byteBuf, float[] floatArr, int numFloats) {
			//int numFloats = byteBuf.length / 4;
			for (int i = 0; i < numFloats * 4; i += 4) {
				// Convert from little endian (Razor) to big endian (Java) and interpret as float
				floatArr[i/4] = Float.intBitsToFloat((byteBuf[i] & 0xff) + ((byteBuf[i+1] & 0xff) << 8) +
						((byteBuf[i+2] & 0xff) << 16) + ((byteBuf[i+3] & 0xff) << 24));
			}
		}

		/**
		 * Parse input stream for given token.
		 * @param token Token to find
		 * @param in Next byte from input stream
		 * @return <code>true</code> if token was found
		 */
		private boolean readToken(byte[] token, byte in) {
			if (in == token[inBufPos++]) {
				if (inBufPos == token.length) {
					// Synch token found
					inBufPos = 0;
					if (DEBUG) Log.d(TAG, "Token found");
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

			/* See if Razor is there */
			// Request synch token to see when Razor is up and running
			final String contactSynchID = "00";
			final String contactSynchRequest = "#s" + contactSynchID;
			final byte[] contactSynchReply = EncodingUtils.getAsciiBytes(SYNCH_TOKEN + contactSynchID + NEW_LINE);
			write(contactSynchRequest);
			t1 = SystemClock.uptimeMillis();

			while (true) {
				// Check timeout
				t2 = SystemClock.uptimeMillis();
				if (t2 - t1 > 200) {
					// 200ms elapsed since last request and no answer -> request synch again.
					// (This happens when DTR is connected and Razor resets on connect)
					write(contactSynchRequest);
					t1 = t2;
				}
				if (t2 - t0 > INIT_TIMEOUT_MS)
					// Timeout -> tracker not present
					throw new IOException("Can not init Razor: response timeout");

				// See if we can read something
				if (inStream.available() > 0) {
					// Synch token found?
					if (readToken(contactSynchReply, readByte()))
						break;
				} else {
					// No data available, wait
					delay(5); // 5ms
				}
			}

			/* Configure tracker */
			// Set binary output mode, enable continuous streaming, disable errors and request synch
			// token. So we're good, no matter what state the tracker currently is in.
			final String configSynchID = "01";
			final byte[] configSynchReply = EncodingUtils.getAsciiBytes(SYNCH_TOKEN + configSynchID + NEW_LINE);
			write("#ob#o1#oe0#s" + configSynchID);
			while (!readToken(configSynchReply, readByte())) { }
		}

		/**
		 * Opens Bluetooth connection to Razor AHRS.
		 * @throws IOException
		 */
		private void connect() throws IOException {		
			// Create Bluetooth socket
			btSocket = btDevice.createRfcommSocketToServiceRecord(UUID_SPP);
			if (btSocket == null) {
				if (DEBUG) Log.d(TAG, "btSocket is null in connect()");
				throw new IOException("Could not create Bluetooth socket");
			}

			// This could be used to create the RFCOMM socekt on older Android devices where
			//createRfcommSocketToServiceRecord is not present yet.
			/*try {
			Method m = btDevice.getClass().getMethod("createRfcommSocket", new Class[] { int.class });
			btSocket = (BluetoothSocket) m.invoke(btDevice, Integer.valueOf(1));
			} catch (Exception e) {
				throw new IOException("Could not create Bluetooth socket using reflection");
			}*/

			// Connect socket to Razor AHRS
			if (DEBUG) Log.d(TAG, "Canceling bt discovery");
			BluetoothAdapter.getDefaultAdapter().cancelDiscovery();	// Recommended
			if (DEBUG) Log.d(TAG, "Trying to connect() btSocket");
			btSocket.connect();

			// Get the input and output streams
			if (DEBUG) Log.d(TAG, "Trying to create streams");
			inStream = btSocket.getInputStream();
			outStream = btSocket.getOutputStream();
			if (inStream == null || outStream == null) {
				if (DEBUG) Log.d(TAG, "Could not create I/O stream(s) in connect()");
				throw new IOException("Could not create I/O stream(s)");
			}
		}

		/**
		 * Bluetooth I/O thread entry method.
		 */
		public void run() {
			if (DEBUG) Log.d(TAG, "Bluetooth I/O thread started");
			try {
				// Check if btDevice is set
				if (btDevice == null) {
					if (DEBUG) Log.d(TAG, "btDevice is null in run()");
					throw new IOException("Bluetooth device is null");
				}

				// Make several attempts to connect
				int i = 1;
				while (true) {
					if (DEBUG) Log.d(TAG, "Connect attempt " + i + " of " + numConnectAttempts);
					sendToParentThread(MSG_ID__CONNECT_ATTEMPT, i);
					try {
						connect();
						break;	// Alrighty!
					} catch (IOException e) {
						if (DEBUG) Log.d(TAG, "Attempt failed: " + e.getMessage());
						// Maximum number of attempts reached or cancel requested?
						if (i == numConnectAttempts || connectionState == ConnectionState.USER_DISCONNECT_REQUEST)
							throw e;

						// We couldn't connect on first try, manually starting Bluetooth discovery
						// often helps
						if (DEBUG) Log.d(TAG, "Starting BT discovery");
						BluetoothAdapter.getDefaultAdapter().startDiscovery();
						delay(5000); // 5 seconds - long enough?

						i++;
					}
				}

				// Set Razor output mode
				if (DEBUG) Log.d(TAG, "Trying to set Razor output mode");
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
				if (DEBUG) Log.d(TAG, "Starting input loop");
				while (true) {
					// Read byte from input stream
					inBuf[inBufPos++] = (byte) readByte();

					if (razorOutputMode == RazorOutputMode.YAW_PITCH_ROLL_ANGLES) {
						if (inBufPos == 12) {	// We received a full frame
							float[] ypr = float3Pool.get();
							byteBufferToFloatArray(inBuf, ypr, 3);

							// Forward to parent thread handler
							sendToParentThread(MSG_ID__YPR_DATA, ypr);

							// Rewind input buffer position
							inBufPos = 0;
						}
					} else {	// Raw or calibrated sensor data mode
						if (inBufPos == 36) {	// We received a full frame
							float[] amg = float9Pool.get();
							byteBufferToFloatArray(inBuf, amg, 9);

							// Forward to parent thread handler
							sendToParentThread(MSG_ID__AMG_DATA, amg);

							// Rewind input buffer position
							inBufPos = 0;
						}
					}
				}
			} catch (IOException e) {
				if (DEBUG) Log.d(TAG, "IOException in Bluetooth thread: " + e.getMessage());
				synchronized (connectionState) {
					// Don't forward exception if it was thrown because we broke I/O on purpose in
					// other thread when user requested disconnect
					if (connectionState != ConnectionState.USER_DISCONNECT_REQUEST) {
						// There was a true I/O error, cleanup and forward exception
						closeSocketAndStreams();
						if (DEBUG) Log.d(TAG, "Forwarding exception");
						if (connectionState == ConnectionState.CONNECTING)
							sendToParentThread(MSG_ID__CONNECT_FAIL, e);
						else
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
				sleep(ms); // Sleep 5ms
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
				case MSG_ID__YPR_DATA:	// Yaw, pitch and roll data
					float[] ypr = (float[]) msg.obj;
					razorListener.onAnglesUpdate(ypr[0], ypr[1], ypr[2]);
					float3Pool.put(ypr);
					break;
				case MSG_ID__AMG_DATA:	// Accelerometer, magnetometer and gyroscope data
					float[] amg = (float[]) msg.obj;
					razorListener.onSensorsUpdate(amg[0], amg[1], amg[2], amg[3], amg[4], amg[5],
							amg[6], amg[7], amg[8]);
					float9Pool.put(amg);
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
