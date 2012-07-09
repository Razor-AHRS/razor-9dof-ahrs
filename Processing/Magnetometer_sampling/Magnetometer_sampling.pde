/******************************************************************************************
* Magnetometer Sampling Sketch for Razor AHRS v1.4.1
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

/*
  NOTE: There seems to be a bug with the serial library in the latest Processing
  versions 1.5 and 1.5.1: "WARNING: RXTX Version mismatch ...". The previous version
  1.2.1 works fine and is still available on the web.
*/

import processing.opengl.*;
import processing.serial.*;

// IF THE SKETCH CRASHES OR HANGS ON STARTUP, MAKE SURE YOU ARE USING THE RIGHT SERIAL PORT:
// 1. Have a look at the Processing console output of this sketch.
// 2. Look for the serial port list and find the port you need (it's the same as in Arduino).
// 3. Set your port number here:
final static int SERIAL_PORT_NUM = 0;
// 4. Try again.


final static int SERIAL_PORT_BAUD_RATE = 57600;

final static int NUM_MAGN_SAMPLES = 10000;
float magnetom[][] = new float[NUM_MAGN_SAMPLES][3];
int magnetomIndex = 0;

PFont font;
Serial serial;

boolean synched = false;
boolean writeFileAndQuit = false;

// Skip incoming serial stream data until token is found
boolean readToken(Serial serial, String token) {
  // Wait until enough bytes are available
  if (serial.available() < token.length())
    return false;
  
  // Check if incoming bytes match token
  for (int i = 0; i < token.length(); i++) {
    if (serial.read() != token.charAt(i))
      return false;
  }
  
  return true;
}

// Global setup
void setup() {
  // Setup graphics
  size(800, 800, OPENGL);
  smooth();
  noStroke();
  frameRate(50);
  colorMode(HSB);
  
  // Load font
  font = loadFont("Univers-66.vlw");
  textFont(font);
  
  // Setup serial port I/O
  println("AVAILABLE SERIAL PORTS:");
  println(Serial.list());
  String portName = Serial.list()[SERIAL_PORT_NUM];
  println();
  println("HAVE A LOOK AT THE LIST ABOVE AND SET THE RIGHT SERIAL PORT NUMBER IN THE CODE!");
  println("  -> Using port " + SERIAL_PORT_NUM + ": " + portName);
  serial = new Serial(this, portName, SERIAL_PORT_BAUD_RATE);
}

void setupRazor() {
  println("Trying to setup and synch Razor...");
  
  // On Mac OSX and Linux (Windows too?) the board will do a reset when we connect, which is really bad.
  // See "Automatic (Software) Reset" on http://www.arduino.cc/en/Main/ArduinoBoardProMini
  // So we have to wait until the bootloader is finished and the Razor firmware can receive commands.
  // To prevent this, disconnect/cut/unplug the DTR line going to the board. This also has the advantage,
  // that the angles you receive are stable right from the beginning. 
  delay(3000);  // 3 seconds should be enough
  
  // Set Razor output parameters
  serial.write("#osrb");  // Turn on binary output of raw sensor data
  serial.write("#o1");    // Turn on continuous streaming output
  serial.write("#oe0");   // Disable error message output
  
  // Synch with Razor
  serial.clear();  // Clear input buffer up to here
  serial.write("#s00");  // Request synch token
}

float readFloat(Serial s) {
  // Convert from little endian (Razor) to big endian (Java) and interpret as float
  return Float.intBitsToFloat(s.read() + (s.read() << 8) + (s.read() << 16) + (s.read() << 24));
}

void skipBytes(Serial s, int numBytes) {
  for (int i = 0; i < numBytes; i++) {
    s.read();
  }  
}

void draw() {
  // Reset scene
  lights();

  // Sync with Razor 
  if (!synched) {
    background(0);
    textAlign(CENTER);
    fill(255);
    text("Connecting to Razor...", width/2, height/2, -200);
    
    if (frameCount == 2)
      setupRazor();  // Set ouput params and request synch token
    else if (frameCount > 2)
      synched = readToken(serial, "#SYNCH00\r\n");  // Look for synch token
    return;
  }
    
  // Output "max samples reached" message?
  if (magnetomIndex == NUM_MAGN_SAMPLES - 1) {
    fill(0, 255, 255);
    text("MAX NUMBER OF SAMPLES REACHED!", width/2, 0, -250);
    println("MAX NUMBER OF SAMPLES REACHED!");
  }
 
  pushMatrix(); {
    translate(width/2, height/2, -700);
    
    // Draw sphere and background once
    if (magnetomIndex == 0) {
      background(0);
      noFill();
      stroke(255);
      sphereDetail(10);
      sphere(400);
      fill(200);
      text("Press 'r' to reset. Press SPACE to save data to sketch directory and quit.", 0, 1100, -600);
    }
  
    // Read and draw new sample
    if (magnetomIndex < NUM_MAGN_SAMPLES && serial.available() >= 36) {
      // Read all available magnetometer data from serial port
      while (serial.available() >= 36) {
        // Skip accel data
        skipBytes(serial, 12);
        // Read magn data
        magnetom[magnetomIndex][0] = readFloat(serial);  // x
        magnetom[magnetomIndex][1] = readFloat(serial);  // y
        magnetom[magnetomIndex][2] = readFloat(serial);  // z
        // Skip gyro data
        skipBytes(serial, 12);
      }
      
      // Draw new point
      fill((magnetom[magnetomIndex][2] + 800) / 8, 255, 255);
      noStroke();
      translate(magnetom[magnetomIndex][0], magnetom[magnetomIndex][1], magnetom[magnetomIndex][2]);
      sphere(5);
      
      magnetomIndex++;
    }
  } popMatrix();
  
  // Done?
  if (writeFileAndQuit) {    
    // Output file
    try {
      println("Trying to write file magnetom.float ...");
      FileOutputStream fos = new FileOutputStream(sketchPath("magnetom.float"));
      DataOutputStream dos = new DataOutputStream(fos);
      for (int i = 0; i < magnetomIndex; i++) {
        dos.writeFloat(magnetom[i][0]);
        dos.writeFloat(magnetom[i][1]);
        dos.writeFloat(magnetom[i][2]);
      }
      fos.close();
      println("Done.");
    } catch(Exception e) {
      println("Exception: " + e.toString());
    }
    
    exit();
  }
}

void keyPressed() {
  switch (key) {
    case '0':  // Turn Razor's continuous output stream off
      serial.write("#o0");
      break;
    case '1':  // Turn Razor's continuous output stream on
      serial.write("#o1");
      break;
    case 'f':  // Request one single yaw/pitch/roll frame from Razor (use when continuous streaming is off)
      serial.write("#f");
      break;
    case ' ':  // Output binary magnetometer samples file
    case ENTER:
    case RETURN:
      writeFileAndQuit = true;
      break;
    case 'r':  // Reset samples and clear screen
      magnetomIndex = 0;
      break;
  }
}
