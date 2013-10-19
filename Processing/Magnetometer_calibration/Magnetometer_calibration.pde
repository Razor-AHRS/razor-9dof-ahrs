/******************************************************************************************
* Magnetometer Sampling Sketch for Razor AHRS v1.4.2
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

/*
  NOTE: There seems to be a bug with the serial library in Processing versions 1.5
  and 1.5.1: "WARNING: RXTX Version mismatch ...".
  Processing 2.0.x seems to work just fine. Later versions may too.
  Alternatively, the older version 1.2.1 also works and is still available on the web.
*/

/*
  IMPORTANT: You have to install a library, before this sketch can be run!
  We're using EJML for matrix math, because it's really fast:
  http://code.google.com/p/java-matrix-benchmark/
  Also, it's released under LGPL, which fits well with our GPL.
  Get the library from: http://code.google.com/p/efficient-java-matrix-library/ (you only need
  the .jar file), find your Processing "libraries" folder (normally this is Processing/libraries
  in your user documents folder). Create a folder "EJML" inside "libraries",
  create a folder "library" inside "EJML" and put the .jar inside. Rename to EJML.jar. So you
  should have "libraries/EJML/library/EJML.jar". Restart Processing and you're good.
  More info on installing libraries in Processing: http://wiki.processing.org/w/How_to_Install_a_Contributed_Library
  Tested to be working with EJML 0.17 and 0.23.
*/
import org.ejml.data.*;
import org.ejml.simple.*;
import org.ejml.ops.*;

// IF THE SKETCH CRASHES OR HANGS ON STARTUP, MAKE SURE YOU ARE USING THE RIGHT SERIAL PORT:
// 1. Have a look at the Processing console output of this sketch.
// 2. Look for the serial port list and find the port you need (it's the same as in Arduino).
// 3. Set your port number here:
final static int SERIAL_PORT_NUM = 0;
// 4. Try again.



import processing.opengl.*;
import processing.serial.*;
import java.io.*;

final static int SERIAL_PORT_BAUD_RATE = 57600;

final static int NUM_MAGN_SAMPLES = 10000;
float magnetom[][] = new float[NUM_MAGN_SAMPLES][3];
int magnetomIndex = 0;

PFont font;
Serial serial;

boolean synched = false;

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
  println("Trying to setup and synch Razor...\n");
  
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
    translate(width/2, height/2, -900);
    
    // Draw sphere and background once
    if (magnetomIndex == 0) {
      background(0);
      noFill();
      stroke(255);
      sphereDetail(10);
      sphere(400);
      fill(200);
      text("Press 'r' to reset. Press SPACE to output calibration parameters to console and quit.", 0, 1100, -600);
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
    case ' ':  // Calculate and output calibration parameters, output binary magnetometer samples file, quit
    case ENTER:
    case RETURN:
      outputCalibration();  // Do the magic
      exit();  // We're done
      break;
    case 'r':  // Reset samples and clear screen
      magnetomIndex = 0;
      break;
  }
}

void outputCalibration() {
  /* ELLIPSOID FITTING CODE */
  // Adaption of 'ellipsoid_fit' matlab code by Yury Petrov (See 'Matlab' folder
  // that came with the archive containing this file).
  
  // Check if we have at least 9 sample points
  if (magnetomIndex < 9) {
    println("ERROR: not enough magnetometer samples. We need at least 9 points.");
    exit();
  }
  
  // Seperate xyz magnetometer values and make column vectors
  SimpleMatrix x = new SimpleMatrix(magnetomIndex, 1);
  SimpleMatrix y = new SimpleMatrix(magnetomIndex, 1);
  SimpleMatrix z = new SimpleMatrix(magnetomIndex, 1);
  for (int i = 0; i < magnetomIndex; i++) {
    x.set(i, magnetom[i][0]);
    y.set(i, magnetom[i][1]);
    z.set(i, magnetom[i][2]);
  }
  
  
  /*
  % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
  D = [ x .* x, ...
        y .* y, ...
        z .* z, ...
    2 * x .* y, ...
    2 * x .* z, ...
    2 * y .* z, ...
    2 * x, ...
    2 * y, ... 
     2 * z ];  % ndatapoints x 9 ellipsoid parameters
  */
  SimpleMatrix D = new SimpleMatrix(x.numRows(), 9);
  D.insertIntoThis(0, 0, x.elementMult(x));
  D.insertIntoThis(0, 1, y.elementMult(y));
  D.insertIntoThis(0, 2, z.elementMult(z));
  D.insertIntoThis(0, 3, x.elementMult(y).scale(2));
  D.insertIntoThis(0, 4, x.elementMult(z).scale(2));
  D.insertIntoThis(0, 5, y.elementMult(z).scale(2));
  D.insertIntoThis(0, 6, x.scale(2));
  D.insertIntoThis(0, 7, y.scale(2));
  D.insertIntoThis(0, 8, z.scale(2));
  
  /*
  % solve the normal system of equations
  v = ( D' * D ) \ ( D' * ones( size( x, 1 ), 1 ) );
  */
  SimpleMatrix tempA = D.transpose().mult(D);
  SimpleMatrix ones = x.copy(); ones.set(1);
  SimpleMatrix tempB = D.transpose().mult(ones);
  SimpleMatrix v = tempA.solve(tempB);
  
  /*
  % form the algebraic form of the ellipsoid
  A = [ v(1) v(4) v(5) v(7); ...
        v(4) v(2) v(6) v(8); ...
        v(5) v(6) v(3) v(9); ...
        v(7) v(8) v(9) -1 ];
  */
  SimpleMatrix A = new SimpleMatrix(new double[][]
    {{v.get(0), v.get(3), v.get(4), v.get(6)},
     {v.get(3), v.get(1), v.get(5), v.get(7)},
     {v.get(4), v.get(5), v.get(2), v.get(8)},
     {v.get(6), v.get(7), v.get(8),     -1.0}});
  
  /*
  % find the center of the ellipsoid
  center = -A( 1:3, 1:3 ) \ [ v(7); v(8); v(9) ];
  */
  SimpleMatrix center = A.extractMatrix(0, 3, 0, 3).scale(-1).solve(v.extractMatrix(6, 9, 0, 1));

  /*
  % form the corresponding translation matrix
  T = eye( 4 );
  T( 4, 1:3 ) = center';
  */
  SimpleMatrix T = SimpleMatrix.identity(4);
  T.insertIntoThis(3, 0, center.transpose());
  
  /*
  % translate to the center
  R = T * A * T';
  % solve the eigenproblem
  [ evecs evals ] = eig( R( 1:3, 1:3 ) / -R( 4, 4 ) );
  radii = sqrt( 1 ./ diag( evals ) );
  */
  SimpleMatrix R = T.mult(A).mult(T.transpose());
  SimpleEVD evd = R.extractMatrix(0, 3, 0, 3).divide(-R.get(3, 3)).eig();

  SimpleMatrix evecs = new SimpleMatrix(3, 3);
  evecs.insertIntoThis(0, 0, evd.getEigenVector(0));
  evecs.insertIntoThis(0, 1, evd.getEigenVector(1));
  evecs.insertIntoThis(0, 2, evd.getEigenVector(2));
  
  SimpleMatrix radii = new SimpleMatrix(new double[][]
    {{Math.sqrt(1.0 / evd.getEigenvalue(0).getReal()),
      Math.sqrt(1.0 / evd.getEigenvalue(1).getReal()),
      Math.sqrt(1.0 / evd.getEigenvalue(2).getReal())}});

  //center.print();
  //evecs.print();
  //radii.print();

  
  /* CALCULATE COMPENSATION MATRIX */
  // Adaption of my 'magnetometer_calibration' matlab code. (See 'Matlab' folder
  // that came with the archive containing this file).
  /*
  % compensate distorted magnetometer data
  % e_eigenvecs is an orthogonal matrix, so ' can be used instead of inv()
  scale = inv([e_radii(1) 0 0; 0 e_radii(2) 0; 0 0 e_radii(3)]) * min(e_radii); % scaling matrix
  map = e_eigenvecs'; % transformation matrix to map ellipsoid axes to coordinate system axes
  invmap = e_eigenvecs; % inverse of above
  comp = invmap * scale * map;
  */
  SimpleMatrix scale = new SimpleMatrix(3, 3);
  scale.zero();
  scale.set(0, 0, radii.get(0));
  scale.set(1, 1, radii.get(1));
  scale.set(2, 2, radii.get(2));
  scale = scale.invert().scale(CommonOps.elementMin(radii.getMatrix()));
  
  SimpleMatrix map = evecs.transpose();
  SimpleMatrix invmap = evecs;
  SimpleMatrix comp = invmap.mult(scale).mult(map);
  //comp.print();
  
  /* OUTPUT RESULTS */
  // Output magnetometer samples file
  try {
    println("Trying to write " + magnetomIndex + " sample points to file magnetom.float ...");
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
  println("\n");

  // Output calibration
  System.out.printf("In the Razor_AHRS.ino, under 'SENSOR CALIBRATION' find the section that reads 'Magnetometer (extended calibration)'\n");
  System.out.printf("Replace the existing 3 lines with these:\n\n");
  System.out.printf("#define CALIBRATION__MAGN_USE_EXTENDED true\n");
  System.out.printf("const float magn_ellipsoid_center[3] = {%.6g, %.6g, %.6g};\n", center.get(0), center.get(1), center.get(2));
  System.out.printf("const float magn_ellipsoid_transform[3][3] = {{%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}};\n",
    comp.get(0), comp.get(1), comp.get(2), comp.get(3), comp.get(4), comp.get(5), comp.get(6), comp.get(7), comp.get(8));
  println("\n");
}
