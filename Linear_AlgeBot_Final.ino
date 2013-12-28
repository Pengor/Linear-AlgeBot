// Drew Johnson and Tom Kreamer
// Pacific Lutheran University 
// Course: MATH 331 - Linear Algebra
// Instructor:  Ksenija Simic-Muller
// Linear Algebra/GPS navigating robot

// External Code libraries used
#include <SoftwareSerial.h> // Arduino.cc
#include <TinyGPS++.h> // http://arduiniana.org/libraries/tinygpsplus/
#include <MatrixMath.h> // http://playground.arduino.cc/Code/MatrixMath

// Microcontroller I/O Pin setup
const int lw_in1 = 1, lw_in2 = 2, lw_pwm = 3; // Left wheel control
const int rw_in1 = 12, rw_in2 = 11, rw_pwm = 10; // Right wheel control
const int RXPin = 8, TXPin = 9; // GPS Receiver I/O

const float pi = 3.14159265; // Define value of pi

// GPS Receiver setup
static const uint32_t GPSBaud = 4800; // Set serial communication baud rate
TinyGPSPlus gps; // Create gps object needed to use TinyGPS++ library
SoftwareSerial ss(RXPin, TXPin); // Define input and output pins for communication

// User defined inputs
float radius = 5; // The radius of the circle to be approximated
const int sides = 6; // Number of sides in the circle-approximating n-gon

// Global variables used for calculations
double initialGPS[2]; // initial robot latititude/longitude
float interAngle; // The interior angle at each vertex of the n-gon
float turnDegr; // The angle the robot must turn at each n-gon vertex
float initTurn; // The initial angle the robot must turn after reaching 1st vertex
float distToTarget; // Used to compare current position to next vertex

// Code that runs before main loop
void setup() {
  // Set signal types for wheel pins
  pinMode(lw_in1, OUTPUT);
  pinMode(lw_in2, OUTPUT);
  pinMode(lw_pwm, OUTPUT);
  pinMode(rw_in1, OUTPUT);
  pinMode(rw_in2, OUTPUT);
  pinMode(rw_pwm, OUTPUT);
  
  // Begin Transmitting Data to/from GPS Receiver
  Serial.begin(115200); // Set rate of data flow to/from PC, for debugging
  ss.begin(GPSBaud); // Start the data flow to the GPS receiver
  
  interAngle = (sides - 2) * 180 / sides;  // Calculate interior angle
  turnDegr = 180 - interAngle;             // Calculate angle of turn
  initTurn = 90 + (180 - interAngle) / 2;  // Calculate initial turn angle
}

// Main program instructions
void loop() {
  int gpsGood = 0; // Boolean flag: becomes 1 if initial coordinates are found

  // Loop until initial coordinates are determined by GPS receiver
  while (true) { 
    while (ss.available() > 0) {
      if (gps.encode(ss.read()) && gps.location.isValid()) {
        setInitialCoord(); // Sets initial coordinates
        gpsGood = 1; // Set boolean flag
        break; // Exit inner loop
      }
    }
    if (gpsGood == 1)
      break; // Exit outer loop
  }
 
  forward(radius); // Go forward to the n-gon perimeter
  turnLeft(initTurn); // Make the initial turn for first side
  
  // Determine the angle (at the n-gon center) between vertices
  float phi = (360 / sides) * (pi / 180);
  
  double destination[2]; // The destination coordinates
  double initialVect[2]; // Coordinates of initial vertex when center = (0,0)
  double tempLoc[2];     // Holds 
  float A[2][2];         // The transformation matrix

  // Setup the initial vector (center to first vertex)
  getCoord(initialVect); // Get latitude/longitude
  initialVect[0] -= initialGPS[0]; // Put X in terms of center = (0,0)
  initialVect[1] -= initialGPS[1]; // Put Y in terms of center = (0,0)
  
  // Repeat loop for each side of n-gon
  for (int i=0; i < sides; i++) {
    A[0][0] = cos(phi * i);   // Setup entries of rotation matrix
    A[0][1] = -sin(phi * i);  // phi is the angle between vertices
    A[1][0] = sin(phi * i);   // and i is number of the current side
    A[1][1] = cos(phi * i);
    
    // Perform the transformation by multiplying A by the initial vector,
    // the result is stored in the destination vector, representing the 
    // coordinates of the next vertex in the n-gon
    Matrix.Multiply((float*)A,(float*)initialVect, 2,2,1, (float*)destination);
    
    getCoord(tempLoc); // Get the current latitude and longitude
    tempLoc[0] -= initialGPS[0]+destination[0]; // Put X in terms of center = (0,0)
    tempLoc[1] -= initialGPS[1]+destination[1]; // Put Y in terms of center = (0,0)
    
    // inner part of distance formula, doesn't bother with costly square root calculation
    distToTarget = pow(tempLoc[0],2) + pow(tempLoc[1],2);
    
    double loopStart = millis(); // Save the system-time at loop's start
    // Send robot forward until it's location starts getting farther from the next vertex
    while (true) {
      forward((int)100); // Send robot forward for 100 milliseconds
      // If the robot is getting farther away from next vertex, stop moving forward
      // Only stops the robot after 3 seconds of moving forward to counteract GPS inaccuracies
      if (pow(tempLoc[0],2) + pow(tempLoc[1],2) > distToTarget && millis() - loopStart > 3000)
        break;
      else // Else set the new distance to target based on current position
        distToTarget = pow(tempLoc[0],2) + pow(tempLoc[1],2);
    }
    turnLeft(turnDegr); // Turn the robot left to prepare to head toward the next vertex
  }
  while (true); // Stops the main instructions from repeating by looping infinitely
}

////////////////////// Additional Helper Functions //////////////////////

// Sends robot forward for a given length of time
// lw_pwm and rw_pwm have different values to fix directional bias
void forward(int timeToRun) {
  digitalWrite(lw_in1, HIGH);
  digitalWrite(lw_in2, LOW);
  analogWrite(lw_pwm, 252);
  digitalWrite(rw_in1, LOW);
  digitalWrite(rw_in2, HIGH);
  analogWrite(rw_pwm, 255);
  delay(timeToRun);
}

// Sends robot forward for a given distance in feet
void forward(float radiusFt) {
  digitalWrite(lw_in1, HIGH);
  digitalWrite(lw_in2, LOW);
  analogWrite(lw_pwm, 252);
  digitalWrite(rw_in1, LOW);
  digitalWrite(rw_in2, HIGH);
  analogWrite(rw_pwm, 255);
  delay(radiusFt * 1000);
}

// Spins the robot counter-clockwise an angle in degrees
void turnLeft(int degr) {
  digitalWrite(lw_in1, LOW);
  digitalWrite(lw_in2, HIGH);
  analogWrite(lw_pwm, 0); // Left wheel stays put
  digitalWrite(rw_in1, LOW);
  digitalWrite(rw_in2, HIGH);
  analogWrite(rw_pwm, 200); // Turn using right wheel only
  delay(degr * 6); // The robot turns at about 6 ms per degree
  digitalWrite(lw_in1, LOW);
  digitalWrite(lw_in2, HIGH);
  analogWrite(lw_pwm, 0); // Stop left wheel
  digitalWrite(rw_in1, LOW);
  digitalWrite(rw_in2, HIGH);
  analogWrite(rw_pwm, 0); // Stop right wheel
}

// Accesses the GPS Receiver to determine initial lat/long
void setInitialCoord() {
  initialGPS[0] = gps.location.lat();
  initialGPS[1] = gps.location.lng();
}

// Accesses GPS Receiver to find current long/lat
void getCoord(double *xy) {
  int gpsIsGood = 0;
  while (true) { 
    while (ss.available() > 0) {
      if (gps.encode(ss.read()) && gps.location.isUpdated()) {
        xy[0] = gps.location.lat();
        xy[1] = gps.location.lng();
        gpsIsGood = 1;
        break;
      }
    }
    if (gpsIsGood == 1)
      break;
  }
}
