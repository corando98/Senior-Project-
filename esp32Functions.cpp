/* Include necessary libraries */
// Arduino & Serial communication:
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <HardwareSerial.h>
// Sensors (Ultrasonic, Compass, BMI):
#include <NewPing.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <BMI160Gen.h>
//#include <CurieIMU.h>
// Motors & Servos:
#include <L298N.h>
#include <ESP32Servo.h>

/* Library and variable initialization for degree calculations from BMI160 */
#include <MadgwickAHRS.h>
Madgwick filter;
unsigned long microsPerReading, microsPrevious;


/* Assign pin names, based on GPIO number: */
// Servos
const int leftArmPWM = 17;
const int rightArmPWM = 16;
// Motor Drivers
const int trayMDriverIn2 = 15;
const int trayMDriverIn1 = 2;
const int trackMDriverIn4 = 26;
const int trackMDriverIn3 = 27;
const int trackMDriverIn2 = 14;
const int trackMDriverIn1 = 12;
const int trayEnable = 4;
const int track1Enable = 13;
const int track2Enable = 25;
// Ultrasonic Sensors
const int frontUltrasonicEcho = 34;
const int frontUltrasonicTrig = 32;
const int binUltrasonicEcho = 35;
const int binUltrasonicTrig = 33;
// UART Communication w/ RPi
const int espRX = 3;
const int espTX = 1;
// I2C Communication w/ Compass
const int I2CSerialClock = 22;
const int CompassData = 21;
// SPI Communication w/ BMI
const int BMIChipSelect = 5;
const int BMISerialClock = 18;
const int BMIMISO = 19;
const int BMIMOSI = 23;

/* Initialize necessary structures (from libraries): */
// Servos
Servo leftArmServo;
Servo rightArmServo;
Servo handServos;
ESP32PWM pwm;
// Motor Drivers
L298N rightMotor(track1Enable, trackMDriverIn1, trackMDriverIn2);
L298N leftMotor(track2Enable, trackMDriverIn3, trackMDriverIn4);
L298N trayMotor(trayEnable, trayMDriverIn1, trayMDriverIn2);
// Ultrasonic Sensors
NewPing sonar(frontUltrasonicTrig, frontUltrasonicEcho, 200);
NewPing binFill(binUltrasonicTrig, binUltrasonicEcho, 200);
// UART Serial Comm w/ RPi
HardwareSerial MySerial(1);
// Compass
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  // put your setup code here, to run once:

  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  leftArmServo.setPeriodHertz(50);      // Standard 50hz servo
  rightArmServo.setPeriodHertz(50);      // Standard 50hz servo
  handServos.setPeriodHertz(50);      // Standard 50hz servo

  // Initialize the I2C library
  Wire.begin();
  
  // Initialize serial communication with a 9600 baud rate, through microUSB
  Serial.begin(9600);
  while (!Serial);  // wait for the serial port to open
  Serial.println("USB Serial Initialized"); // output to console when opened

  // Initialize serial communication to RPi (9600 bps, 8 bits, No Parity, 1 Stop Bit)
  //MySerial.begin(9600, SERIAL_8N1, espRX, espTX);
  //MySerial.begin(9600, SERIAL_8N1, 16, 17);
  //MySerial.setDebugOutput(true);
  //while(!MySerial); // wait for serial port to open
  Serial.println("Serial Initialized"); // output to console when opened
  
  // Initialize the SPI library
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);  //divide the clock by 4

  // Set pins mode for each output
  pinMode(leftArmPWM, OUTPUT);
  pinMode(rightArmPWM, OUTPUT);
  pinMode(trayMDriverIn2, OUTPUT);
  pinMode(trayMDriverIn1, OUTPUT);
  pinMode(trayEnable, OUTPUT);
  pinMode(trackMDriverIn4, OUTPUT);
  pinMode(trackMDriverIn3, OUTPUT);
  pinMode(trackMDriverIn2, OUTPUT);
  pinMode(trackMDriverIn1, OUTPUT);
  pinMode(track1Enable, OUTPUT);
  pinMode(track2Enable, OUTPUT);
  pinMode(frontUltrasonicTrig, OUTPUT);
  pinMode(binUltrasonicTrig, OUTPUT);
  pinMode(BMIChipSelect, OUTPUT);
  // Set pins mode for each input
  pinMode(frontUltrasonicEcho, INPUT);
  pinMode(binUltrasonicEcho, INPUT);
  Serial.println("Pins Defined"); // output to console that pins were defined


  // digitalWrite(trayEnable, HIGH);
  // ledcSetup(0, 30000, 8);
  // ledcAttachPin(trayEnable, 0);

  // Give sensors & serial communication time to initialize
  delay(100);
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;

  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

void forward() {
  digitalWrite(trackMDriverIn1, HIGH);
  digitalWrite(trackMDriverIn3, HIGH);

  digitalWrite(trackMDriverIn2, LOW);
  digitalWrite(trackMDriverIn4, LOW);
}

void reverse() {
  digitalWrite(trackMDriverIn1, LOW);
  digitalWrite(trackMDriverIn3, LOW);

  digitalWrite(trackMDriverIn2, HIGH);
  digitalWrite(trackMDriverIn4, HIGH);
}

void turn(double degree) {

}

void stopDriving() {
  digitalWrite(trackMDriverIn1, HIGH);
  digitalWrite(trackMDriverIn2, HIGH);

  digitalWrite(trackMDriverIn3, HIGH);
  digitalWrite(trackMDriverIn4, HIGH);
}

void openTray() {

}

void closeTray() {

}

int bigMinUs = 365; // 365 for MG996R
int bigMaxUs = 2470; // 2460-2480 for MG996R

void lowerArms() {
  int pos = 0;
  leftArmServo.attach(leftArmPWM, bigMinUs, bigMaxUs);
  rightArmServo.attach(rightArmPWM, bigMinUs, bigMaxUs);
  pwm.attachPin(27, 10000);//10khz

  for (pos = 160; pos >= 50; pos -= 1) { // sweep from 180 degrees to 0 degrees
		rightArmServo.write(pos);
    leftArmServo.write((160-pos)+50);
		delay(10);
	}

  MySerial.print("Left servo value: ");
  MySerial.println(leftArmServo.read());
  MySerial.print("Right servo value: ");
  MySerial.println(rightArmServo.read());

  rightArmServo.detach();
  leftArmServo.detach();
  pwm.detachPin(27);
}

void raiseArms() {
  int pos = 0;
  leftArmServo.attach(leftArmPWM, bigMinUs, bigMaxUs);
  rightArmServo.attach(rightArmPWM, bigMinUs, bigMaxUs);
  pwm.attachPin(27, 10000);//10khz

  for (pos = 50; pos <= 160; pos += 1) { // sweep from 0 degrees to 180 degrees
		// in steps of 1 degree
		rightArmServo.write(pos);
    leftArmServo.write((160-pos)+50);
		delay(20);             // waits 20ms for the servo to reach the position
	}

  MySerial.print("Left servo value: ");
  MySerial.println(leftArmServo.read());
  MySerial.print("Right servo value: ");
  MySerial.println(rightArmServo.read());

  rightArmServo.detach();
  leftArmServo.detach();
  pwm.detachPin(27);
}

int minUs = 500;
int maxUs = 2400;

void closeHands() {
  int pos = 0;
  handServos.attach(trayEnable, minUs, maxUs);
  pwm.attachPin(27, 10000);//10khz

  for (pos = 10; pos <= 170; pos += 1) { // sweep from 0 degrees to 180 degrees
		// in steps of 1 degree
		handServos.write(pos);
		delay(10);             // waits 20ms for the servo to reach the position
	}

  Serial.print("Hand servo value: ");
  Serial.println(handServos.read());

  handServos.detach();
  pwm.detachPin(27);
}

void openHands() {
  int pos = 0;
  handServos.attach(trayEnable, minUs, maxUs);
  pwm.attachPin(27, 10000);//10khz

  for (pos = 170; pos >= 10; pos -= 1) { // sweep from 0 degrees to 180 degrees
		// in steps of 1 degree
		handServos.write(pos);
		delay(10);             // waits 20ms for the servo to reach the position
	}

  Serial.print("Hand servo value: ");
  Serial.println(handServos.read());

  handServos.detach();
  pwm.detachPin(27);
}

void readFrontUltrasonic() {
  Serial.print("Sonar Ping: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
}

void readBinUltrasonic() {
  Serial.print("Bin Ping: ");
  Serial.print(binFill.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
}



byte number = 6;
byte input = 0;

void loop() {
  // put your main code here, to run repeatedly:

  //digitalWrite(trayEnable, HIGH);

  //pwm.attachPin(27, 30000, 8);//30khz
  // trayMotor.setSpeed(200);

  // // Move the DC motor forward at maximum speed
  // Serial.println("Moving Forward");
  // // digitalWrite(trayMDriverIn1, LOW);
  // // digitalWrite(trayMDriverIn2, HIGH); 
  // // digitalWrite(trayEnable, HIGH);
  // trayMotor.forward();
  // delay(2000);

  // // Stop the DC motor
  // Serial.println("Motor stopped");
  // // digitalWrite(trayMDriverIn1, LOW);
  // // digitalWrite(trayMDriverIn2, LOW);
  // // digitalWrite(trayEnable, LOW);
  // trayMotor.stop();
  // delay(1000);

  // trayMotor.setSpeed(100);

  // // Move DC motor backwards at maximum speed
  // Serial.println("Moving Backwards");
  // // digitalWrite(trayMDriverIn1, HIGH);
  // // digitalWrite(trayMDriverIn2, LOW);
  // // digitalWrite(trayEnable, HIGH); 
  // trayMotor.backward();
  // delay(2000);

  // // Stop the DC motor
  // Serial.println("Motor stopped");
  // // digitalWrite(trayMDriverIn1, LOW);
  // // digitalWrite(trayMDriverIn2, LOW);
  // // digitalWrite(trayEnable, LOW);
  // trayMotor.stop();
  // delay(1000);

  //int dutyCycle = 200;
  // Move DC motor forward with increasing speed
  // digitalWrite(trayMDriverIn1, HIGH);
  // digitalWrite(trayMDriverIn2, LOW);
  // while (dutyCycle <= 255){
  //   ledcWrite(trayEnable, dutyCycle);   
  //   Serial.print("Forward with duty cycle: ");
  //   Serial.println(dutyCycle);
  //   dutyCycle = dutyCycle + 5;
  //   delay(500);
  // }
  // dutyCycle = 200;
  unsigned long ping = 0.0;
  Serial.print("Motor forward for ");
  ping = sonar.ping();
  Serial.print(ping);
  Serial.println(" ms");
  trayMotor.forwardFor(ping);
  trayMotor.reset();

  delay(1000);

  // Stop the DC motor
  // Serial.println("Motor stopped");
  // // digitalWrite(trayMDriverIn1, LOW);
  // // digitalWrite(trayMDriverIn2, LOW);
  // // digitalWrite(trayEnable, LOW);
  // trayMotor.stop();

  //pwm.detachPin(27);


  // readFrontUltrasonic();
  // raiseArms();
  // delay(2000);
  // readBinUltrasonic();
  // lowerArms();
  // delay(5000);
  
  // Logical Pseudocode:
  // if (Serial.available()) {}
  // then retrieve degree change needed and call turn function
  // after turning, check with compass that 
}
