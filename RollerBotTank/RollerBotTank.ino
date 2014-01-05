/*
  (Demonstrates vertical sliders, gauges)
  
  This code is designed for a rollerbot with 'tank controls' - you control each motor directly and independently
  The controller consists of two vertical sliders (one for the left motor and one for the right motor)
  In addition to the sliders, the controller has two gauges to display the actual power being
    supplied to each motor. This can be useful because PWM below a certain value isn't strong enough
    to drive the motor at all, and it can also be useful to see the power we're sending separate from the direction

    Vertical slider for left motor sends on change: L?,
    Vertical slider for right motor sends on change: R?,
    Left gauge changes value when it receives: m9:?,
    Right gauge changes value with it received: m10:?,
*/

// Software serial to be used for bluetooth
#include <SoftwareSerial.h>
SoftwareSerial mySerial(P1_4, P1_5);

// Pin mappings
int pin_motor1Direction = P2_0;
int pin_motor2Direction = P2_3;
int pin_motor1Enable = P2_1;
int pin_motor2Enable = P2_2;

int pin_led1 = P1_0;
int pin_led2 = P1_6;


// Global variables (none!)

// Constants
const int FORWARD = HIGH;
const int BACKWARD = LOW;

const int MAXPWM = 255;
const int MINPWM = 0.75 * MAXPWM; // below this cutoff, didn't seem like the motors would reliably move the bot

// controller always send 0-1023 for 'analog' values
const int STICK_MIN = 0;
const int STICK_MAX = 1023;

const int STICK_MID = (STICK_MAX - STICK_MIN) / 2;

// calculate constants to create a 'deadzone' in the middle of a vertical slider - we use this to stop the motors
//   if the stick is near the middle, not just at the exact middle
//   specifically, if the computer tells us the stick value is between STICK_DEADZONE_BOTTOM and STICK_DEADZONE_TOP, we'll
//   stop the motors
const int STICK_DEADZONE = 0.10 * (STICK_MAX - STICK_MIN); // deadzone will cover the middle 10% of the stick range
const int STICK_DEADZONE_BOTTOM = STICK_MID - STICK_DEADZONE/2; // ~402
const int STICK_DEADZONE_TOP = STICK_MID + STICK_DEADZONE/2; // ~614


void setup() {
  // setup pin modes
  pinMode(pin_led1, OUTPUT);
  pinMode(pin_led2, OUTPUT);
  
  pinMode(pin_motor1Direction, OUTPUT);
  pinMode(pin_motor2Direction, OUTPUT);
  pinMode(pin_motor1Enable, OUTPUT);
  pinMode(pin_motor2Enable, OUTPUT);
  
  // initialize pin values - motors off, ready to move forward  
  digitalWrite(pin_motor1Enable, LOW);
  digitalWrite(pin_motor2Enable, LOW);
  digitalWrite(pin_motor1Direction, FORWARD);
  digitalWrite(pin_motor2Direction, FORWARD);
  
  
  mySerial.begin(9600);
  
  delay(1000);
  //btSetup();  // you can uncomment this out to setup bt module
}


void btSetup() {
  mySerial.print("AT+NAMERB Tank 0104"); // set bt module name to RB Tank 0104
  while(mySerial.available()) mySerial.read();
  
  flash(pin_led1);
  flash(pin_led2);
  
  delay(1000);
  
  mySerial.print("AT+PIN0104"); // set bt pin to 0104
  
  while(mySerial.available()) mySerial.read(); 
  flash(pin_led1);
  flash(pin_led2);
  mySerial.flush();
}

// flash leds to help with inital debugging of bluetooth setup
void flash() {
  flash(pin_led1); 
}

void flash(int pin) {
 digitalWrite(pin, HIGH);
 delay(250);
 digitalWrite(pin, LOW);
 delay(250); 
}




// loop /////////////////////////////////////////////

void loop()
{
  char btInput;
  int valueIn;
  
  // get the next character from bluetooth if available
  if(mySerial.available()) {
    btInput = mySerial.read();
  } else {
    btInput = ','; // no data available, just setting it to an unused value 
  }
  
  // figure out what to do based on the input
  if(btInput == 'L') { // left stick value is incoming!
    valueIn = mySerial.parseInt(); // read the value from the left stick
    setMotorPWM(pin_motor1Enable, pin_motor1Direction, valueIn);
  } else if(btInput == 'R') { // right stick value is incoming!
    valueIn = mySerial.parseInt(); // read the value from the right stick
    setMotorPWM(pin_motor2Enable, pin_motor2Direction, valueIn);
  } else if(btInput == 'i') { // request info; write it out over bluetooth to read in the terminal
    mySerial.print(MINPWM);
    mySerial.print(',');
    mySerial.print(MAXPWM);
    mySerial.print(',');
    mySerial.print(STICK_DEADZONE_BOTTOM);
    mySerial.print(',');
    mySerial.print(STICK_DEADZONE_TOP);
    mySerial.print(',');
  }
  
  delay(1);
}

// helper function to set left/right motor pwm based on a stick value
// takes the deadzone into account
void setMotorPWM(int motorEnablePin, int motorDirectionPin, int stickValue) {
  int writeValue;
  if(stickValue < STICK_DEADZONE_BOTTOM) { // below the deadzone - drive backwards
    writeValue = map(stickValue, STICK_DEADZONE_BOTTOM, STICK_MIN, MINPWM, MAXPWM);
    digitalWrite(motorDirectionPin, BACKWARD);
    analogWrite(motorEnablePin, writeValue);
  } else if(stickValue > STICK_DEADZONE_TOP) { // above deadzone - drive forwrads
    writeValue = map(stickValue, STICK_DEADZONE_TOP, STICK_MAX, MINPWM, MAXPWM);
    digitalWrite(motorDirectionPin, FORWARD);
    analogWrite(motorEnablePin, writeValue);
  } else { // inside the deadzone, stop the motor
    writeValue = 0;
    digitalWrite(motorEnablePin, LOW);
  }
  
  // write out the actual value we decided on to the serial port to set gauges
  mySerial.print('m');
  mySerial.print(motorEnablePin);
  mySerial.print(':');
  mySerial.print(map(writeValue, 0, MAXPWM, 0, 1023)); // pwm value goes from 0 to 255, gauge value goes from 0 to 1023
  mySerial.print(',');
  
}
