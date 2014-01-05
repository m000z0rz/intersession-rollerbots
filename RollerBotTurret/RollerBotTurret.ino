/*
  (Demonstrates Thumbstick, switch, button, gauge, connections)
  
  This code is an advanced example of a rollerbot with customizations. The rollerbot has a servo pan/tilt mechanism with
    a spring-loaded missile launcher toy mounted on it. By moving the two servos, the rollerbot can aim the missile launcher
    left/right and up/down. The missile launcher holds six missile, and the controller lets you switch fire modes between 
    single-fire (fire one each time you press the button), and automatic mode (keep firing missiles as long as the button is held
    down). There's also a gauge that counts ammo capacity and a button to reset it to full.
    To make all of this controller, the left thumb drives the bot with a thumbstick, and the right thumb can aim the turret with a
    second thumbstick.
    Finally, there's a Connections control that will request a bullet count reset with connecting (we assume the missile
      launcher has been refilled) and makes sure the robot stops moving when we disconnect
    
    Drive thumstick sends X?,Y?, and snaps to middle
    Aiming thumbstick sends U?,V?, and doesn't snap back to middle
    Fire button sends "f" when pressed and "q" when released
    Fire Mode switch sends "s" when swtiched to the left (Single fire mode), and "a" when switched to the right (Automatic fire mode)
    Reset Ammo count button sends "r" when pressed
    Ammo gauge changes value when receiving "B?,"
    Connections control sends "rs" when we first connect and "X512,Y512,U512,V512" when we disconnect
*/


#include <SoftwareSerial.h>
#include <Servo.h>

// Software serial to be used for bluetooth
SoftwareSerial mySerial(P1_4, P1_5);

// Pin mappings
int pin_motor1Direction = P2_0;
int pin_motor2Direction = P2_3;
int pin_motor1Enable = P2_1;
int pin_motor2Enable = P2_2;

int pin_led1 = P1_0;
int pin_led2 = P1_6;

int pin_aim_rotate = P2_4; // pan servo
int pin_aim_elevate = P2_5; // tilt servo

int pin_fire1 = P1_6; // as long as we suply power to the 'fire' pin, the missile launcher toy will keep firing
int pin_fire_sense1 = P1_7; // the missile launcher toy has two pins that connect briefly after firing each missile.
                            // by connecting one pin to ground and monitoring the other pin, we can tell when each
                            // missile fires

// Global Variables
Servo aimRotate;
Servo aimElevate;

byte fireMode;
boolean lastFireSense1;
byte bullets;
int stickX, stickY;

// Constants
const int FORWARD = HIGH;
const int BACKWARD = LOW;

const int MAXPWM = 255;
const int MINPWM = 0.75 * MAXPWM; // below this cutoff, didn't seem like the motors would reliably move the bot

const int STICK_DEADZONE = 0.10 * 1023;
const int STICK_MIN = 0;
const int STICK_MAX = 1023;
const int STICK_MID = (STICK_MIN + STICK_MAX)/2;
// different algorithm is used than with the tank controls. we want our deadzone constants centered on zero:
const int STICK_DEADZONE_BOTTOM = -STICK_DEADZONE/2;
const int STICK_DEADZONE_TOP = STICK_DEADZONE/2;

// angle limits for servos. in theory they coudl go from around 0 to 180 but in practice don't go that far
const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int SERVO_MID = (SERVO_MIN + SERVO_MAX)/2;

const byte FIRE_MODE_SINGLE = 1;
const byte FIRE_MODE_AUTO = 2;

const int MAX_BULLETS = 6;









void setup() {
  // setup pin modes
  pinMode(pin_led1, OUTPUT);
  pinMode(pin_led2, OUTPUT);
  
  pinMode(pin_motor1Direction, OUTPUT);
  pinMode(pin_motor2Direction, OUTPUT);
  pinMode(pin_motor1Enable, OUTPUT);
  pinMode(pin_motor2Enable, OUTPUT);
  
  pinMode(pin_fire1, OUTPUT);
  pinMode(pin_fire_sense1, INPUT_PULLUP); // INPUT_PULLUP means that when the pin isn't connected to anything, it will
                                          //  go high by default
                                          //  then, when the fire sensor connects, it will get connected to ground and pulled low

  // setup our pan/tilt servos
  aimRotate.attach(pin_aim_rotate);
  aimElevate.attach(pin_aim_elevate);
  
  // initialize pin values - motors off, ready to move forward, don't fire missiles
  digitalWrite(pin_motor1Enable, LOW);
  digitalWrite(pin_motor2Enable, LOW);
  digitalWrite(pin_motor1Direction, FORWARD);
  digitalWrite(pin_motor2Direction, FORWARD);
  
  digitalWrite(pin_fire1, LOW); 
  
  
  
  mySerial.begin(9600);
  
  delay(1000);
  
  // start out in single fire mode, and assume the fire sensor was last high
  fireMode = FIRE_MODE_SINGLE;
  lastFireSense1 = 1;
  
  //btSetup(); // you can uncomment this out to setup bt module
}

void btSetup() {
  mySerial.print("AT+NAMERB Turret 0105"); // set bt module name to RB Turret 0105
  while(mySerial.available()) mySerial.read();
  
  flash(pin_led1);
  flash(pin_led2);
  
  delay(1000);
  
  mySerial.print("AT+PIN0105"); // set bt pin to 0105

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
  
  int stickL, stickR;
  
  // get the next character from bluetooth if available
  if(mySerial.available()) {
    btInput = mySerial.read();
  } else {
    btInput = ','; // no data available, just setting it to an unused value
  }
  
  // figure out what to do based on the input
  if(btInput == 'X') { // drive left/right
    stickX = mySerial.parseInt(); // just store the value in a global variable now. the thumbstick will send a Y value
                                  //  next, and once we get it we'll use stickX and stickY to figure out how to drive each motor
  } else if(btInput == 'Y') { // drive fwd/back
    stickY = mySerial.parseInt(); //get the stick Y value
    // stickX should be fresh from right before we got the Y value
    
    // center stickX and stickY on 0
    stickX = stickX - STICK_MID;  // take them from 0 to 1023 centered on 512 to something more like -512 to 511 centered on 0
    stickY = stickY - STICK_MID;
    
    if(stickX > STICK_DEADZONE_BOTTOM && stickX < STICK_DEADZONE_TOP
        && stickY > STICK_DEADZONE_BOTTOM && stickY < STICK_DEADZONE_TOP) {
        // in the deadzone! stop the motors
      digitalWrite(pin_motor1Enable, LOW);
      digitalWrite(pin_motor2Enable, LOW);
    } else {
      // not in the deadzone. let's calculate values for left motor / right motor based on the thumbstick position in stickX and stickY
      // why do to the addition & subtraction work? just try a few values to convince yourself
      //  when stick Y is forward (512) and stickX is in the middle (0), then stickL is 512 and stickR is 512 - both go forward
      // when stick X is left (-512) and stickY is in the middle (0), then stickL is -512 and stickR is 512 - left motor back
      //      and right motor forward, which turns it left
      stickL = stickY + stickX;
      stickR = stickY - stickX;
      setMotorPWM(pin_motor1Enable, pin_motor1Direction, stickL);
      setMotorPWM(pin_motor2Enable, pin_motor2Direction, stickR);
    }
  }
  
  else if(btInput == 'U') { // aim turret left/right
    valueIn = mySerial.parseInt();
    aimRotate.write(map(valueIn, 0, 1023, SERVO_MAX, SERVO_MIN));
  } else if(btInput == 'V') { // aim turret up/down
    valueIn = mySerial.parseInt();
    aimElevate.write(map(valueIn, 0, 1023, SERVO_MAX, SERVO_MIN));
  }
  
  else if(btInput == 'a') { // fire mode: auto
    fireMode = FIRE_MODE_AUTO;
  } else if(btInput == 's') { // fire mode: single
    fireMode = FIRE_MODE_SINGLE;
  }
  
  else if(btInput == 'f') { // fire!
    digitalWrite(pin_fire1, HIGH);
    /*
    if(fireMode == FIRE_MODE_SINGLE) {
      digitalWrite(pin_fire1, HIGH); 
      //while(digitalRead(pin_fire_sense1) == LOW) ; // may have stopped on the sensor touching last time. make sure we get past it
      //while(digitalRead(pin_fire_sense1) == HIGH) ; // hold until the sensor hits?      
    } else if(fireMode == FIRE_MODE_AUTO) {
      digitalWrite(pin_fire1, HIGH);
    }
    */
  } else if(btInput == 'q') { // released the fire button
    if(fireMode == FIRE_MODE_SINGLE) {
      // don't do anything - fireMode is single mode, so we'll stop when the fire sensor tells us one missile has fired
    } else if (fireMode == FIRE_MODE_AUTO) {
      digitalWrite(pin_fire1, LOW); // stop when release button 
    }
  }
  
  else if(btInput == 'r') { // reset ammo count
    bullets = MAX_BULLETS;
    mySerial.print('B'); // set the ammo gauge on the controller
    mySerial.print(map(bullets, 0, MAX_BULLETS, 1, 1023));
    mySerial.print(',');
  }
  
  
  // now it's time to read the fire sensor - we'll use that to stop firing if we're in single fire mode,
  //   and also to update the ammo gauge as needed
  boolean fireSense1 = digitalRead(pin_fire_sense1);
  if(fireSense1 == LOW && lastFireSense1 == HIGH) {
    // fireSensor went from high to low, which means we just fired a bullet
    
    if(fireMode == FIRE_MODE_SINGLE) digitalWrite(pin_fire1, LOW); // stop firing if we're in single fire mode
     
    if(bullets > 0) bullets -= 1; // decrease our bullet count if it's above 0
    mySerial.print('B'); // write out the new bullet count
    mySerial.print(map(bullets, 0, MAX_BULLETS, 1, 1023));
    mySerial.print(',');
  }
  lastFireSense1 = fireSense1; // remember the fire sensor value for comparison with next time
    
  delay(1);
}


// helper function to set left/right motor pwm based on a stick value
  // different from RollerBotTank, this assume stickValue from ~-1024 to ~1024, and deadzone
  //    is already taken care of (stickValue == 0 iff in the deadzone)
void setMotorPWM(int motorEnablePin, int motorDirectionPin, int stickValue) {
  int writeValue;
  if(stickValue < 0) {
    writeValue = map(stickValue, 0, -1024, MINPWM, MAXPWM);
    digitalWrite(motorDirectionPin, BACKWARD);
  } else {
    writeValue = map(stickValue, 0, 1024, MINPWM, MAXPWM);
    digitalWrite(motorDirectionPin, FORWARD);
  }
  analogWrite(motorEnablePin, writeValue);
  
  /*
  mySerial.print('m');
  mySerial.print(motorEnablePin);
  mySerial.print(':');
  mySerial.print(writeValue);
  mySerial.print(",\n");
  */
}
