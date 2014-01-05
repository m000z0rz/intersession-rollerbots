/*
  (Demonstrates buttons, gauges)
  
  This code is designed for a rollerbot with basic 'wasd' controls - you use the w, a, s, and d
    keys on the keyboard like an arrow pad to make it go forward, left, backwards, or right (respectively)
  The controller consists of four buttons with the necessary keyboard shortcuts
  In addition to the buttons, the controller has an indicator. The robot will cycle the color of the
      indicator through a given sequence
      
   Forward button (w) sends "fg" when pressed and "s" when released
   Backward button (s) sends "bg" when pressed and "s" when released
   Left button (a) sends "lg" when pressed and "s" when released
   Right button (d) sends "rg" when pressed and "s" when released
   Indicator turns red when "red" is received, orange when "orange" is received, etc
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

// Global variables - keep track of what color we sent last, and when we should send the next one
unsigned long nextColorTime;
unsigned long time;
int currentColor = 0;

char* colorNames[] = {
 "red",
 "orange",
 "yellow",
 "green",
 "blue",
 "purple",
 "white",
 "gray",
 "black"
};

// Constants

const int FORWARD = HIGH;
const int BACKWARD = LOW;







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
  
  nextColorTime = millis() + 2000; // send the next color 2000 milliseconds from now (2 seconds from now)
}


void btSetup() {
mySerial.print("AT+NAMERB Basic 0103"); // set bt module name to RB Basic 0103
  while(mySerial.available()) mySerial.read(); //Serial.write(mySerial.read());
  
  flash(pin_led1);
  flash(pin_led2);
  
  delay(1000);
  
  mySerial.print("AT+PIN0103"); // set bt pin to 0103
  
  while(mySerial.available()) mySerial.read(); //Serial.write(mySerial.read());
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
 
  // get the next character from bluetooth if available
  if(mySerial.available()) {
    btInput = mySerial.read();
  } else {
    btInput = ','; // no data available, just setting it to an unused value
  }

  // figure out what to do based on the input
  if(btInput == 'g') { // enable motors
    mySerial.println("enb"); // writing for debug info to terminal
    digitalWrite(pin_motor1Enable, HIGH);
    digitalWrite(pin_motor2Enable, HIGH);
  } else if(btInput == 's') { // stop motors
    mySerial.println("stp");
    digitalWrite(pin_motor1Enable, LOW); 
    digitalWrite(pin_motor2Enable, LOW);
  } else if(btInput == 'f') { // move robot forward - both motors forward
    mySerial.println("fwd");
    digitalWrite(pin_motor1Direction, FORWARD); 
    digitalWrite(pin_motor2Direction, FORWARD);
  } else if(btInput == 'b') { // move robot backward - both motor backward
    mySerial.println("bck");
    digitalWrite(pin_motor1Direction, BACKWARD); 
    digitalWrite(pin_motor2Direction, BACKWARD);
  } else if(btInput == 'l') { // turn robot left - left motor back, right motor fwd
    mySerial.println("lft");
    digitalWrite(pin_motor1Direction, BACKWARD); 
    digitalWrite(pin_motor2Direction, FORWARD);
  } else if(btInput == 'r') { // turn robot right - left motor fwd, right motor back
    mySerial.println("rgt");
    digitalWrite(pin_motor1Direction, FORWARD); 
    digitalWrite(pin_motor2Direction, BACKWARD);
  } 
  // below commands can be uncommented to add commands to set motor power to half or 3/4
  /*else if (btInput == 'h') {
    mySerial.println("hlf");
    analogWrite(pin_motor1Enable, 128);
    analogWrite(pin_motor2Enable, 128);
  } else if (btInput == 't') {
    mySerial.println("thr");
    analogWrite(pin_motor1Enable, 192); 
    analogWrite(pin_motor2Enable, 192);
  }*/
  
  
  // see if it's time to send a new color for the indicator
  time = millis(); // get the current time
  if(time > nextColorTime) { // have we passed the point where we're supposed to send a new color?
    mySerial.println(colorNames[currentColor]); // send the color we're currently on
    currentColor = currentColor + 1; // point to the next color
    if(currentColor > 8) currentColor = 0; // if we moved passed the end of our list of colors, wrap around to the beginning
    nextColorTime = time + 2000; // and get ready to send the next color 2 seconds from now!
  }
  
  delay(1);
}
