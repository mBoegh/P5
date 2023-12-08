// Last edited:
String date = "05-12-2023";
// Comment:
String comment = "Stream freq 500Hz";

#include <LiquidCrystal_I2C.h>
#include  <Wire.h>

int row = 0;

int Bauderate = 19200;

LiquidCrystal_I2C lcd(0x27,  20, 3);

// Pins for H-bridge
const int IN1=10;
const int IN2=9;

// Pins for potmeter
const int pot_in = A0;

const int cont_in = A2;
const int cont_5v = 2;

const int edge_led = 6;

bool CW = true;
bool CCW = false;
bool moving = true;     // Tracks the movement of the motor

int upLimit = 120;
int downLimit = 50;

int tPos = 90;
int percent = 0;

float time = 0;
float prevTime = 0;
float pos = 0;
float prevPos = 0;
int read = 0;
int waitTime = 20;

int counter = 0;        // Encoder counter
int currentStateCLK;    // Current state of second output from the encoder
int previousStateCLK;   // Previous state of second output from the encoder
String encdir = "";     // Keeps track of encoder direction (CW or CCW)
String data;            // Holds the latest recived data from the serial

unsigned long start_time = 0;
unsigned long old_time = 0;
unsigned long change_time = 0;

String mode = "velocity";
char sepperator = '\n';

void setup() {
  // Set pinmodes for the rest
  pinMode (pot_in,INPUT);
  pinMode (cont_in,INPUT);
  pinMode (cont_5v,OUTPUT);
  digitalWrite(cont_5v, HIGH);

  pinMode (edge_led,OUTPUT);
  digitalWrite(edge_led, LOW);

  // LCD power pin
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  //initialize lcd screen
  lcd.init();
  // turn on the backlight
  lcd.backlight();

  // Start serial and send a welcome message
  Serial.begin(Bauderate);
  delay(500);

  // Serial.println("Enter velocity -100 to 100 followed by comma");
  lcd.setCursor(7,0);
  lcd.print("EXONET");
  lcd.setCursor(0,1);
  lcd.print("Last updated:");
  lcd.setCursor(0,2);
  lcd.print(date);
  lcd.setCursor(0,3);
  lcd.print(comment);
  start_time = millis();
}

void loop() {
  if(millis()<start_time+2){
    return;
  }
  old_time = start_time;
  start_time = millis();

  if(Serial.available()){                       // If a command is present in the terminal
    data = Serial.readStringUntil(sepperator);         // Read until the sepparating comma
    Serial.read();                            // Empty the remaining serial text
    // lcdData(data);
    if (data[0] == '/'){
      mode = "position";
      data = data.substring(1);
      tPos = data.toInt();                 // Convert the recieved command to an integer
    } else {
      mode = "velocity";
      percent = data.toInt();                 // Convert the recieved command to an integer
      percent = percent - 1000;
      percent = constrain(percent, -100, 100);     // Make sure it is within bounds

      // if (data == "stop"){
      //   brake();
      //   // Serial.println("Braking");
      // } else 
      if (percent < -0){
        if (moving == CW){
          stop();
          // Serial.println("Pausing... ");
          change_time = millis();
          moving = CCW;
        } else if (millis()>change_time+500){
          percent = map(percent, -1, -100, -10, -100);
          // Serial.println("Going CW at " + String(-percent));
          motorSpeed(CCW, -percent);
        }

      } else if (percent > 0){
        if (moving == CCW){
          stop();
          // Serial.println("Pausing... ");
          change_time = millis();
          moving = CW;
        } else if (millis()>change_time+500){
          percent = map(percent, 1, 100, 10, 100);
          // Serial.println("Going CCW at " + String(percent));
          motorSpeed(CW, percent);
        }
      } else {
        // Serial.println("Stopped");
        stop();
      }
    }
  }

  // Loop consists of two parts: first the arduino reads from serial and moves the motor, then it measures the encoder and sends it back.
  if(mode == "velocity"){
    delay(0);
  } else if(mode == "position"){
    int angle = getJointAngle();
    if (angle >= tPos+5){
      if (moving == CW){
          stop();
          // Serial.println("Pausing... ");
          delay(500);
        }
        // Moving down
        motorSpeed(CCW, 10); //angle-tPos+10
        moving = CCW;
    } else if (angle <= tPos-5){
      if (moving == CCW){
          stop();
          // Serial.println("Pausing... ");
          delay(500);
        }
        // Moving up
        motorSpeed(CW, tPos-angle+10);
        moving = CW;
    } else {
      // Serial.println("POSITION STOP");
      stop();
    }
    // Serial.println(String(angle) + "   " + String(tPos) + "   " + String(angle-tPos));0

  } else if(mode == "potmeter"){
    int read = analogRead(cont_in);
    int percent = map(1023-read, 0, 1023, -50, 50);

    if (percent <= -10){
      // Serial.println("Going CCW at " + String(-percent));
      motorSpeed(CCW, -percent);
      moving = CCW;

    } else if (percent >= 10){
      // Serial.println("Going CW at " + String(percent));
      motorSpeed(CW, percent);
      moving = CW;

    } else {
      // Serial.println("Stopped");
      stop();
    }
  }
  checkJointLimits();


  read = analogRead(pot_in);
  // Serial.println(read);

  // prevPos = pos;
  // pos = map(1023-read, 0, 1023, -30, 210);
  // int vel = (pos - prevPos)/((time - prevTime)/1000);
  // Serial.print(String(read) + sepperator);

  

  // Convert the String to a char array (C-style string)
  char utf8Buffer[50];  // Adjust the size according to your needs
  String(read).toCharArray(utf8Buffer, sizeof(utf8Buffer));

  // Now you can use Serial.print() to send the data
  Serial.println(utf8Buffer);


  // Serial.print(String(read) + );

  // delay(waitTime);

  // Serial.println(start_time - old_time);
}

void motorSpeed(bool direction,int speed)
{
  int pwm=map(speed, 0, 100, 0, 255);
  if (direction == CW && getJointAngle() <= upLimit)
  {
   analogWrite(IN1,pwm);
   analogWrite(IN2,LOW);   
  } else if (direction == CCW && getJointAngle() >= downLimit) {
   analogWrite(IN2,pwm);
   analogWrite(IN1,LOW);  
  }
}

int getJointAngle(){
  int angle = map(1023-analogRead(pot_in), 0, 1023, -30, 210);
  // Serial.println(angle); 
  return angle;
}

void checkJointLimits(){
  if (moving == CCW && getJointAngle() <= downLimit)
  {
    digitalWrite(edge_led, HIGH);
    stop();
    // Serial.println("LOWER");
  } else if (moving == CW && getJointAngle() >= upLimit) {
    digitalWrite(edge_led, HIGH);
    stop();
  //  Serial.println("UPPER");
  } else {
    digitalWrite(edge_led, LOW);
  }
}

void brake()
{
  analogWrite(IN1,HIGH);
  analogWrite(IN2,HIGH);   
}

void stop()
{
  analogWrite(IN1,LOW);
  analogWrite(IN2,LOW);   
}

void lcdData(String text){
  lcd.setCursor(0,row);
  lcd.print("            ");
  lcd.setCursor(0,row);
  lcd.print(data);
  row ++;
  if (row>3){
    row = 0;
  }
}

//🤡