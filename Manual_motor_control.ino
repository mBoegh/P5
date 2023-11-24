// Pins for H-bridge
const int IN1=10;
const int IN2=9;

// Pins for potmeter
const int pot_in = A0;

const int cont_in = A2;
const int cont_5v = 2;

bool CW = true;
bool CCW = false;
bool moving = true;     // Tracks the movement of the motor

int upLimit = 100;
int downLimit = 50;

int tPos = 90;

float time = 0;
float prevTime = 0;
float pos = 0;
float prevPos = 0;
int read = 0;
int waitTime = 100;

int counter = 0;        // Encoder counter
int currentStateCLK;    // Current state of second output from the encoder
int previousStateCLK;   // Previous state of second output from the encoder
String encdir = "";     // Keeps track of encoder direction (CW or CCW)
String data;            // Holds the latest recived data from the serial

String mode = "velocity";
char sepperator = ',';

void setup() {
  // Set pinmodes for the rest
  pinMode (pot_in,INPUT);
  pinMode (cont_in,INPUT);
  pinMode (cont_5v,OUTPUT);
  digitalWrite(cont_5v, HIGH);

  // Start serial and send a welcome message
  Serial.begin(9600);
  delay(200);
  Serial.println("Enter velocity -100 to 100 followed by comma");
}

void loop() {
  // Loop consists of two parts: first the arduino reads from serial and moves the motor, then it measures the encoder and sends it back.
  if(mode == "velocity"){
    // ##### Read and move #####
    if(Serial.available()){                       // If a command is present in the terminal
      data = Serial.readStringUntil(sepperator);         // Read until the sepparating comma
      Serial.read();                            // Empty the remaining serial text

      int percent = data.toInt();                 // Convert the recieved command to an integer
      percent = constrain(percent, -100, 100);     // Make sure it is within bounds

      if (data == "stop"){
        brake();
        // Serial.println("Braking");
      } else if (percent <= -10){
        if (moving == CCW){
          stop();
          // Serial.println("Pausing... ");
          delay(500);
        }
        // Serial.println("Going CW at " + String(-percent));
        motorSpeed(CW, -percent);
        moving = CW;

      } else if (percent >= 10){
        if (moving == CW){
          stop();
          // Serial.println("Pausing... ");
          delay(500);
        }
        // Serial.println("Going CCW at " + String(percent));
        motorSpeed(CCW, percent);
        moving = CCW;

      } else {
        // Serial.println("Stopped");
        stop();
      }
    }
    checkJointLimits();

  } else if(mode == "position"){
    if(Serial.available()){                       // If a command is present in the terminal
      data = Serial.readStringUntil(sepperator);         // Read until the sepparating comma
      Serial.read();                            // Empty the remaining serial text

      tPos = data.toInt();                 // Convert the recieved command to an integer
    }
    int angle = getJointAngle();
    if (angle >= tPos+10){
      if (moving == CCW){
          stop();
          // Serial.println("Pausing... ");
          delay(500);
        }
        // Moving down
        motorSpeed(CW, angle-tPos+10);
        moving = CW;
    } else if (angle <= tPos-10){
      if (moving == CW){
          stop();
          // Serial.println("Pausing... ");
          delay(500);
        }
        // Moving up
        motorSpeed(CCW, tPos-angle+10);
        moving = CCW;
    } else {
      stop();
    }

  } else if(mode == "potmeter"){
    int read = analogRead(cont_in);
    int percent = map(1023-read, 0, 1023, -50, 50);

    if (percent <= -10){
      Serial.println("Going CW at " + String(-percent));
      motorSpeed(CW, -percent);
      moving = CW;

    } else if (percent >= 10){
      Serial.println("Going CCW at " + String(percent));
      motorSpeed(CCW, percent);
      moving = CCW;

    } else {
      Serial.println("Stopped");
      stop();
    }
    checkJointLimits();
  }

  prevTime = time;
  time = millis();
  read = analogRead(pot_in);
  // Serial.println(read);

  prevPos = pos;
  pos = map(1023-read, 0, 1023, -30, 210);
  float vel = (pos - prevPos)/((time - prevTime)/1000);
  Serial.print(String(pos) + "," + String(vel) + ",");
  delay(waitTime);
}

void motorSpeed(bool direction,int speed)
{
  int pwm=map(speed, 0, 100, 0, 255);
  if (direction == CW && getJointAngle() >= downLimit)
  {
   analogWrite(IN1,pwm);
   analogWrite(IN2,LOW);   
  } else if (direction == CCW && getJointAngle() <= upLimit) {
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
  if (moving == CW && getJointAngle() <= downLimit)
  {
   stop();
  } else if (moving == CCW && getJointAngle() >= upLimit) {
   stop();
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

//🤡