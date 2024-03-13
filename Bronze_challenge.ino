/****************************************************************************
BRONZE CHALLENGE                                                     COMPLETE
SILVER CHALLENGE
  ENTER SPEED INTO GUI AND HAVE BUGGY TRAVEL AT THAT SPEED           COMPLETE
  REPORT DISTANCE OF OBJECT TO GUI                               NOT COMPLETE
  REPORT CURRENT MODE OF CONTROL TO GUI                              COMPLETE
  REPORT REFERENCE SPEED TO GUI                                      COMPLETE
  REPORT BUGGY SPEED TO GUI                                      NOT COMPLETE
  KEEP CONSTANT 15CM GAP FROM OBJECT USING PID                   NOT COMPLETE
*****************************************************************************/
#include <WiFiS3.h>
WiFiServer server(5200);  // creates server on port 5200
WiFiClient client;        // creates client object for GUI
char ssid[] = "GroupZ2";
char pass[] = "GroupZ2R2Z2";
//High=dark Low=light
// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
const float pi = 3.1415;
const int LEYE = 0;           //The Pin that reads digital signals from Left eye IR sensor(1)
const int REYE = 13;          //(2)
bool state_left = 0;          // true if high, false if low Store state of corrsponding eye sensor(1)
bool state_right = 0;         //(2)
const int leftSwitch = 9;     //PWM pins to modulate the motor speed (1)
const int rightSwitch = 11;   //(2)
const int LmotorLogic1 = 4;   //These pins send logic signals to H-Bridge to determine direction of rotation(1)
const int LmotorLogic2 = 7;   //(2)
const int RmotorLogic1 = 5;   //(3)
const int RmotorLogic2 = 10;  //(4)
bool trackColour;             //1 = Black 0 = White
const int usTrig = 12;
const int usEcho = 6;
int objDistance = 0;
long duration = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
bool stop = false;          //true = stop false = go
bool speed = true;          //true = full false = half
bool override = false;      //Override is used to determine void loop will be utilised or not
bool sameObject = false;    //Used to determine whether there is a new obstruction or not
const int lWheel = 2;       //Left wheel encoder pin
const int rWheel = 3;       //Right wheel encoder pin
volatile int lCount = 0;    //Counts number of rotations of left wheel
volatile int rCount = 0;    //Counts number of rotations of right wheel
float avgCount = 0.0;       //Mean of two rotation counts
float pTravDistance = 0.0;  //Used to calculate difference in ditsance travelled
float travDistance = 0.0;   //Used to find total distance travelled
//PID constants
double kp = 5;
double ki = 0;
double kd = -3;
unsigned long currentTimePID, prevTimePID;
double elapsedTimePID;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
int maxSpeed = 255;
int minSpeed;  // determine minimum driving speed of buggy
int currentSpeed = 200;
int referenceSpeed = 0;
bool followMode = true;  // choose between buggy following object or buggy travelling at reference speed given from GUI

void setup() {
  setPoint = 15;
  Serial.begin(9600);
  startWiFi();
  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);
  pinMode(leftSwitch, OUTPUT);
  pinMode(rightSwitch, OUTPUT);
  pinMode(LmotorLogic1, OUTPUT);
  pinMode(LmotorLogic2, OUTPUT);
  pinMode(RmotorLogic1, OUTPUT);
  pinMode(RmotorLogic2, OUTPUT);
  pinMode(usTrig, OUTPUT);
  pinMode(usEcho, INPUT);
  pinMode(lWheel, INPUT_PULLUP);
  pinMode(rWheel, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(lWheel), lUpdate, FALLING);
  attachInterrupt(digitalPinToInterrupt(rWheel), rUpdate, FALLING);
  startTime = millis();
  fwdDrive();
}

void lUpdate() {
  lCount = lCount + 1;
}

void rUpdate() {
  rCount = rCount + 1;
}

void loop() {
  checkClient();
  if (override == false) {  // START BUTTON WAS PRESSED
    unsigned int currTime = millis();
    elapsedTime = currTime - startTime;
    if (elapsedTime >= 500) {  // code for polling US sensor every 500ms
      startTime = currTime;
      checkObject();
      if (followMode) {
        input = objDistance;
        output = computePID(input);
        currentSpeed -= output;
        if (currentSpeed > 200)
          currentSpeed = 200;
        if (currentSpeed < 130)
          currentSpeed = 130;
        driveSpeed();
        Serial.println(currentSpeed);
      }
      if (objDistance < 10) {
        if (sameObject == false) {
          client.write('o');  // GUI will display "Object Spotted!"
          sameObject = true;
        }
        Stop();
        stop = true;
      } else {
        sameObject = false;
        client.write('z');  // GUI will stop displaying "Object Spotted!"
        // speed = true;
        driveSpeed();
        stop = false;
      }
    }
    updateState();
    if (stop == false) {
      state_left = digitalRead(LEYE);
      if (state_left == LOW) {  //Poll Left sensor to determine to whether to turn or not
        fwdLeft();
      } else
        analogWrite(leftSwitch, currentSpeed);
      state_right = digitalRead(REYE);
      if (state_right == LOW) {  //Poll Right sensor to determine to whether to turn or not
        fwdRight();
      } else
        analogWrite(rightSwitch, currentSpeed);
    }
    distance();                                  //calculate distance every loop
    if ((travDistance - pTravDistance) >= 10) {  //Code to update GUI distance
      pTravDistance = travDistance;
      client.write('+');  //GUI will update distance travelled by .1m
    }
  } else {  // STOP BUTTON WAS PRESSED
  }
}
double computePID(double inp) {
  currentTimePID = millis();                              //get current time
  elapsedTimePID = (double)(currentTimePID - prevTimePID);  //compute time elapsed from previous computation

  error = setPoint - inp;                         // determine error
  cumError += error * elapsedTimePID;                // compute integral
  rateError = (error - lastError) / elapsedTimePID;  // compute derivative

  double out = kp * error + ki * cumError + kd * rateError;  //PID output

  lastError = error;           //remember current error
  prevTimePID = currentTimePID;  //remember current time

  return out;  //have function return the PID output
}
void startWiFi() {  // sets up AP, shows IP and begins server.
  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address:");
  Serial.println(ip);
  server.begin();
}
void checkClient() {  // connects client and reads from client
  client = server.available();
  if (client.available()) {
    char c = client.read();
    Serial.println(c);
    if (c == 'w') {  // start was pressed
      override = false;
      fullSpeed();
      startTime = millis();
    }
    if (c == 's') {  // stop was presssed
      override = true;
      Stop();
    }
    if (c == 'f')  // "Follow Mode" was selected
      followMode = true;
    if (c == 'r') {  // "Use Reference Speed" was selected
      followMode = false;
      currentSpeed = referenceSpeed;
    }
    if (c == 'i')
      referenceSpeed += 25;
    if (c == 'd')
      referenceSpeed -= 25;
  }
}
void distance() {  //calculate distance using average count of both wheels
  avgCount = 0.5 * (lCount + rCount);
  travDistance = (avgCount / 8.0) * pi * 6;  //Iteratively Update distance travelled
}
void fwdDrive() {  //Function to drive forward
  digitalWrite(LmotorLogic1, HIGH);
  digitalWrite(LmotorLogic2, LOW);
  digitalWrite(RmotorLogic1, LOW);
  digitalWrite(RmotorLogic2, HIGH);
}
void reverse() {  //Function to reverse
  digitalWrite(LmotorLogic1, LOW);
  digitalWrite(LmotorLogic2, HIGH);
  digitalWrite(RmotorLogic1, HIGH);
  digitalWrite(RmotorLogic2, LOW);
}
void halfSpeed() {  //Function to drive at half speed
  analogWrite(leftSwitch, 255.0 / 2.0);
  analogWrite(rightSwitch, 255.0 / 2.0);
}
void Stop() {  //Function to stop using PWM
  analogWrite(leftSwitch, 0);
  analogWrite(rightSwitch, 0);
}

void fwdLeft() {  //Function to turn left
  analogWrite(leftSwitch, 0);
  analogWrite(rightSwitch, currentSpeed+25);
}

void fwdRight() {  //Funtion to turn right
  analogWrite(leftSwitch, currentSpeed+25);
  analogWrite(rightSwitch, 0);
}
void fullSpeed() {  //Funtion to make buggy drive @ full speed
  analogWrite(leftSwitch, 255);
  analogWrite(rightSwitch, 255);
}
void driveSpeed() {
  analogWrite(leftSwitch, currentSpeed);
  analogWrite(rightSwitch, currentSpeed);
}
void updateState() {  //Function to update IR sensors
  state_left = digitalRead(LEYE);
  state_right = digitalRead(REYE);
}

void checkObject() {  //Function to poll US sensor
  digitalWrite(usTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(usTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(usTrig, LOW);
  duration = pulseIn(usEcho, HIGH);
  objDistance = duration / 58;
}
