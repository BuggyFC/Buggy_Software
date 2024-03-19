/****************************************************************************
BRONZE CHALLENGE                                                     COMPLETE
SILVER CHALLENGE
  ENTER SPEED IN M/S INTO GUI AND HAVE BUGGY GO AT THAT SPEED        COMPLETE
  REPORT DISTANCE OF OBJECT TO GUI                                   COMPLETE
  REPORT CURRENT MODE OF CONTROL TO GUI                              COMPLETE
  REPORT REFERENCE SPEED TO GUI IN M/S                               COMPLETE
  REPORT BUGGY SPEED TO GUI                                          COMPLETE
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
bool trackColour;             // 1 = Black 0 = White
const int usTrig = 12;
const int usEcho = 6;
int objDistance = 0;
long duration = 0;
unsigned long startTime = 0;
unsigned long startT = 0;
unsigned long elapsedTime = 0;
unsigned long elapsedT = 0;
bool stop = false;          //true = stop false = go
bool speed = true;          //true = full false = half
bool override = true;       //Override is used to determine void loop will be utilised or not
bool sameObject = false;    //Used to determine whether there is a new obstruction or not
const int lWheel = 2;       //Left wheel encoder pin
const int rWheel = 3;       //Right wheel encoder pin
volatile int lCount = 0;    //Counts number of rotations of left wheel
volatile int rCount = 0;    //Counts number of rotations of right wheel
float avgCount = 0.0;       //Mean of two rotation counts
float pTravDistance = 0.0;  //Used to calculate difference in ditsance travelled
float travDistance = 0.0;   //Used to find total distance travelled
//PID constants
double kpObj = 5;
double kiObj = 0;
double kdObj = -3;
double kpSpd = -50;
double kiSpd = 0;
double kdSpd = 0;
unsigned long currentTimePID, prevTimePID;
double elapsedTimePID;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
int maxSpeed = 120;
int minSpeed = 0;  // determine minimum driving speed of buggy
int currentSpeed = 100;
double referenceSpeed = 0;
double measuredSpeed = 0.0;  // on board measured speed
bool followMode = false;     // choose between buggy following object or buggy travelling at reference speed given from GUI
float travDistSpd = 0.0;     // used to calculate speed
float prevTravDistSpd = 0.0;

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
  startT = millis();
  fwdDrive();
}
void loop() {
  checkClient();
  if (override == false) {  // START BUTTON WAS PRESSED
    unsigned int currTime = millis();
    elapsedTime = currTime - startTime;
    if (elapsedTime >= 500) {  // code for polling US sensor every 500ms
      startTime = currTime;
      checkObject();
      // code for calculating speed every 500ms
      measuredSpeed = 10 * (travDistSpd - prevTravDistSpd) / elapsedTime;
      prevTravDistSpd = travDistSpd;
      if (followMode) {  // reference object mode
        if (objDistance > 10) {
          input = objDistance;
          output = computePID(input, setPoint, kpObj, kiObj, kdObj);
          currentSpeed -= output;
          if (currentSpeed > maxSpeed)
            currentSpeed = maxSpeed;
          if (currentSpeed < minSpeed)
            currentSpeed = minSpeed;
        }
      } else {  // reference speed mode
        input = measuredSpeed;
        output = computePID(input, referenceSpeed, kpSpd, kiSpd, kdSpd);
        currentSpeed -= output;
        if (currentSpeed > maxSpeed)
          currentSpeed = maxSpeed;
        if (currentSpeed < 0)
          currentSpeed = 0;
      }
      driveSpeed();
      if (objDistance <= 10) {
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
    distance();  //calculate distance every loop
    unsigned int currT = millis();
    elapsedT = currT - startT;
    if (elapsedT >= 1000) {  // code for sending data every 1 second
      startT = currT;
      sendData();
    }
  } else {  // STOP BUTTON WAS PRESSED
    measuredSpeed = 0;
  }
//
  // Serial.print("measured speed: ");
  // Serial.println(measuredSpeed);
  // Serial.print("reference speed: ");
  // Serial.println(referenceSpeed);
  // Serial.print ("travDistSpd : " );
  // Serial.println(travDistSpd);
  // Serial.print("prevDistSpd: ");
  // Serial.println(prevTravDistSpd);
  // Serial.println(currentSpeed);
}
double computePID(double inp, double sPoint, double kp, double ki, double kd) {
  setPoint = sPoint;
  currentTimePID = millis();                                //get current time
  elapsedTimePID = (double)(currentTimePID - prevTimePID);  //compute time elapsed from previous computation

  error = setPoint - inp;                            // determine error
  cumError += error * elapsedTimePID;                // compute integral
  rateError = (error - lastError) / elapsedTimePID;  // compute derivative

  double out = kp * error + ki * cumError + kd * rateError;  //PID output

  lastError = error;             //remember current error
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
    if (c == 'v') {
      String speedStr = client.readStringUntil('v');
      referenceSpeed = speedStr.toFloat();
      // Serial.println("speedStr: " + speedStr);
      // Serial.println("refSpeed: ");
      // Serial.print(referenceSpeed);
    }
    if (c == '1') {  // start was pressed
      override = false;
      driveSpeed();
      startTime = millis();
    }
    if (c == '0') {  // stop was presssed
      override = true;
      Stop();
    }
    if (c == 'f') {  // "Reference Object" was selected
      followMode = true;
      currentSpeed = 200;
    }
    if (c == 'r') {  // "Reference Speed" was selected
      followMode = false;
    }
  }
}
void sendData() {
  String distStr = String(objDistance) + "d";  // code for sending object distance to GUI
  client.write('d');
  client.write(distStr.c_str());
  String speedStr = String(measuredSpeed) + "v";  // code for sending speed to GUI
  client.write('v');
  client.write(speedStr.c_str());
  String travStr = String(travDistance) + "t";  // code for sending distance travelled to GUI
  client.write('t');
  client.write(travStr.c_str());
}
void distance() {  //calculate distance using average count of both wheels
  avgCount = 0.5 * (lCount + rCount);
  travDistance = (avgCount / 8.0) * pi * 6;  //Iteratively Update distance travelled
  travDistSpd = travDistance;
}
void fwdDrive() {  //Function to drive forward
  digitalWrite(LmotorLogic1, HIGH);
  digitalWrite(LmotorLogic2, LOW);
  digitalWrite(RmotorLogic1, LOW);
  digitalWrite(RmotorLogic2, HIGH);
}
void Stop() {  //Function to stop using PWM
  analogWrite(leftSwitch, 0);
  analogWrite(rightSwitch, 0);
}
void fwdLeft() {  //Function to turn left
  analogWrite(leftSwitch, currentSpeed * 0.3);
  analogWrite(rightSwitch, currentSpeed * 1.3);
}
void fwdRight() {  //Funtion to turn right
  analogWrite(leftSwitch, currentSpeed * 1.3);
  analogWrite(rightSwitch, currentSpeed * 0.3);
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
void lUpdate() {
  lCount = lCount + 1;
}
void rUpdate() {
  rCount = rCount + 1;
}