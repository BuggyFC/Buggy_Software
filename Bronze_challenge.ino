/****************************************************************************
BRONZE CHALLENGE
  TRAVERSE TRACK TWICE USING IR SENSORS TO FOLLOW LINE :         C0MPLETE
  PAUSE FOR OBSTACLES USING US SENSOR :                          COMPLETE
  START RUN ON RECEIVING GO COMMAND VIA WIFI :                   COMPLETE
  STOP RUN ON RECEIVING STOP COMMAND VIA WIFI :                  COMPLETE
  REPORT TO PC WHEN OBSTACLES ARE DETECTED :                     COMPLETE
  DISTANCE TRAVELLED USING WHEEL ENCODER :                       COMPLETE
*****************************************************************************/
#include <WiFiS3.h>
WiFiServer server(5200); // creates server on port 5200
WiFiClient client;      // creates client object for GUI 
char ssid[] = "GroupZ2";
char pass[] = "GroupZ2R2Z2";
//High=dark Low=light
// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
const float pi = 3.1415;
const int LEYE = 0; //The Pin that reads digital signals from Left eye IR sensor(1)
const int REYE = 13; //(2)
bool state_left = 0; // true if high, false if low Store state of corrsponding eye sensor(1)
bool state_right = 0; //(2)
const int leftSwitch = 9; //PWM pins to modulate the motor speed (1)
const int rightSwitch = 11; //(2)
const int LmotorLogic1 = 4; //These pins send logic signals to H-Bridge to determine direction of rotation(1)
const int LmotorLogic2 = 7; //(2)
const int RmotorLogic1 = 5; //(3)
const int RmotorLogic2 = 10; //(4)
bool trackColour ; //1 = Black 0 = White 
const int usTrig = 12;
const int usEcho = 6;
int objDistance = 0;
long duration = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
bool stop = false; //true = stop false = go 
bool speed = true; //true = full false = half
bool override = true; //Override is used to determine void loop will be utilised or not
bool sameObject = false; //Used to determine whether there is a new obstruction or not
const int lWheel = 2; //Left wheel encoder pin
const int rWheel = 3; //Right wheel encoder pin
volatile int lCount = 0; //Counts number of rotations of left wheel
volatile int rCount = 0; //Counts number of rotations of right wheel
float avgCount = 0.0; //Mean of two rotation counts
float pTravDistance = 0.0; //Used to calculate difference in ditsance travelled
float travDistance = 0.0; //Used to find total distance travelled

void setup() {
  Serial.begin(9600);
  startWiFi();
  pinMode(LEYE,INPUT);
  pinMode(REYE,INPUT); 
  pinMode(leftSwitch,OUTPUT);
  pinMode(rightSwitch,OUTPUT);
  pinMode(LmotorLogic1,OUTPUT);
  pinMode(LmotorLogic2,OUTPUT);
  pinMode(RmotorLogic1,OUTPUT);
  pinMode(RmotorLogic2,OUTPUT);
  pinMode(usTrig,OUTPUT);
  pinMode(usEcho,INPUT);
  pinMode(lWheel,INPUT_PULLUP);
  pinMode(rWheel,INPUT_PULLUP);
  attachInterrupt( digitalPinToInterrupt(lWheel), lUpdate, FALLING);
  attachInterrupt( digitalPinToInterrupt(rWheel), rUpdate, FALLING);
  startTime = millis(); 
  fwdDrive();
}

void lUpdate(){
  lCount = lCount + 1;
}

void rUpdate(){
  rCount = rCount + 1;
}

void loop() {
  checkClient();
  fwdDrive();
  if (override == false){ // START BUTTON WAS PRESSED
    unsigned int currTime=millis(); 
    elapsedTime = currTime - startTime;
    if(elapsedTime >= 500){ // code for polling US sensor every 500ms
        startTime = currTime;
        checkObject();
        if(objDistance<15){
          if (sameObject == false){ 
            client.write('o'); // GUI will display "Object Spotted!"
            sameObject = true;
          }
          Stop();
          stop = true;
        }
        else{ 
          sameObject = false;
          client.write('z'); // GUI will stop displaying "Object Spotted!"
          speed = true;
          stop = false;
        }
    }
    updateState();
    if(stop == false){
      state_left=digitalRead(LEYE);
      if ( state_left == LOW){ //Poll Left sensor to determine to whether to turn or not
        fwdLeft();
        client.write('l');
      }
      else
        analogWrite(leftSwitch,200);
      state_right = digitalRead(REYE);
      if ( state_right == LOW){ //Poll Right sensor to determine to whether to turn or not
        fwdRight();
        client.write('r');
      }
      else
        analogWrite(rightSwitch,200);
    }
    distance();//calculate distance every loop
    if((travDistance - pTravDistance) >= 10){ //Code to update GUI distance
      pTravDistance = travDistance;
      client.write('+');//GUI will update distance travelled by .1m
    }
  }
  else{ // STOP BUTTON WAS PRESSED
  }
}

void startWiFi(){ // sets up AP, shows IP and begins server.
  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address:");
  Serial.println(ip);
  server.begin();
}
void checkClient(){ // connects client and reads from client
  client = server.available();
  if (client.available()){
    char c = client.read(); 
    Serial.println(c);
    if (c == 'w'){ // start was pressed
        override = false;
        fwdDrive();
        fullSpeed();
        startTime=millis(); 
    }
    if (c == 's'){ // stop was presssed
      override = true;
      Stop();
    }
  }
}

void distance(){ //calculate distance using average count of both wheels
  avgCount = 0.5 * (lCount + rCount);
  travDistance = (avgCount / 8.0) * pi * 6;//Iteratively Update distance travelled
}
  
void fwdDrive(){ //Function to drive forward
  digitalWrite(LmotorLogic1,HIGH);
  digitalWrite(LmotorLogic2,LOW);
  digitalWrite(RmotorLogic1,LOW);
  digitalWrite(RmotorLogic2,HIGH);
}

void reverse(){ //Function to reverse
  digitalWrite(LmotorLogic1,LOW);
  digitalWrite(LmotorLogic2,HIGH);
  digitalWrite(RmotorLogic1,HIGH);
  digitalWrite(RmotorLogic2,LOW);
}

void halfSpeed(){ //Function to drive at half speed
  analogWrite(leftSwitch,255.0/2.0);
  analogWrite(rightSwitch,255.0/2.0);
}
void Stop(){ //Function to stop using PWM
  analogWrite(leftSwitch,0);
  analogWrite(rightSwitch,0);
}

void fwdLeft(){ //Function to turn left
  analogWrite(leftSwitch,0);
  analogWrite(rightSwitch,255);
}

void fwdRight(){ //Funtion to turn right
  analogWrite(leftSwitch,255);
  analogWrite(rightSwitch,0);
}
void fullSpeed(){ //Funtion to make buggy drive @ full speed
  analogWrite(leftSwitch,255);
  analogWrite(rightSwitch,255);
}

void updateState(){ //Function to update IR sensors
  state_left = digitalRead(LEYE);
  state_right = digitalRead(REYE);
}

void checkObject(){ //Function to poll US sensor
  digitalWrite(usTrig,LOW);
  delayMicroseconds(2);
  digitalWrite(usTrig,HIGH);
  delayMicroseconds(10);
  digitalWrite(usTrig,LOW);
  duration=pulseIn(usEcho,HIGH);
  objDistance=duration/58;
}
