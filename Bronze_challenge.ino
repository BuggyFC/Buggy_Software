/*  COMBINES CODE FROM WED21FEB.INO AND WIFI-COMMUNICATION.INO
    CURRENTLY CRASHES RANDOMLY
****************************************************************************
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
//High = dark Low = light
// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
const float pi = 3.1415;
const int l_eye = 0;//The Pin that reads digital signals from Left eye IR sensor(1)
const int r_eye = 13;//(2)
bool state_left = 0; // true if high, false if low Store state of corrsponding eye sensor(1)
bool state_right = 0;//(2)
const int left_switch = 9;//PWM pins to modulate the motor speed (1)
const int right_switch = 11;//(2)
const int l_motor_logic1 = 4;//These pins send logic signals to H-Bridge to determine direction of rotation(1)
const int l_motor_logic2 = 7;//(2)
const int r_motor_logic1 = 5;//(3)
const int r_motor_logic2 = 10;//(4)
bool track_colour ;//1 = Black 0 = White 
const int us_trig = 12;
const int us_echo = 6;
int obj_dist = 0;
long duration = 0;
unsigned long start_time = 0;
unsigned long elapsed_time = 0;
bool stop = false;//true = stop false = go
bool speed = true;//true = full false = half
bool override = true;//Override is used to determine void loop will be utilised or not
bool same_object = false;//Used to determine whether there is a new obstruction or not
const int l_wheel = 2;//Left wheel encoder pin
const int r_wheel = 3;//Right wheel encoder pin
volatile int l_count = 0;//Counts number of rotations of left wheel
volatile int r_count = 0;//Counts number of rotations of right wheel
float avg_count = 0.0;//Mean of two rotation counts
float prev_trav_dist = 0.0;//Used to calculate difference in ditsance travelled
float tot_trav_dist = 0.0;//Used to find total distance travelled

void setup() {
  Serial.begin(9600);
  startWiFi();
  pinMode(l_eye,INPUT);
  pinMode(r_eye,INPUT); 
  pinMode(left_switch,OUTPUT);
  pinMode(right_switch,OUTPUT);
  pinMode(l_motor_logic1,OUTPUT);
  pinMode(l_motor_logic2,OUTPUT);
  pinMode(r_motor_logic1,OUTPUT);
  pinMode(r_motor_logic2,OUTPUT);
  pinMode(us_trig,OUTPUT);
  pinMode(us_echo,INPUT);
  start_time = millis(); 
  fwdDrive();
  pinMode(l_wheel,INPUT_PULLUP);
  pinMode(r_wheel,INPUT_PULLUP);
  attachInterrupt( digitalPinToInterrupt(l_wheel), lUpdate, FALLING);
  attachInterrupt( digitalPinToInterrupt(r_wheel), rUpdate, FALLING);
}

void lUpdate(){
  l_count = l_count + 1;
}

void rUpdate(){
  r_count = r_count + 1;
}

void loop() {
  checkClient();
  fwdDrive();
  if (override == false){ // START BUTTON WAS PRESSED
    unsigned int currTime=millis(); 
    elapsed_time = currTime - start_time;
    if(elapsed_time >= 500){ // code for polling US sensor every 500ms
        start_time = currTime;
        checkObject();
        if(obj_dist<15){
          if (same_object == false){ 
            client.write('o'); // GUI will display "Object Spotted!"
            same_object = true;
          }
          Stop();
          stop = true;
        }
        else{ 
          same_object = false;
          client.write('z'); // GUI will stop displaying "Object Spotted!"
          speed = true;
          stop = false;
        }
    }
    updateState();
    if(stop == false){
      state_left=digitalRead(l_eye);
      if ( state_left == LOW) //Poll Left sensor to determine to whether to turn or not
        fwdLeft();
      else
        analogWrite(left_switch,200);
      state_right = digitalRead(r_eye);
      if ( state_right == LOW) //Poll Right sensor to determine to whether to turn or not
        fwdRight();
      else
        analogWrite(right_switch,200);
    }
    distance();//calculate distance every loop
    if((tot_trav_dist - prev_trav_dist) >= 10){//Code to update GUI
      prev_trav_dist = tot_trav_dist;
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
        start_time=millis(); 
    }
    if (c == 's'){ // stop was presssed
      override = true;
      Stop();
    }
  }
}

void distance(){//calculate distance using average count of both wheels
  avg_count = 0.5 * (l_count + r_count);
  tot_trav_dist = (avg_count / 8.0) * pi * 6;//Iteratively Update distance travelled
}

void fwdDrive(){//Function to drive forward
  digitalWrite(l_motor_logic1,HIGH);
  digitalWrite(l_motor_logic2,LOW);
  digitalWrite(r_motor_logic1,LOW);
  digitalWrite(r_motor_logic2,HIGH);
}

void reverse(){//Function to reverse
  digitalWrite(l_motor_logic1,LOW);
  digitalWrite(l_motor_logic2,HIGH);
  digitalWrite(r_motor_logic1,HIGH);
  digitalWrite(r_motor_logic2,LOW);
}

void halfSpeed(){//Function to drive at half speed
  analogWrite(left_switch,255.0/2.0);
  analogWrite(right_switch,255.0/2.0);
}
void Stop(){//Function to stop using PWM
  analogWrite(left_switch,0);
  analogWrite(right_switch,0);
}

void fwdLeft(){//Function to turn left
  analogWrite(left_switch,0);
  analogWrite(right_switch,255);
}

void fwdRight(){//Funtion to turn right
  analogWrite(left_switch,255);
  analogWrite(right_switch,0);
}
void fullSpeed(){//Funtion to make buggy drive @ full speed
  analogWrite(left_switch,255);
  analogWrite(right_switch,255);
}

void updateState(){//Function to update IR sensors
  state_left = digitalRead(l_eye);
  state_right = digitalRead(r_eye);
}

void checkObject(){//Function to poll US sensor
  digitalWrite(us_trig,LOW);
  delayMicroseconds(2);
  digitalWrite(us_trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(us_trig,LOW);
  duration=pulseIn(us_echo,HIGH);
  obj_dist=duration/58;
}
