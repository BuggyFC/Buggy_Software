/***************************************************************************
BRONZE CHALLENGE:
  TRAVERSE TRACK TWICE USING IR SENSORS TO FOLLOW LINE:         C0MPLETE
  PAUSE FOR OBSTACLES USING US SENSOR:                          COMPLETE
  START RUN ON RECEIVING GO COMMAND VIA WIFI:                   NOT COMPLETE
  STOP RUN ON RECEIVING STOP COMMAND VIA WIFI:                  NOT COMPLETE
  REPORT TO PC WHEN OBSTACLES ARE DETECTED:                     NOT COMPLETE
  (MAYBE OPTIONAL?) DISTANCE TRAVELLED USING WHEEL ENCODER:     NOT COMPLETE
*****************************************************************************/

//High=dark Low=light
// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
const int LEYE = 1;//The Pin that reads digital signals from Left eye IR sensor(1)
const int REYE = 13;//(2)
bool state_left = 0; // true if high, false if low Store state of corrsponding eye sensor(1)
bool state_right = 0;//(2)
const int leftSwitch = 3;//PWM pins to modulate the motor speed (1)
const int rightSwitch = 11;//(2)
const int LmotorLogic1 = 4;//These pins send logic signals to H-Bridge to determine direction of rotation(1)
const int LmotorLogic2 = 7;//(2)
const int RmotorLogic1 = 5;//(3)
const int RmotorLogic2 = 10;//(4)
bool trackColour ;//1=Black 0=White 
const int usTrig = 12;
const int usEcho = 6;
int distance;
long duration;
unsigned long startTime;
unsigned long elapsedTime;
bool stop = false;//true = stop false = go
bool speed = true;//true = full false = half

void setup() {
  Serial.begin(9600);
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
  //trackColour=digitalRead(LEYE); 
  startTime=millis(); 
  fwdDrive();
}

void loop() {
  fwdDrive();
  unsigned int currTime=millis();
  elapsedTime = currTime - startTime;
  if(elapsedTime >= 500){
      startTime = currTime;
      checkObject();
      if(distance <= 25 && distance >= 15){
        halfSpeed();
        fwdDrive();
        speed = false;
        stop = false;
      }
      else if(distance<15||distance>2000){
        Stop();
        stop = true;
      }
      else{ 
        speed = true;
        stop = false;
      }
  }
  updateState();
  //Serial.println(state_left);
  if(stop == false){
    state_left=digitalRead(LEYE);
    if ( state_left == LOW) 
      fwdLeft();
    else
      analogWrite(leftSwitch,200);
    state_right = digitalRead(REYE);
    if ( state_right == LOW)
      fwdRight();
    else
      analogWrite(rightSwitch,200);
  }
}

void fwdDrive(){
  digitalWrite(LmotorLogic1,HIGH);
  digitalWrite(LmotorLogic2,LOW);
  digitalWrite(RmotorLogic1,LOW);
  digitalWrite(RmotorLogic2,HIGH);
}

void Reverse(){
  digitalWrite(LmotorLogic1,LOW);
  digitalWrite(LmotorLogic2,HIGH);
  digitalWrite(RmotorLogic1,HIGH);
  digitalWrite(RmotorLogic2,LOW);
}

void halfSpeed(){
  analogWrite(leftSwitch,255.0/2.0);
  analogWrite(rightSwitch,255.0/2.0);
}
void Stop(){
  analogWrite(leftSwitch,0);
  analogWrite(rightSwitch,0);
}

void fwdLeft(){
  analogWrite(leftSwitch,0);
  analogWrite(rightSwitch,255);
}

void fwdRight(){
  analogWrite(leftSwitch,255);
  analogWrite(rightSwitch,0);
}

void updateState(){
  state_left = digitalRead(LEYE);
  state_right = digitalRead(REYE);
}

void checkObject(){
  digitalWrite(usTrig,LOW);
  delayMicroseconds(2);
  digitalWrite(usTrig,HIGH);
  delayMicroseconds(10);
  digitalWrite(usTrig,LOW);
  duration=pulseIn(usEcho,HIGH);
  distance=duration/58;
}