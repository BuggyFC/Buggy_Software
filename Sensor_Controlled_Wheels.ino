//High=dark Low=light
// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
const int LEYE = 2;
const int REYE = 13;
bool state_left = 0; // true if high, false if low
bool state_right = 0;
const int leftSwitch = 3;
const int rightSwitch = 12;
const int LmotorLogic1 = 4;
const int LmotorLogic2 = 7;
const int RmotorLogic1 = 5;
const int RmotorLogic2 = 11;
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
  digitalWrite(leftSwitch,HIGH);
  digitalWrite(rightSwitch,HIGH);
}

void loop() {
  
  //if ( state_left != digitalRead (LEYE)) {
    state_left = digitalRead (LEYE);
    if( digitalRead( LEYE ) == LOW ){
      Serial.println("Gonna Stop Now");
      digitalWrite(LmotorLogic1,LOW);
      digitalWrite(LmotorLogic2,LOW);
      
    }else{
      //Serial.println("light_l");
      digitalWrite(LmotorLogic1,HIGH);
      digitalWrite(LmotorLogic2,LOW);
    }
  //}

  //if (state_right != digitalRead (REYE)){
    state_right = digitalRead(REYE);
    if( digitalRead( REYE ) == LOW ){
      Serial.println("Gonna Stop Now");
      digitalWrite(RmotorLogic1,LOW);
      digitalWrite(RmotorLogic2,LOW);

    }else{
      //Serial.println("high_r");
      digitalWrite(RmotorLogic1,LOW);
      digitalWrite(RmotorLogic2,HIGH);
    }
  //}
  
}
