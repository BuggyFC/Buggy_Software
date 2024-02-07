
// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
const int LEYE = 9;
const int REYE = 10;
bool state_left = 0; // true if high, false if low
bool state_right = 0;
void setup() {
  Serial.begin(9600);
  pinMode (13, OUTPUT) ;
  pinMode( LEYE, INPUT );
  pinMode( REYE, INPUT );
}

void loop() {

  if ( state_left != digitalRead (LEYE)) {
    state_left = digitalRead (LEYE);
    if( digitalRead( LEYE ) == HIGH ){
      Serial.println("low_l  ");
    }else{
      Serial.println("high_l ");
    }
  }

  if (state_right != digitalRead (REYE)){
    state_right = digitalRead(REYE);
    if( digitalRead( REYE ) == HIGH ){
      Serial.println("low_r  ");
    }else{
      Serial.println("high_r");
    }
  }
  delay(1000);
}
