
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

const uint32_t R[] ={
  	0x1f810810,
		0x81101e01,
		0x20110108,
		66
};
const uint32_t two[]={
		0x3fc09009,
		0x900900,
		0x900903fc,
		66  
};
const uint32_t zed[]={
    0x3fc00801,
		0x200400,
		0x801003fc,
		66
};
void setup() {
  Serial.begin(9600);
  matrix.begin();
}
void loop() {
  // put your main code here, to run repeatedly:
matrix.loadFrame(R);
delay (1000);
matrix.loadFrame(two);
delay (1000);
matrix.loadFrame(zed);
delay (1000);
matrix.loadFrame(two);
delay (1000); 
}



