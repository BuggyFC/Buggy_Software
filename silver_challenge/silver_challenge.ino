#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
// Global variable containing the huskylens control object.
// This gets initialised in setup() so that it communicates over I2C
HUSKYLENS huskylens;
// THis program uses I2C for communications, so
// connect HuskyLens green line to SDA and blue line to SCL (and supply power, obviously)
#include <WiFiS3.h>
WiFiServer server(5200);  // creates server on port 5200
WiFiClient client;        // creates client object for GUI
char ssid[] = "GroupZ2";
char pass[] = "GroupZ2R2Z2";
//High=dark Low=light
const float pi = 3.1415;
const int LEYE = 0;                  //The Pin that reads digital signals from Left eye IR sensor(1)
const int REYE = 13;                 //(2)
bool state_left = 0;                 // true if high, false if low Store state of corrsponding eye sensor(1)
bool state_right = 0;                //(2)
const int left_switch = 9;           //PWM pins to modulate the motor speed (1)
const int right_switch = 11;         //(2)
const int left_motor_logic_1 = 4;    //These pins send logic signals to H-Bridge to determine direction of rotation(1)
const int left_motor_logic_2 = 7;    //(2)
const int right_motor_logic_1 = 5;   //(3)
const int right_motor_logic_2 = 10;  //(4)
const int us_trig = 12;
const int us_echo = 6;
int obj_distance = 0;
long duration = 0;
unsigned long start_time = 0;
unsigned long t_start = 0;
unsigned long elapsed_time = 0;
unsigned long t_elapsed = 0;
bool stop = false;               //true = stop false = go
bool speed = true;               //true = full false = half
bool override = true;            //Override is used to determine void loop will be utilised or not
bool same_object = false;        //Used to determine whether there is a new obstruction or not
const int left_wheel = 2;        //Left wheel encoder pin
const int right_wheel = 3;       //Right wheel encoder pin
volatile int left_count = 0;     //Counts number of rotations of left wheel
volatile int right_count = 0;    //Counts number of rotations of right wheel
float avg_count = 0.0;           //Mean of two rotation counts
float prev_trav_distance = 0.0;  //Used to calculate difference in ditsance travelled
float trav_distance = 0.0;       //Used to find total distance travelled
//PID constants
double kp_obj = 5;
double ki_obj = 0;
double kd_obj = -3;
double kp_spd = -50;
double ki_spd = 0;
double kd_spd = 10;
unsigned long current_time_pid, prev_time_pid;
double elapsed_time_pid;
double error;
double last_error;
double input, output, set_point;
double cum_error, rate_error;
int max_speed = 200;
int min_speed = 150;  // determine minimum driving speed of buggy
int current_speed = 150;
double reference_speed = 10;
double measured_speed = 0.0;  // on board measured speed
bool follow_mode = true;      // choose between buggy following object or buggy travelling at reference speed given from GUI
bool tag1 = false;
bool tag2 = false;
bool tag3 = false;
bool tag4 = false;
String log_string;
String speed_str;
unsigned long prev_timeT3 = 0;
unsigned long current_timeT3 = 0;
unsigned long elapsed_timeT3 = 0;
unsigned long prev_timeT4 = 0;
unsigned long current_timeT4 = 0;
unsigned long elapsed_timeT4 = 0;

void setup() {
  set_point = 15;  // distance for reference object, 15cm
  Serial.begin(9600);
  startWiFi();
  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);
  pinMode(left_switch, OUTPUT);
  pinMode(right_switch, OUTPUT);
  pinMode(left_motor_logic_1, OUTPUT);
  pinMode(left_motor_logic_2, OUTPUT);
  pinMode(right_motor_logic_1, OUTPUT);
  pinMode(right_motor_logic_2, OUTPUT);
  pinMode(us_trig, OUTPUT);
  pinMode(us_echo, INPUT);
  pinMode(left_wheel, INPUT_PULLUP);
  pinMode(right_wheel, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(left_wheel), lUpdate, FALLING);
  attachInterrupt(digitalPinToInterrupt(right_wheel), rUpdate, FALLING);
  start_time = millis();
  t_start = millis();
  fwdDrive();
  Wire.begin();
  // Connect I2C bus and huskylens.
  // Returns false if there is a problem, so I check for that
  // and print errors until things are working.
  while (!huskylens.begin(Wire)) {
    Serial.println(F("Huskylens begin failed!"));
    Serial.println(F("Check Huskylens protocol is set to I2C (General > Settings > Protocol Type > I2C"));
    Serial.println(F("And confirm the physical connection."));
    delay(1000);  // Wait a second before trying to initialise again.
  }
}
void loop() {
  checkClient();
  if (override == false) {  // START BUTTON WAS PRESSED
    unsigned int curr_time = millis();
    elapsed_time = curr_time - start_time;
    if (elapsed_time >= 300) {  // code for polling US sensor every 300ms
      start_time = curr_time;
      checkObject();
      // code for calculating speed every 500ms
      measured_speed = 10 * (trav_distance - prev_trav_distance) / elapsed_time;
      prev_trav_distance = trav_distance;
      if (!tag1 && !tag2) {  // In order to prevent reference modes from activating at same time as speed tags
        if (follow_mode) {   // reference object mode
          if (obj_distance > 10) {
            input = obj_distance;
            output = computePID(input, set_point, kp_obj, ki_obj, kd_obj);
            current_speed -= output;
            if (current_speed > max_speed) // constraining current_speed to max_speed, min_speed
              current_speed = max_speed;
            if (current_speed < min_speed)
              current_speed = min_speed;
          }
        } else {  // reference speed mode
          input = measured_speed;
          output = computePID(input, reference_speed, kp_spd, ki_spd, kd_spd);
          current_speed -= output;
          if (current_speed > max_speed)// constraining current_speed to max_speed, min_speed
            current_speed = max_speed;
          if (current_speed < min_speed)
            current_speed = min_speed;
        }
      }
      if (obj_distance <= 10) {
        if (same_object == false) {
          same_object = true;
        }
        Stop();
        stop = true;
      } else {
        same_object = false;
        driveSpeed();
        stop = false;
      }
    }

    updateState();
    if (stop == false) {
      if (state_left == LOW && tag3 && state_right == LOW) {  // Turn Right tag
        prev_timeT3 = millis();
        fwdRight();
        Serial.println("Turning right cause I was told");
      } else {
        current_timeT3 = millis();
        elapsed_timeT3 = current_timeT3 - prev_timeT3;
        if (elapsed_timeT3 >= 500) {
          prev_timeT3 = current_timeT3;
          tag3 = false;
          tag4 = true;
        }
        if (state_left == LOW) {  //Poll Left sensor to determine to whether to turn or not
          fwdLeft();
        } else
          analogWrite(left_switch, current_speed);

        if (state_right == LOW) {  //Poll Right sensor to determine to whether to turn or not
          fwdRight();
        } else
          analogWrite(right_switch, current_speed);
      }
      if (state_right == LOW && tag4 && state_left == LOW) {  // Turn left Tag
        prev_timeT4 = millis();
        fwdLeft();
        Serial.println("Turning left cause I was told");
      } else {
        current_timeT4 = millis();
        elapsed_timeT4 = current_timeT4 - prev_timeT4;
        if (elapsed_timeT4 >= 500) {
          prev_timeT4 = current_timeT4;
          tag3 = true;
          tag4 = false;
          client.write('k');
        }
        if (state_left == LOW) {  //Poll Left sensor to determine to whether to turn or not
          fwdLeft();
        } else
          analogWrite(left_switch, current_speed);

        if (state_right == LOW) {  //Poll Right sensor to determine to whether to turn or not
          fwdRight();
        } else
          analogWrite(right_switch, current_speed);
      }

      distance();  //calculate distance every loop
    } else {       // STOP BUTTON WAS PRESSED
      measured_speed = 0;
    }
    unsigned int currT = millis();
    t_elapsed = currT - t_start;
    if (t_elapsed >= 1000) {  // code for sending data every .5 second
      t_start = currT;
      sendData();
      Serial.println(measured_speed);
    }
    checkCam();
  }
}
double computePID(double inp, double sPoint, double kp, double ki, double kd) {
  set_point = sPoint;
  current_time_pid = millis();                                    //get current time
  elapsed_time_pid = (double)(current_time_pid - prev_time_pid);  //compute time elapsed from previous computation

  error = set_point - inp;                               // determine error
  cum_error += error * elapsed_time_pid;                 // compute integral
  rate_error = (error - last_error) / elapsed_time_pid;  // compute derivative

  double out = kp * error + ki * cum_error + kd * rate_error;  //PID output

  last_error = error;                //remember current error
  prev_time_pid = current_time_pid;  //remember current time

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
    switch (c) {
      case 'v': // prepare to read in reference speed string
        speed_str = client.readStringUntil('v');
        reference_speed = speed_str.toFloat();
      break;
      case 'w': // start was pressed
        override = false;
        driveSpeed();
        start_time = millis();
      break;
      case 's':  // stop was presssed
        override = true;
        Stop();
      break;
      case 'f':  // "Reference Object" was selected
        follow_mode = true;
        current_speed = 130;
      break;
      case 'r':  // "Reference Speed" was selected
        follow_mode = false;
      break;
      case 'l': // GUI was reset, reset the distance travelled
        trav_distance = 0;
      break;
    }
  }
}
void sendData() {
  String dist_str = String(obj_distance) + "d";  // code for sending object distance to GUI
  client.write('d');
  client.write(dist_str.c_str());
  String speed_str = String(measured_speed) + "v";  // code for sending speed to GUI
  client.write('v');
  client.write(speed_str.c_str());
  String trav_str = String(trav_distance) + "t";  // code for sending distance travelled to GUI
  client.write('t');
  client.write(trav_str.c_str());
  
}
void distance() {  //calculate distance using average count of both wheels
  avg_count = 0.5 * (left_count + right_count);
  trav_distance = (avg_count / 8.0) * pi * 6;  //Iteratively Update distance travelled
}
void fwdDrive() {  //Function to drive forward
  digitalWrite(left_motor_logic_1, HIGH);
  digitalWrite(left_motor_logic_2, LOW);
  digitalWrite(right_motor_logic_1, LOW);
  digitalWrite(right_motor_logic_2, HIGH);
}
void Stop() {  //Function to stop using PWM
  current_speed = 110;
  analogWrite(left_switch, 0);
  analogWrite(right_switch, 0);
}
void fwdLeft() {  //Function to turn left
  analogWrite(left_switch, current_speed * 0);
  analogWrite(right_switch, current_speed * 1.3);
}
void fwdRight() {  //Funtion to turn right
  analogWrite(left_switch, current_speed * 1.3);
  analogWrite(right_switch, current_speed * 0);
}
void fullSpeed() {  //Funtion to make buggy drive @ full speed
  analogWrite(left_switch, 200);
  analogWrite(right_switch, 200);
}
void driveSpeed() {
  analogWrite(left_switch, current_speed);
  analogWrite(right_switch, current_speed);
}
void updateState() {  //Function to update IR sensors
  state_left = digitalRead(LEYE);
  state_right = digitalRead(REYE);
}
void checkObject() {  //Function to poll US sensor
  digitalWrite(us_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(us_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(us_trig, LOW);
  duration = pulseIn(us_echo, HIGH);
  obj_distance = duration / 58;
}
void lUpdate() {
  left_count = left_count + 1;
}
void rUpdate() {
  right_count = right_count + 1;
}
void tOnePro() {
  current_speed -= 5;
  if(current_speed < min_speed)
  current_speed = min_speed;
  driveSpeed();
  tag1 = true;
  tag2 = false;
  tag3 = false;
  tag4 = false;
}
void tTwoPro() {
  current_speed += 5;
  if(current_speed > max_speed)
  current_speed = max_speed;
  driveSpeed();
  tag1 = false;
  tag2 = true;
  tag3 = false;
  tag4 = false;
}
void tThreePro() {
  tag1 = false;
  tag2 = false;
  tag3 = true;
  tag4 = false;
}
void tFourPro() {
  tag1 = false;
  tag2 = false;
  tag2 = false;
  tag4 = true;
}
void checkCam() {
  // First, check that we have the huskylens connected...
  if (!huskylens.request()) Serial.println("Fail to request data from HUSKYLENS, recheck the connection!");
  // then check that it's been trained on something...
  // else if (!huskylens.isLearned()) Serial.println("Nothing learned, press learn button on HUSKYLENS to learn one!");
  // Then check whether there are any blocks visible at this exact moment...
   else if (!huskylens.available()) {
    if(tag1){
      tag1 = false;
      current_speed = min_speed;
    }
    if(tag2){
      tag2 = false;
      current_speed = max_speed;
    }
   }
  else {
    // OK, we have some blocks to process. available() will return the number of blocks to work through.
    // fetch them using read(), one at a time, until there are none left. Each block gets given to
    // my printResult() function to be printed out to the serial port.
    if (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      switch (result.ID) {
        case 1:  // slow speed
        tOnePro();
        client.write('m');
        break;
      case 2:  // max speed
        tTwoPro();
        client.write('n');
        break;
      case 3:  // right turn
        tThreePro();
        client.write('p');
        break;
      case 4:  // left turn
        tFourPro();
        client.write('q');
        break;
        // code block
    }
  }
}
}
