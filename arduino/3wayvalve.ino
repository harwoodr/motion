#include <PID_v1.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>

int address = 6;      //i2c slave address (1-6)
unsigned long hbt;    //heartbeat timer
bool debug = false;   //debug mode flag

//servo settings
Servo myservo;
int servoMin;         
int servoMax;

//PID settings
double input;
double output;
double setpoint = 30; //set to ambient pressure reading
//Kp=.25, Ki=0, Kd=0;
double Kp = 0.2;
double Ki = 0.01;
double Kd = 0.01;
int sampleTime;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

int tolerance;

void setup() {
  Wire.begin(address);          // join i2c bus with address
  TWAR = (address << 1) | 1;    // enable i2c broadcasts to be received  
  Wire.onReceive(receiveEvent); // register receive event
  Wire.onRequest(requestEvent); // register request event
  myservo.attach(3);            // initialise servo
  myservo.write(90);            // set to 90 degrees
  pinMode(13, OUTPUT);          // solenoid on/off pin
  //get setup variables from EEPROM
  EEPROM.get(0, Kp);
  EEPROM.get(4, Ki);
  EEPROM.get(8, Kd);
  myPID.SetTunings(Kp, Ki, Kd);
  EEPROM.get(12, sampleTime);
  if(sampleTime == 0){
    sampleTime = 20;
  }
  myPID.SetSampleTime(sampleTime);
  EEPROM.get(14, servoMin);
  if(servoMin == 0){
    servoMin = 45;
  }
  EEPROM.get(16, servoMax);
  if(servoMax == 0){
    servoMax = 135;
  }
  EEPROM.get(18, tolerance);
  if(tolerance == 0){
    tolerance = 5;
  }  
  myPID.SetOutputLimits(servoMin-90,servoMax-90); //set limits for servo motion
  myPID.SetMode(AUTOMATIC);                 //turn the PID on
  hbt = millis();                           //set timer for debug output
  Serial.begin(115200);                     // start serial for output
  Serial.println("Ready!");
  Serial.setTimeout(20);
}

void loop() {
  //read current position
  input = float(map(analogRead(3), 0, 1023, 0, 255));  // ~25-30 is ambient and 255 should be about 100 psi.
  double error = input - setpoint;
  if (abs(int(error)) < tolerance){
    input = setpoint;
  }
  //compute PID output
  myPID.Compute();

  //set servo valve position
  myservo.write(output+90);

  //debug output timer and check
  if (hbt+1000<millis()) {
    hbt=millis();
    if(debug){
      Serial.print("setpoint: ");
      Serial.println(setpoint);
      Serial.print("input: ");
      Serial.println(input);
      Serial.print("output: ");
      Serial.println(output);
      Serial.print("Kp: ");
      Serial.println(Kp);
      Serial.print("Ki: ");
      Serial.println(Ki);
      Serial.print("Kd: ");
      Serial.println(Kd);
      Serial.print("sampleTime: ");
      Serial.println(sampleTime);
      Serial.print("servoMin: ");
      Serial.println(servoMin);
      Serial.print("servoMax: ");
      Serial.println(servoMax);
      Serial.print("tolerance: ");
      Serial.println(tolerance);      
    }
  }
}

//i2c info request from master
void requestEvent() {
  byte response[10];
  response[0] = byte(setpoint);
  response[1] = byte(input);
  response[2] = byte(output);
  response[3] = byte(Kp*100);
  response[4] = byte(Ki*100);
  response[5] = byte(Kd*100);;
  response[6] = byte(sampleTime);
  response[7] = byte(servoMin);
  response[8] = byte(servoMax);
  response[9] = byte(tolerance);
  Wire.write(response,9);  //send current values
  if (debug){
    Serial.print("setpoint: ");
    Serial.println(setpoint);
    Serial.print("input: ");
    Serial.println(input);
    Serial.print("output: ");
    Serial.println(output);
    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);
    Serial.print("sampleTime: ");
    Serial.println(sampleTime);
    Serial.print("servoMin: ");
    Serial.println(servoMin);
    Serial.print("servoMax: ");
    Serial.println(servoMax);
    Serial.print("tolerance: ");
    Serial.println(tolerance);        
  }
}


//commands
//P - set proportional
//I - set integral
//D - set derivative
//T - set target
//S - set sample time
//N - set servoMin
//X - set servoMax
//L - set tolerance
//W - write to eeprom
//R - read from eeprom
//O - open solenoid and set target to 0 
//C - close solenoid and set target to 30
//G - toggle debug

void receiveEvent(int howMany) {
  char command;  //i2c command
  if (howMany == 2) {
    command = Wire.read();  //command byte
    int val = Wire.read();  //data byte
    if (command == 'T') {
      //set target 
      setpoint = float(val);
    } else if (command == 'P') {
      //set Kp
      Kp = float(val)/100;
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (command == 'I') {
      //set Ki
      Ki = float(val)/100;
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (command == 'D') {
      //set Kd
      Kd = float(val)/100;
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (command == 'S') {
      //set sampleTime
      sampleTime = val;
      if(sampleTime == 0){
        sampleTime = 200;
      }
      myPID.SetSampleTime(sampleTime);
    } else if (command == 'N') {
      //set servoMin
      servoMin = val;
      myPID.SetOutputLimits(servoMin-90,servoMax-90);
    } else if (command == 'X') {
      //set servoMax
      servoMax = val;
      myPID.SetOutputLimits(servoMin-90,servoMax-90);
    } else if (command == 'L') {
      //set tolerance
      tolerance = val;
    }
  } else if (howMany == 1) {
    command = Wire.read();
    if (command =='W') {
      //Write Kp, Ki, Kd, sampleTime, servoMin and servoMax to EEPROM
      EEPROM.put(0,Kp);
      EEPROM.put(4,Ki);
      EEPROM.put(8,Kd);
      EEPROM.put(12,sampleTime);  
      EEPROM.put(14, servoMin);
      EEPROM.put(16, servoMax);
      EEPROM.put(18, tolerance);
    } else if (command =='R') {
      //Read Kp, Ki, Kd, sampleTime, servoMin and servoMax from EEPROM
      EEPROM.get(0, Kp);
      EEPROM.get(4, Ki);
      EEPROM.get(8, Kd);
      myPID.SetTunings(Kp, Ki, Kd);
      EEPROM.get(12, sampleTime);
      if(sampleTime == 0){
        sampleTime = 200;
      }
      myPID.SetSampleTime(sampleTime);
      EEPROM.get(14, servoMin);
      EEPROM.get(16, servoMax);
      myPID.SetOutputLimits(servoMin-90,servoMax-90);
      EEPROM.get(18, tolerance);
    } else if (command =='O') {
      //Open solenoid and and set target to 0 -- shutdown
      setpoint = 0;
      digitalWrite(13, LOW);
    } else if (command =='C') {
      //Close solenoid and and set target to 30
      setpoint = 30;
      digitalWrite(13, HIGH);
    } else if (command =='G') {
      //Toggle debug serial output
      if (debug) {
        debug = false;
        Serial.println("debug off");
      } else {
        debug = true;
        Serial.println("debug on");
      }
    }
  }

}
