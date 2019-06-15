#include <Wire.h>
int address;
bool toggle = false;
unsigned long hbt;  //heartbeat timer
int setpoint;
void setup() {
  pinMode(13,OUTPUT);   //heartbeat led
  analogWrite(0,128);
  Wire.begin();         // join i2c bus (address optional for master)
  //Wire.setClock(400000L);
  Serial.begin(115200);   // start serial for output
  hbt = millis(); 
  Serial.println("Ready!");
  Serial.setTimeout(50);
}

void loop() {

  //slave commands
  //<addr> is the single digit address of the slave being transmitted to (1 - 6), with 0 being the broadcast address
  //<byte> is a three digit (0-9) integer (000-255)
  //<addr>P<byte> - set proportional
  //<addr>I<byte> - set integral
  //<addr>D<byte> - set derivative
  //<addr>S<byte> - set sample time
  //<addr>T<byte> - set target
  //<addr>N<byte> - set servoMin
  //<addr>X<byte> - set servoMax
  //<addr>L<byte> - set tolerance
  //<addr>W - write to eeprom
  //<addr>R - read from eeprom
  //<addr>O - open solenoid and set target to 0 
  //<addr>C - close solenoid and set target to 30
  //<addr>G - toggle debug
  //<addr>I - request settings/readings
  //0M<byte><byte><byte><byte><byte><byte>  - set targets for 1-6

  while (Serial.available() > 0) {
    
    int slength = Serial.available();    //number of bytes read into serial buffer
    String sdata;
    byte wdata[9];  //i2c data buffer
    char command;   //command to send
    int address;    //i2c address
    sdata = Serial.readString();  //read the serial buffer
    
    //Serial.print("Received: ");
    //Serial.println(sdata);

    if (isDigit(sdata.charAt(0))) {
      address = sdata.substring(0,1).toInt();
      command = sdata.charAt(1);
      if (command == '?' && address !=0) {
        //send request for info - but not to the broadcast address
        Wire.requestFrom(address,10);
        int i=0;
        while (Wire.available()) {
          wdata[i] = Wire.read();
          i++;
        }
        Serial.print("Info from ");
        Serial.println(address);
        Serial.print("setpoint: ");
        Serial.println(int(wdata[0]));
        Serial.print("input: ");
        Serial.println(int(wdata[1]));
        Serial.print("output: ");
        Serial.println(int(wdata[2])-127);
        Serial.print("Kp*100: ");
        Serial.println(int(wdata[3]));   
        Serial.print("Ki*100: ");
        Serial.println(int(wdata[4]));
        Serial.print("Kd*100: ");
        Serial.println(int(wdata[5]));
        Serial.print("sampleTime: ");
        Serial.println(int(wdata[6]));
        Serial.print("servoMin: ");
        Serial.println(int(wdata[7]));
        Serial.print("servoMax: ");
        Serial.println(int(wdata[8]));
        Serial.print("tolerance: ");
        Serial.println(int(wdata[9]));
      } else if (command == 'P' || command == 'I' || command == 'D' || command == 'T' || command == 'S' || command == 'N' || command == 'X'|| command == 'L') { 
        //send command if P, N or A
        wdata[0] = command;
        wdata[1] = sdata.substring(2,5).toInt();
        Wire.beginTransmission(address);
        Wire.write(wdata,2);
        Wire.endTransmission(true);
        Serial.print("Sent: ");
        Serial.print(command);
        Serial.print(" ");
        Serial.print(int(wdata[1]));
        Serial.print(" to ");
        Serial.println(address);
      } else if (command == 'W' || command == 'R' || command == 'O' || command == 'C' || command == 'G') {
        wdata[0] = command;
        Wire.beginTransmission(address);
        Wire.write(wdata,1);
        Wire.endTransmission(true);
        Serial.print("Sent: ");
        Serial.print(command);
        Serial.print(" to ");
        Serial.println(address);
      } else if (command =='M'){
        wdata[0] = 'T';
        for (int i=0;i<6;i++){
          Wire.beginTransmission(i+1);
          //2,5 5,8 8,11 11,14 14,17 17,20
          wdata[1] = sdata.substring(2+(i*3),5+(i*3)).toInt();
          Wire.write(wdata,2);
          Wire.endTransmission(true);
          Serial.println("M");
        }
      }
    }
  }
  
  if (hbt+1000<millis()) {
    hbt=millis();
    toggle = !toggle;
    digitalWrite(13,toggle);
  }
  

}
