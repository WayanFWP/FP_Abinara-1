#include <Servo.h>
#include <math.h>
#include <FABRIK2D.h>

Servo base, shoulder, elbow, wrist;
String inputString;

#define PI 3.1416

void setup() {

  Serial.begin(9600);


  base.attach(PA8);  
  shoulder.attach(PA9); 
  elbow.attach(PA10); 
  wrist.attach(PA11); 
  
  resetServos();
}

void loop() {
  while(Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if(inChar == '\n') {
      handle_incoming(inputString);
      Serial.println(inputString);
      inputString = "";
    }
  }
}

void handle_incoming(String inputString) {
  char receivedChars[inputString.length()];
  for(int i=0; i<inputString.length(); i++) {
    receivedChars[i] = inputString[i];
  }

  char * strtokIndx;
  strtokIndx = strtok(receivedChars,",");
  int x = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  int y = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  int z = atoi(strtokIndx);
  strtokIndx = strtok(NULL, '\n');
  int phi = atoi(strtokIndx);
  
  gotocord(x, y, z, phi);
}

void resetServos() {
  base.write(90);
  delay(200);
  shoulder.write(90);
  delay(200);
  elbow.write(90);
  delay(200);
  wrist.write(90);
  delay(200);
}

double deg2rad(int deg) {
  double rad = deg * PI / 180.0;
  return rad;
}

double rad2deg(int rad) {
  double deg = rad * 180.0 / PI;
  return deg;
}

void gotocord(int x, int y, int z, int phi) {
  double theta1, theta2, theta3, theta4;
  double phi_rad = deg2rad(phi);
  double wx = x - a3*cos(phi_rad);
  double wy = y - a3*sin(phi_rad);
  double delta = sq(wx) + sq(wy);
  double c2 = (delta - sq(a1) - sq(a2)) / (2*a1*a2);
  double s2 = sqrt(1 - sq(c2));
  theta3 = atan2(s2,c2);
  double s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta;
  double c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta;
  theta2 = atan2(s1,c1);
  theta1 = atan2(z,x);
  theta1 = rad2deg(theta1);
  theta2 = rad2deg(theta2);
  theta3 = rad2deg(theta3);
  theta4 = phi - theta2 - theta3;
  
  base.write(theta1);
  shoulder.write(theta2);
  elbow.write(theta3);
  wrist.write(theta4);
}
