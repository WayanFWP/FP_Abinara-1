#include <Servo.h>

Servo base, shoulder, elbow, wrist;

void moveBase(int angle){
  base.write(angle);  
}

void moveShoulder(int angle){
  shoulder.write(angle);
}

void moveElbow(int angle){
  elbow.write(angle);
}

void moveWrist(int angle){
  wrist.write(angle);
}

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);

  // Attach servos to respective pins
  base.attach(PIN_BASE);
  shoulder.attach(PIN_SHOULDER);
  elbow.attach(PIN_ELBOW);
  wrist.attach(PIN_WRIST);
}

void loop() {

  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    int x, y, z, phi;
    sscanf(inputString.c_str(), "%d,%d,%d,%d", &x, &y, &z, &phi);
    gotocord(x, y, z, phi);
  }
}

void gotocord(int x, int y, int z, int phi) {:
  moveBase(x);
  moveShoulder(y);
  moveElbow(z);
  moveWrist(phi);
}
