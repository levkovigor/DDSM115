#include <DDSM115.h>

DDSM115 ddsm(&Serial2); 
//DDSM115 ddsm(&Serial2, 2); /*- if max485 has no auto switch of direction, switching gpio can be set */

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
}

void printWheelData(){
    Serial.println("ID: " + String(ddsm.responseData.id));
    Serial.println("Mode: " + String(ddsm.responseData.mode));
    Serial.println("Current: " + String(ddsm.responseData.current));
    Serial.println("Velocity: " + String(ddsm.responseData.velocity));
    Serial.println("Angle: " + String(ddsm.responseData.angle));
    Serial.println("Temp: " + String(ddsm.responseData.winding_temp));
    Serial.println("Error: " + String(ddsm.responseData.error));
}

void loop() {
  if (ddsm.getID()) printWheelData();
  if (ddsm.getMode(1)) printWheelData();

  while(ddsm.responseData.mode != VELOCITY_LOOP){
    ddsm.setMode(1, VELOCITY_LOOP);
    delay(100);
    ddsm.getMode(1);
  }
   
  while (!ddsm.setBrakes(1)) delay(100);
  delay(5000);

  while (!ddsm.setVelocity(1, 200, 0)) delay(100);
  delay(5000);
  while (!ddsm.getMode(1)) delay(100);
  printWheelData();
  
  while (!ddsm.setVelocity(1, -200, 0)) delay(100);
  delay(5000);
  while (!ddsm.getMode(1)) delay(100);
  printWheelData();

  while (!ddsm.setVelocity(1, 0, 0)) delay(100);
  delay(1000);
  while (!ddsm.getMode(1)) delay(100);
  printWheelData();

  while(ddsm.responseData.mode != POSITION_LOOP){
    ddsm.setMode(1, POSITION_LOOP);
    delay(100);
    ddsm.getMode(1);
  }

  while (!ddsm.setPosition(1, 0)) delay(100);
  delay(10000);
  while (!ddsm.setPosition(1, 0)) delay(100);
  printWheelData();

  while (!ddsm.setPosition(1, 90)) delay(100);
  delay(10000);
  while (!ddsm.setPosition(1, 90)) delay(100);
  printWheelData();

  while (!ddsm.setPosition(1, 180)) delay(100);
  delay(10000);
  while (!ddsm.setPosition(1, 180)) delay(100);
  printWheelData();

  while (!ddsm.setPosition(1, 270)) delay(100);
  delay(10000);
  while (!ddsm.setPosition(1, 270)) delay(100);
  printWheelData();

  while (!ddsm.setPosition(1, 360)) delay(100);
  delay(10000);
  while (!ddsm.setPosition(1, 360)) delay(100);
  printWheelData();
}
