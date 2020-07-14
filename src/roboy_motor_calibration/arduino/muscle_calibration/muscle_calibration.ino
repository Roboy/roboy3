#include "HX711.h"
#include <Wire.h>
#include "rgb_lcd.h"
#include "wirelessLove.hpp"

const char ssid[] = "roboy";
const char passwd[]= "wiihackroboy";
IPAddress broadcastIP(192,168,255,255);
WirelessLove *wifi;

const int numRows = 2;
const int numCols = 16;
rgb_lcd lcd;

#define calibration_factor 21000 //This value is obtained using the SparkFun_HX711_Calibration sketch

#define DOUT  5
#define CLK  4

#define BUTTON0 6
#define BUTTON1 7
#define BUTTON2 8
#define BUTTON3 9

int button_pressed =-1;
bool dirty = true;

#define MEASURE 0
#define CALIBRATE 1
#define NETWORK 2
#define PARTY 3
#define STARTMENU 123
#define SCROLLUP 124
#define SCROLLDOWN 125

int state = MEASURE;

int current_entry = 0;
const int menu_entries = 4;

String menu[] = {
  "measure",                  
  "calibrate",            
  "network",          
  "party"
};

HX711 scale(DOUT, CLK);

void buttonPressed() {
  button_pressed = -1;
  if(!digitalRead(BUTTON0))
    button_pressed = 0;
  if(!digitalRead(BUTTON1))
    button_pressed = 1;
  if(!digitalRead(BUTTON2))
    button_pressed = 2;
  if(!digitalRead(BUTTON3))
    button_pressed = 3;
}

void printMenu(){
  if(!dirty)
    return;
  lcd.setRGB(255,255,255);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(menu[current_entry]);
  lcd.setCursor(0,1);
  if(current_entry<menu_entries-1)
    lcd.print(menu[current_entry+1]);
  else
    lcd.print(menu[0]);

  dirty = false;
}

void network(){
  if(!dirty)
    return;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("network");
  lcd.setCursor(0,1);
  lcd.print("WIFI");
  lcd.setCursor(6,1);
  if(wifi->connect(ssid,passwd,broadcastIP)){
    lcd.setRGB(0,255,0);
    lcd.print("CONNECTED");
    lcd.setCursor(0,0);
    lcd.print("UDP");
    lcd.setCursor(6,1);
    if(wifi->initUDPSockets())
      lcd.print("READY");
    else
      lcd.print("ERROR");
    dirty = false;
  }else{
    lcd.setRGB(255,0,0);
    lcd.print("ERROR");
  }
  delay(200);
}

void calibrate(){
  if(!dirty)
    return;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("calibrate");
  dirty = false;
}

void measure(){
  if(dirty){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("measure");
    scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
    scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0
  }
  float val = scale.get_units();
  lcd.setCursor(0,1);
  lcd.print(val);

  uint32_t packed = pack754_32(val);
  wifi->broadcast_send((uint8_t*)&packed, 4);
  dirty = false;
}

void party(){
  if(dirty)
    lcd.clear();
  lcd.setCursor(random(0, 15),0);
  lcd.print("ROBOY");
  lcd.setCursor(random(0, 15),1);
  lcd.print("ROBOY");
  lcd.setRGB(random(0, 255),random(0, 255),random(0, 255));
  delay(5);
  dirty = false;
}

void setup() {
  pinMode(BUTTON0,INPUT_PULLUP);
  pinMode(BUTTON1,INPUT_PULLUP);
  pinMode(BUTTON2,INPUT_PULLUP);
  pinMode(BUTTON3,INPUT_PULLUP);

  attachInterrupt(BUTTON0, buttonPressed, CHANGE);
  attachInterrupt(BUTTON1, buttonPressed, CHANGE);
  attachInterrupt(BUTTON2, buttonPressed, CHANGE);
  attachInterrupt(BUTTON3, buttonPressed, CHANGE);
  
  lcd.begin(numCols, numRows);
  lcd.print("welcome stranger");

  lcd.setCursor(0,1);
  lcd.print("WIFI");

  wifi = new WirelessLove(ssid, passwd, broadcastIP);
  lcd.setCursor(6,1);
  if(wifi->connected && wifi->initUDPSockets()){
    lcd.setRGB(0,255,0);
    lcd.print("connected");
  }else{
    lcd.setRGB(255,0,0);
    lcd.print("ERROR");
  }
  
  delay(1000);
  lcd.setRGB(255,255,255);
}

void loop() {
  if(button_pressed==-1){
    switch(state){
      case STARTMENU:{
          printMenu();
        break;
      }
      case SCROLLUP:{
          if(current_entry==0)
            current_entry = menu_entries-1;
          else
            current_entry--;
          dirty = true;
          state = STARTMENU;
        break;
      }
      case SCROLLDOWN:{
          if(current_entry==menu_entries-1)
            current_entry = 0;
          else
            current_entry++;
          dirty = true;
          state = STARTMENU;
        break;
      }
      case NETWORK:{
        network();
        break;
      }
      case CALIBRATE:{
        calibrate();
        break;
      }
      case MEASURE:{
        measure();
        break;
      }
      case PARTY:{
        party();
        break;
      }
    }
  }else{
    switch(button_pressed){
      case 0:
        state = STARTMENU;
        break;
      case 1:
        state = SCROLLDOWN;
        break;  
      case 2:
        state = SCROLLUP;
        break;  
      case 3:
        state = current_entry;
        break;  
    }
    dirty = true;
  }
}
