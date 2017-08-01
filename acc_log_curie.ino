#include "CurieTimerOne.h"
#include "CurieIMU.h"
#include "CurieBLE.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "Adafruit_TMP007.h"
#include <Adafruit_ADXL345_U.h>

#define SEC 1000000
#define FREQ 50
#define S_RATE  SEC/FREQ
#define BUFF_SIZE 50
#define PULSE_PIN 2



//#define DEBUG 



typedef struct sensorReadings_s
    {
      int ax, ay, az, gx, gy, gz, ahx, ahy, ahz, ps;
      unsigned long ts;
      byte as;
      
    } sensorReadings;
    
sensorReadings buf[BUFF_SIZE];


BLEPeripheral blePeripheral; // create peripheral instance
BLECentral bleCentral = blePeripheral.central();
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service
BLEIntCharacteristic switchChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic stateChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic secLogged("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic temp("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

Adafruit_TMP007 tmp007;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);// Adafruit_ADXL345_Unified(13,12,11,10,12345);

float objt, senst;

unsigned long mls;

volatile int readPtr, int_cnt, sec ;

volatile boolean  doMeasure , flag, listfile, stopmeasure, startmeasure, listtemp, rmfile;

byte activity_state,seccount;

const int IMULow = -32768;
const int IMUHigh = 32767;
const int chipSelect = 7;  //CS PIN for SD card

File dataFile;


void  timerInterrupt(){
    int_cnt++;
    flag = true;
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 

  #ifdef DEBUG 
     while(!Serial);
  #endif   
  

  readPtr = 0;
  
  
  
  activity_state = 0;
  int_cnt = 0;
  sec = 0;
  objt = 0;
  senst = 0;
  seccount = 0;
  flag = false;
  listfile = false;
  stopmeasure = false;
  startmeasure = false;
  listtemp = false;
  rmfile = false;
 
  pinMode(PULSE_PIN,OUTPUT);
 
  Serial.println("Initializing ADXL345..."); 

   if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    
  }
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_50_HZ);


  Serial.println("Initializing SD card...");

 
 
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
   }
  Serial.println("card initialized.");

 Serial.println("Initializing temperature...");
  
 tmp007.begin(TMP007_CFG_1SAMPLE);

 Serial.println("Initializing BLE card...");

  blePeripheral.setLocalName("Log");
  blePeripheral.setDeviceName("Log");
  blePeripheral.setAppearance(128);
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());
  blePeripheral.setConnectionInterval(0x0006, 0x0010);


  // add service and characteristic
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchChar);
  blePeripheral.addAttribute(stateChar);
  blePeripheral.addAttribute(secLogged);
  blePeripheral.addAttribute(temp);

  // assign event handlers for connected, disconnected to peripheral
  
  // assign event handlers for characteristic
  switchChar.setEventHandler(BLEWritten, switchCharacteristicWritten);
  stateChar.setEventHandler(BLEWritten, stateCharacteristicWritten);
// set an initial value for the characteristic
  switchChar.setValue(0);
  stateChar.setValue(0);
  secLogged.setValue(0);
  temp.setValue(0);
  

  // advertise the service
 

  
  
  CurieIMU.begin();
  
  Serial.println("Testing device connections...");
  if (CurieIMU.begin()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  CurieIMU.setGyroRange(125);
  CurieIMU.setAccelerometerRange(2);
  CurieIMU.setAccelerometerRate(50);

  blePeripheral.begin();

  Serial.println(("Bluetooth device active, waiting for connections..."));

   
  CurieTimerOne.start(S_RATE, &timerInterrupt); 
  
  #ifdef DEBUG
   
  #else
    doMeasure = false;
  #endif    
 
  
  
}


void loop() {

   blePeripheral.poll();

   if (flag && doMeasure){
    flag = false;
    if (readPtr >= BUFF_SIZE) {
        readPtr = 0;
        saveToFile(BUFF_SIZE);
    }
   
    CurieIMU.readMotionSensor(buf[readPtr].ax, buf[readPtr].ay, buf[readPtr].az,  buf[readPtr].gx, buf[readPtr].gy, buf[readPtr].gz);
    buf[readPtr].ps = analogRead(0);
    buf[readPtr].ahx=accel.getX();
    buf[readPtr].ahy=accel.getY();
    buf[readPtr].ahz=accel.getZ();
    buf[readPtr].ts=millis()-mls;
    buf[readPtr].as=activity_state;
    readPtr++;
   }
  
    if (seccount > 60){
    
    #ifdef DEBUG
      Serial.println("One minute left .....");  
    #endif
      
    float avgobjt = objt / 60;
    objt = 0;
    float avgsenst = senst / 60;
    senst = 0;
    saveTempFile(avgobjt,avgsenst);
    temp.setValue(avgobjt);
    seccount = 0;
  }

  
  if (int_cnt > FREQ) {
       
    if (doMeasure){ 
       sec ++;
       secLogged.setValue(sec);
    }
    int_cnt = 0;
    objt += tmp007.readObjTempC();
    senst += tmp007.readDieTempC();
    seccount++;
    
   }

   if (listfile){
     listfile = false;
     dataFile = SD.open("datalog.txt");
     if (dataFile) {
        Serial.println("datalog.txt:");
       // read from the file until there's nothing else in it:
        while (dataFile.available()) {
            Serial.write(dataFile.read());
             blePeripheral.poll();
            }
           // close the file:
           dataFile.close();
        } else {
         // if the file didn't open, print an error:
       Serial.println("error opening datalog.txt");
       } 
   }

  if (listtemp){
     listtemp = false;
     dataFile = SD.open("templog.txt");
     if (dataFile) {
        Serial.println("templog.txt:");
       // read from the file until there's nothing else in it:
        while (dataFile.available()) {
            Serial.write(dataFile.read());
             blePeripheral.poll();
            }
           // close the file:
           dataFile.close();
        } else {
         // if the file didn't open, print an error:
       Serial.println("error opening templog.txt");
       } 
   }


  if (stopmeasure){
    stopmeasure = false;
    doMeasure = false; 
    digitalWrite(PULSE_PIN,LOW);
    Serial.println("Stop measurement");
    saveToFile(readPtr+1);
    mls = 0;
  }

  if(startmeasure){
    startmeasure = false;
    readPtr = 0;
    sec = 0;
    mls = millis();
    Serial.println("Start measurement");
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataFile.println("-----------------------new----------------------------------");
    dataFile.close();
    digitalWrite(PULSE_PIN,HIGH); 
    doMeasure  = true;
  }

  if (rmfile){
    rmfile = false;
    SD.remove("datalog.txt");
    SD.remove("templog.txt");
  }
   
 }


void saveTempFile(float objt, float senst){
    File dFile = SD.open("templog.txt", FILE_WRITE);
    if (dFile) {
    #ifdef DEBUG
      Serial.println("Swapping to file temperature");
    #endif    
    dFile.print(objt);
    dFile.print(" ");
    dFile.print(senst);
    dFile.print(" ");
    dFile.println(millis());
     }
    dFile.close(); 
  }
  


void saveToFile(int upper_index){
    #ifdef DEBUG
      int i=millis();
      Serial.println("Swapping to file accelerometer");
    #endif    
    Serial.println("Swapping to file accelerometer");
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
     for (int i = 0; i < upper_index; i++){
          dataFile.print(buf[i].ax);
          dataFile.print(" ");
          dataFile.print(buf[i].ay);
          dataFile.print(" ");
          dataFile.print(buf[i].az);
          dataFile.print(" ");
          dataFile.print(buf[i].gx);
          dataFile.print(" ");
          dataFile.print(buf[i].gy);
          dataFile.print(" ");
          dataFile.print(buf[i].gz);
          dataFile.print(" ");
          dataFile.print(buf[i].ahx);
          dataFile.print(" ");
          dataFile.print(buf[i].ahy);
          dataFile.print(" ");
          dataFile.print(buf[i].ahz);
          dataFile.print(" ");
          dataFile.print(buf[i].ps);
          dataFile.print(" ");
          dataFile.print(buf[i].ts);
          dataFile.print(" ");
          dataFile.println(buf[i].as);
     }
    dataFile.close(); 
  }
  #ifdef DEBUG
       i = millis() - i;
       Serial.print("Time to swap: ");
       Serial.println(i);
    #endif    
}
  

void switchCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
   
   switch(switchChar.value()){
    case 0:
     stopmeasure = true;
     break;
    case 1:
     startmeasure = true; 
     break;
    case 2:
     if (doMeasure) Serial.println("Stop measuring first"); else  listfile = true;
     break; 
    case 3:
     if (doMeasure) Serial.println("Stop measuring first"); else  listtemp = true;
     break;  
    case 4:
     if (doMeasure) Serial.println("Stop measuring first"); else  rmfile = true;
     break;  
  }
}





void stateCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  
  activity_state = stateChar.value();
   
}




