#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to BMW280
#include <MsTimer2.h>
#include <SparkFunBME280.h>
#include <mcp_can.h>
#include <avr/wdt.h>

//pin define
#define GNSS_RST 22
#define Servo_PWM 46
#define IGsig 40
#define CS 53
#define IGPWM 11
#define O2_flow A0
#define Air_flow A1
#define LPG_flow A2
#define LPG_PWM 3
#define Air_PWM 2
#define O2_PWM 5
#define Thermocouple_PIN A3
#define LoRa_RESET 24
#define RST_mega 7
#define SW1 8
#define SW2 9
#define SW3 10
#define CAN0_INT 40                   // Set INT to pin 40


#define FILE_NAME "DataLog.txt"

//////////グローバル変数の宣言//////////
float Temp=0;
float Humidity=0;
float Pressure=0;
int timecount=0, IG_count=0;
uint8_t time_flag=0 , Press_IG_count[5]={0};
long latitude[1]={0} , longitude[1]={0} , altitude[1]={0};
byte Hour, Minute, Second;
String Buffer_GNSS;
String Buffer_TIME;
String Buffer_BME280;
String RECEVE_Str;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char CAN_data[8]={0};
////////////////////////////////////

/////////////クラスの宣言/////////////
SFE_UBLOX_GNSS myGNSS;
BME280 bme280;
File myFile;
MCP_CAN CAN0(42);  // Set CS to pin 42
///////////////////////////////////


////////////関数の宣言//////////////
void pinSetup(void);
void GNSSsetup(void);
void GNSS_data(void);
void init_BME280(void);
void setupBME280(void);
void BME280_data(void);
void SDsetup(void);
void Serial_print(void);
void Create_Buffer(void);
void SDWriteData(void);
void CANsetup(void);
void CAN_read(void);
void CAN_send(void);
void IG_Get();
/////////////////////////////////

/****************************************************
  　　　　　　　プロトタイプ宣言した関数
 ****************************************************/
void pinSetup(){
  pinMode(SW1,INPUT);
  pinMode(SW2,INPUT);
  pinMode(SW3,INPUT);
  pinMode(LoRa_RESET,OUTPUT);
  pinMode(RST_mega,OUTPUT);
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  digitalWrite(LoRa_RESET,HIGH);
  digitalWrite(RST_mega,LOW);
  //pinMode(GNSS_RST,OUTPUT);
  //digitalWrite(GNSS_RST,HIGH);
}

////////////////////////////////SD Card//////////////////////////////
void SDsetup(){
  #ifdef DEBUG_SENS
  Serial.print("Initializing SD card...");
  #endif
  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
    //while (1);  //ウォッチドッグに引っかかって再起動する
  }
  myFile = SD.open(FILE_NAME, FILE_WRITE);
  myFile.write("Time(ms),");
  myFile.write("Temp_Out,Humidity_Out,PressureOut,Temp_In,Humidity_In,Pressure_In,");
  myFile.write("Latitude,Longitude,Height,");
  myFile.println("Hour,Min,Sec");
  myFile.flush(); 
  #ifdef DEBUG_SENS
  Serial.println("initialization done.");
  #endif
}

void SDWriteData(void) {
  if (myFile) {               // if the file opened okay, write to it:
    myFile.print(millis());
    myFile.write(',');
    myFile.print(Buffer_BME280);
  }
}
//////////////////////////////////////////////////////////////////////

/////////////////////////////////GNSS/////////////////////////////////
void GNSSsetup(void) {
  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages
  do {
    #ifdef DEBUG_SENS
    Serial.println("GNSS: trying 38400 baud");
    #endif
    Serial1.begin(38400);
    if (myGNSS.begin(Serial1) == true) break;
    delay(100);
    #ifdef DEBUG_SENS
    Serial.println("GNSS: trying 9600 baud");
    #endif
    Serial1.begin(9600);
    if (myGNSS.begin(Serial1) == true) {
        #ifdef DEBUG_SENS
        Serial.println("GNSS: connected at 9600 baud, switching to 38400");
        #endif
        myGNSS.setSerialRate(38400);
        delay(100);
    } else {
        //myGNSS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  #ifdef DEBUG_SENS
  Serial.println("GNSS serial connected");
  #endif
  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
}

void GNSS_data(void) {
  latitude[0] = myGNSS.getLatitude();
  longitude[0] = myGNSS.getLongitude();
  altitude[0] = myGNSS.getAltitude();
  Hour = myGNSS.getHour();
  Minute = myGNSS.getMinute();
  Second = myGNSS.getSecond();
}
//////////////////////////////////////////////////////////////////

///////////////////////////////CAN////////////////////////////////////
void CANsetup(){
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
}

void CAN_read(){
  if(!digitalRead(CAN0_INT)) {                // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    if(rxId == 0x100){  //生存確認
      
    }
  }  
}

void CAN_send(){
  if((540.19 > Pressure) && (Pressure > 264.36) && Press_IG_count[0] == 0 ){
    byte sndStat = CAN0.sendMsgBuf(0x200, 0, 1, CAN_data);
    if(sndStat == CAN_OK){
      Serial.println("SEND CAN0");
      Press_IG_count[0] = 1;
    }
     else Serial.println("Fail CAN0");
  }
  if((264.36 > Pressure) && (Pressure > 120.45) && Press_IG_count[1] == 0 ){
    byte sndStat = CAN0.sendMsgBuf(0x200, 0, 1, CAN_data);
    if(sndStat == CAN_OK){
      Serial.println("SEND CAN1");
      Press_IG_count[1] = 1;
    }
  }
  if((120.45 > Pressure) && (Pressure > 54.74) && Press_IG_count[2] == 0 ){
    byte sndStat = CAN0.sendMsgBuf(0x200, 0, 1, CAN_data);
    if(sndStat == CAN_OK){
      Serial.println("SEND CAN2");
      Press_IG_count[2] = 1;
    }
  }
  if((54.74 > Pressure) && (Pressure > 1.0) && Press_IG_count[3] == 0 ){
    byte sndStat = CAN0.sendMsgBuf(0x300, 0, 1, CAN_data);
    if(sndStat == CAN_OK){
      Serial.println("SEND CAN3");
      Press_IG_count[3] = 1;
    }
  }
}

//////////////////////////////////////////////////////////////////////



///////////////////////////BME280////////////////////////////////
void setupBME280(void) {
  bme280.setI2CAddress(0x77);
  if (bme280.beginI2C()) {
    Wire.beginTransmission(0x77);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 adress 0x77 OK");
    #endif
    return;
    delay(50);
  }
  bme280.setI2CAddress(0x76);
  if (bme280.beginI2C()) {
    Wire.beginTransmission(0x76);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 adress 0x76 OK");
    #endif
    return;
  }
  #ifdef DEBUG_SENS
  Serial.println("Sensor connect failed");
  #endif
}

void BME280_data(void) {
  Temp = bme280.readTempC(); //°C
  Humidity = bme280.readFloatHumidity(); //%
  Pressure = bme280.readFloatPressure() / 100; //hPa
}

//////////////////////////////////////////////////////////////////

///////////////////////////Buffer/////////////////////////////////
void Create_Buffer_GNSS(void){
  Buffer_GNSS.remove(0);
  Buffer_GNSS.concat(latitude[0]/10000000);
  Buffer_GNSS.concat(".");
  Buffer_GNSS.concat(latitude[0]%10000000);
  Buffer_GNSS.concat(",");
  Buffer_GNSS.concat(longitude[0]/10000000);
  Buffer_GNSS.concat(".");
  Buffer_GNSS.concat(longitude[0]%10000000);
  Buffer_GNSS.concat(",");
  Buffer_GNSS.concat(altitude[0]/1000);
  Buffer_GNSS.concat(".");
  Buffer_GNSS.concat(altitude[0]%1000);
  Buffer_GNSS.concat(",");
  Buffer_GNSS.concat(Hour);
  Buffer_GNSS.concat(",");
  Buffer_GNSS.concat(Minute);
  Buffer_GNSS.concat(",");
  Buffer_GNSS.concat(Second);
}

void Create_Buffer_TIME(){
  Buffer_TIME.remove(0);
  Buffer_TIME.concat(Hour);
  Buffer_TIME.concat(",");
  Buffer_TIME.concat(Minute);
  Buffer_TIME.concat(",");
  Buffer_TIME.concat(Second);
}

void Create_Buffer_BME280(void){
  Buffer_BME280.remove(0);
  Buffer_BME280.concat(Temp);
  Buffer_BME280.concat(","); 
  Buffer_BME280.concat(Humidity);
  Buffer_BME280.concat(",");
  Buffer_BME280.concat(Pressure);
  Buffer_BME280.concat(",");
}



void IG_Get(int ig_time){
  if(Serial.available()>3){
     #ifdef DEBUG_SENS
       Serial.println("available");
     #endif
    do{
      RECEVE_Str.concat(char(Serial.read()));
    }while(Serial.available()>0);
    #ifdef DEBUG_SENS
    Serial.println(RECEVE_Str);
    #endif
    if(RECEVE_Str.compareTo("REIG") == 0){
       #ifdef DEBUG_SENS
       Serial.println("get REIG");
       #endif
      }
      RECEVE_Str.remove(0);
    }
}
