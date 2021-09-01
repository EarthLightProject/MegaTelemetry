#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to BMW280
#include <MsTimer2.h>
#include <SparkFunBME280.h>
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

#define O2PWMset OCR3A  //Timer3のDuty設定用レジスタ名
#define AirPWMset OCR3B
#define LPGPWMset OCR3C

#define FILE_NAME "DataLog.txt"

//////////グローバル変数の宣言//////////
int16_t Temp_IN=0.0,Temp_OUT=0.0;
float Humidity_IN=0.0,Humidity_OUT=0.0;
float Pressure_IN=0.0,Pressure_OUT=0.0;
int timecount=0, IG_count=0;
uint8_t IG_flag=0 , Flow_flag=0 , time_flag=0 , IG_point[4]={0} , Pulse_Count = 0;
long latitude , longitude , altitude;
byte Hour, Minute, Second;
String Buffer_GNSS;
String Buffer_TIME;
String Buffer_BME280_OUT;
String Buffer_BME280_IN;
String RECEVE_Str;
////////////////////////////////////

/////////////クラスの宣言/////////////
SFE_UBLOX_GNSS myGNSS;
BME280 BME280_OUT;
BME280 BME280_IN;
File myFile;
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
void Diaphragm_control(void);
void O2_Conrol();
void Air_Control();
void LPG_Control();
void IG_Get();
void Pressure_IG();
void IG_Pulse();
/////////////////////////////////

/****************************************************
  　　　　　　　プロトタイプ宣言した関数
 ****************************************************/
void pinSetup(){
  pinMode(IGsig, INPUT);
  pinMode(Servo_PWM,OUTPUT);
  pinMode(O2_PWM,OUTPUT);
  pinMode(LPG_PWM,OUTPUT);
  pinMode(Air_PWM,OUTPUT);
  pinMode(IGPWM,OUTPUT);
  pinMode(O2_flow,INPUT);
  pinMode(Air_flow,INPUT);
  pinMode(LPG_flow,INPUT);
  pinMode(Thermocouple_PIN,INPUT);
  pinMode(LoRa_RESET,OUTPUT);
  pinMode(RST_mega,OUTPUT);
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
    myFile.print(Buffer_BME280_OUT);
    myFile.print(Buffer_BME280_IN);
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
  latitude = myGNSS.getLatitude();
  longitude = myGNSS.getLongitude();
  altitude = myGNSS.getAltitude();
  Hour = myGNSS.getHour();
  Minute = myGNSS.getMinute();
  Second = myGNSS.getSecond();
}
//////////////////////////////////////////////////////////////////

///////////////////////////BME280////////////////////////////////
void setupBME280(void) {
  BME280_OUT.setI2CAddress(0x77);
  if (BME280_OUT.beginI2C()) {
    Wire.beginTransmission(0x77);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 Outside OK");
    #endif
    //return;
    delay(50);
  }
  BME280_IN.setI2CAddress(0x76);
  if (BME280_IN.beginI2C()) {
    Wire.beginTransmission(0x76);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 Inside OK");
    #endif
    return;
  }
  #ifdef DEBUG_SENS
  Serial.println("Sensor connect failed");
  #endif
}

void BME280_OUT_data(void) {
  Temp_OUT = (int16_t)(100*BME280_OUT.readTempC()); //°C
  Humidity_OUT = BME280_OUT.readFloatHumidity(); //%
  Pressure_OUT = BME280_OUT.readFloatPressure() / 100; //hPa
}

void BME280_IN_data(void){
  Temp_IN = (int16_t)(100*BME280_IN.readTempC()); //°C
  Humidity_IN = (uint16_t)(100*BME280_IN.readFloatHumidity()); //%
  Pressure_IN = BME280_IN.readFloatPressure() / 100; //hPa
}

//////////////////////////////////////////////////////////////////

///////////////////////////Buffer/////////////////////////////////
void Create_Buffer_GNSS(void){
  Buffer_GNSS.remove(0);
  Buffer_GNSS.concat(latitude/10000000);
  Buffer_GNSS.concat(".");
  Buffer_GNSS.concat(latitude%10000000);
  Buffer_GNSS.concat(",");
  Buffer_GNSS.concat(longitude/10000000);
  Buffer_GNSS.concat(".");
  Buffer_GNSS.concat(longitude%10000000);
  Buffer_GNSS.concat(",");
  Buffer_GNSS.concat(altitude/1000);
  Buffer_GNSS.concat(".");
  Buffer_GNSS.concat(altitude%1000);
}

void Create_Buffer_TIME(){
  Buffer_TIME.remove(0);
  Buffer_TIME.concat(Hour);
  Buffer_TIME.concat(",");
  Buffer_TIME.concat(Minute);
  Buffer_TIME.concat(",");
  Buffer_TIME.concat(Second);
}

void Create_Buffer_BME280_OUT(void){
 // Buffer.concat("Latitude,Longitude,Height\n"); 
  Buffer_BME280_OUT.remove(0);
  Buffer_BME280_OUT.concat(Temp_OUT);
  Buffer_BME280_OUT.concat(","); 
  Buffer_BME280_OUT.concat(Humidity_OUT);
  Buffer_BME280_OUT.concat(",");
  Buffer_BME280_OUT.concat(Pressure_OUT);
  Buffer_BME280_OUT.concat(",");
}

void Create_Buffer_BME280_IN(void){
  Buffer_BME280_IN.remove(0);
  Buffer_BME280_IN.concat(Temp_IN);
  Buffer_BME280_IN.concat(","); 
  Buffer_BME280_IN.concat(Humidity_IN);
  Buffer_BME280_IN.concat(",");
  Buffer_BME280_IN.concat(Pressure_IN);
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
       IG_flag = 1;
       Flow_flag = !Flow_flag;
       IG_count = ig_time;
       #ifdef DEBUG_SENS
       Serial.println("get REIG");
       #endif
      }
      RECEVE_Str.remove(0);
    }
}
