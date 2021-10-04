#define DEBUG_SENS  //センサ系のデバッグ用

//////////////通信系定数//////////////
#define Ts 1000 //(ms)タイマ割り込みの周期, 制御周期
#define SENDTIME 4  //送信間隔(s)
#define ALIVE 60  //CANによる生存確認リミット(s)
/////////////////////////////////////

#include "MegaTelemetry.h"  //ライブラリとピン定義

void setup(){
  pinSetup();            //IOピンの設定
  Serial.begin(9600);    //デバッグ用UART通信開始
  Serial.println("MegaTelemtry");
  delay(4000);
  wdt_enable(WDTO_4S);   //n秒周期のウォッチドッグタイマ開始
  Serial2.begin(115200);  //LoRaとの通信開始
//  Serial2.println("MegaTelemtry");
  GNSSsetup();
  wdt_reset();
  Wire.begin();          //I2C通信開始
  setupBME280();
  CANsetup();
  SDsetup();
//  digitalWrite(RST_mega,HIGH);
  MsTimer2::set(Ts, TIME_Interrupt); // TsごとTIME_Interruptを呼び出す
  MsTimer2::start();
}

void loop(){
  BME280_data();
  Create_Buffer_BME280();
  SDWriteData(); 
  if(time_flag==1){
    GNSS_data();
    Create_Buffer_GNSS();
    if (myFile)  myFile.print(Buffer_GNSS);
    Binaly_send();
    time_flag=0;
    #ifdef DEBUG_SENS
    Serial.println(Buffer_GNSS);
    Serial.println(Buffer_BME280);
    #endif
  }
  if (myFile){
    myFile.println();
    myFile.flush();
  } 
  CAN_send();
  
}

///////////////////////サブ関数////////////////////////////
void TIME_Interrupt(void){
  wdt_reset();
  timecount++;
  if(timecount > (int)(SENDTIME*1000/Ts)-1){
    time_flag=1;
    timecount=0;
  }
}

void Binaly_send(){
  int16_t Temp_bin[1] = {(int16_t)(Temp*100)};
  uint16_t Humidity_bin[1] = {(uint16_t)(Humidity*100)};
  uint16_t Pressure_bin[1] = {(uint16_t)(Pressure*10)};
  Serial2.write(18);
  Serial2.write((byte*)Temp_bin,2);
  Serial2.write((byte*)Humidity_bin,2);
  Serial2.write((byte*)Pressure_bin,2);
  Serial2.write((byte*)latitude,4);
  Serial2.write((byte*)longitude,4);
  Serial2.write((byte*)altitude,4);
}
///////////////////////////////////////////////////////////
