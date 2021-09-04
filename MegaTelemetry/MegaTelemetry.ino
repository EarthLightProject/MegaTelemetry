#define DEBUG_SENS  //センサ系のデバッグ用

//////////////通信系定数//////////////
#define Ts 1000 //(ms)タイマ割り込みの周期, 制御周期
#define SENDTIME 1  //送信間隔(s)
/////////////////////////////////////

#include "MegaTelemetry.h"  //ライブラリとピン定義

void setup(){
  pinSetup();            //IOピンの設定
  Serial.begin(9600);    //デバッグ用UART通信開始
  Serial.println("MegaTelemtry");
  delay(4000);
  wdt_enable(WDTO_4S);   //n秒周期のウォッチドッグタイマ開始
  Serial2.begin(115200);  //LoRaとの通信開始
  Serial2.println("MegaTelemtry");
//  GNSSsetup();
  wdt_reset();
//  Wire.begin();          //I2C通信開始
//  setupBME280();
  SDsetup();
//  digitalWrite(RST_mega,HIGH);
  MsTimer2::set(Ts, TIME_Interrupt); // TsごとTIME_Interruptを呼び出す
  MsTimer2::start();
}

void loop(){
//  BME280_data();
//  Create_Buffer_BME280();
//  SDWriteData();
  /*if(time_flag!=0){
    GNSS_data();
    Create_Buffer_GNSS();
    Create_Buffer_TIME();
    myFile.write(',');
    myFile.print(Buffer_GNSS);
    myFile.write(',');
    myFile.print(Buffer_TIME);
    time_flag=0;*/
    
//  }
  // myFile.println();
  // myFile.flush(); 
}

///////////////////////サブ関数////////////////////////////
void TIME_Interrupt(void){
  wdt_reset();
  timecount++;
  if(timecount>(int)(1000/Ts)) time_flag=1;
}

void Serial_print(void){
  Serial.print(Buffer_BME280);
  Serial.print(Buffer_GNSS); 
}
///////////////////////////////////////////////////////////
