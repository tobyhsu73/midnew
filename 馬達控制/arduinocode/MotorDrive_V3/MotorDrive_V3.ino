/* ==================================================
 * MotorDrive_V3_1025
 * 2021/10/01
 * 執行基本遙控功能+USB控制功能
 * 
 * 1025 新增USB指令持續時間指令
 * ==================================================
 */
//==============================標頭及宣告==============================
//=====函式庫=====
#include <EnableInterrupt.h>  //引腳變化中斷函式庫
#include "sbus.h"  //函式庫 S-bus

//=====可設定參數=====
  //馬達與編碼器方向設定
const bool motorDirectionConfig[4] = {1,1,0,0};  //馬達轉向配置(調整DIR腳位的正反轉定義) 預設0
const bool enableDirectionConfig[4] = {0,0,1,1};  //編碼器轉向配置(調整A-B超前的正反轉定義)  預設1:A超前B為正向
const byte sbusChannelSize = 12;
const int SbusValueRange[3] = {306,996,1694};

//=====定義腳位=====
  //Micro-馬驅板
const byte DirPin[4] = {2, 3, 12, 13};
const byte PwmPin[4] = {5, 6, 10, 11};
const byte SlpPin[4] = {4, 4, 7, 7};  //始能  高電位始能
const byte CsPin[4] = {A5, A4, A3, A2};  //電流檢測
  //Micro-編碼器
const byte EncoderPinA[4] = {17, 16, 9, 8};  //PCINT　17-SS 、 16-MOSI
const byte EncoderPinB[4] = {14, 15, A0, A1};  //14-MISO 、 15-SCK

//=====控制輸入輸出
  //指令與實際輸出
int LSpeed,RSpeed;  //左右輪的速度(由Sbus和USB共用)
int motorPWM[4] = {0, 0, 0, 0};  //四只馬達的PWM
bool motorDir[4] = {0, 0, 0, 0};  //四只馬達的方向，(0正轉，1反轉)
byte MaxSpeed;  //允許的最高速度
//=====編碼器控制
  //編碼器讀數
volatile long EncoderValue[4] = {0, 0, 0, 0};   //編碼器觸發次數累計
long MemoryEncoderValue[2][4] = {{0, 0, 0, 0},{0, 0, 0, 0}};  //本次紀錄與前次紀錄
float TargetEncoderValue[4] = {0, 0, 0, 0};  //換算後的目標
float DurationStepSize[4] = {0, 0, 0, 0};  //每次的變動量
float DifferenceDuration[4] = {0, 0, 0, 0};  //距離目標的差額
int DeltaDuration[4] = {0, 0, 0, 0};  //與上次的變化量
  //時間
unsigned long lastSbusTime;
unsigned long lastUsbTime;
int usbDuration;  //USB的指令持續時間  map(duration_usb,0,255,0,2550);  //最久2.55秒

//=====物件=====
SbusRx sbus_rx(&Serial1);

//=====測試=====

//==============================end 標頭及宣告==============================

//==============================setup==============================
void setup(){
  delay(100);
  //Arduino Micro 序列埠操作
  //while (!Serial);
  delay(10);
  while(Serial.read()!=-1);  //清緩存
  //開啟S-bus
  sbus_rx.Begin();
  
  //設定編碼器中斷觸發引腳、涵式、觸發類型
  enableInterrupt(EncoderPinA[0], myFunctionA, CHANGE); 
  enableInterrupt(EncoderPinA[1], myFunctionB, CHANGE);
  enableInterrupt(EncoderPinA[2], myFunctionC, CHANGE); 
  enableInterrupt(EncoderPinA[3], myFunctionD, CHANGE);

  //引腳模式設定
    //馬達驅動  pwm及cs不必設定
  for(byte n=0;n<4;n++){
    pinMode(DirPin[n],OUTPUT);  //方向控制
    pinMode(SlpPin[n],OUTPUT);  //致能控制
  }
    //編碼器
  for(byte n=0;n<4;n++){
    pinMode(EncoderPinA[n],INPUT_PULLUP); //編碼器脈衝讀取-輸入上拉
    pinMode(EncoderPinB[n],INPUT_PULLUP);
  }
}
//==============================end setup==============================

//==============================loop==============================
void loop(){
  //讀取SBUS並控制
  if(sbus_rx.Read()) {
    //Sbus讀取指令如下
    //sbus_rx.rx_channels()[i]
    //sbus_rx.lost_frame()
    //sbus_rx.failsafe()
    
    //動作
        //丟包或讀取失敗-停車
    if(sbus_rx.lost_frame() || sbus_rx.failsafe()){
      if(millis()-lastSbusTime < 3000){
        for(byte n=0;n<4;n++){
          digitalWrite(SlpPin[n],LOW);
          analogWrite(PwmPin[n],0);
        }
      }
    }else{  //成功讀取到
      lastSbusTime = millis();  //記下Sbus最後一次成功讀取的時間點
        //由ch8 VR(B) 控制最高速度
      if(sbus_rx.rx_channels()[7] < 310){
        MaxSpeed = 20;
      }else if(sbus_rx.rx_channels()[7] > 1690){
        MaxSpeed = 255;
      }else{
        MaxSpeed =map(sbus_rx.rx_channels()[7],310,1690,20,255);
      }
      Serial.print(" maxSpeed= ");Serial.print(MaxSpeed);Serial.print("\t");

      //由ch2 左Y 決定進退速度
      int Speed;
      if(sbus_rx.rx_channels()[1] < 310){  //全速前進
        Speed = MaxSpeed;
      }else if(sbus_rx.rx_channels()[1] > 1690){  //全速後退
        Speed = -MaxSpeed;
      }else if(sbus_rx.rx_channels()[1] < 970){  //前進
        Speed = map(sbus_rx.rx_channels()[1],970,310,10,MaxSpeed);
      }else if(sbus_rx.rx_channels()[1] > 1030){  //後退
        Speed = map(sbus_rx.rx_channels()[1],1030,1690,-10,-MaxSpeed);
      }else{
        Speed = 0;
      }

      //由ch4 左x 決定轉向
      int turnSpeed;
      if(sbus_rx.rx_channels()[3] < 310){  //全速左轉
        turnSpeed = MaxSpeed;
      }else if(sbus_rx.rx_channels()[3] > 1690){  //全速右轉
        turnSpeed = -MaxSpeed;
      }else if(sbus_rx.rx_channels()[3] < 970){  //左轉
        turnSpeed = map(sbus_rx.rx_channels()[3],970,310,10,MaxSpeed);
      }else if(sbus_rx.rx_channels()[3] > 1030){  //右轉
        turnSpeed = map(sbus_rx.rx_channels()[3],1030,1690,-10,-MaxSpeed);
      }else{
        turnSpeed = 0;
      }

      Serial.print(" Speed= ");Serial.print(Speed);Serial.print("\t");
      Serial.print(" turnSpeed= ");Serial.print(turnSpeed);Serial.print("\t");
      Serial.println();

      //將進退及轉向換算成左右轉速
      differentialCalculation(Speed,turnSpeed);
      //將左右轉速換算成PWM及DIR
      LRspeedToPWM();
      //馬達動作
      motorAct();
    }
  }else{
    if(millis()-lastUsbTime > usbDuration && millis()-lastUsbTime < 5000){  //超過持續時間沒接收USB指令
      LSpeed = 0;
      RSpeed = 0;
      LRspeedToPWM();
      motorAct();
    }
  }

  //=====USB控制
  if(Serial.available()>4){  //標頭(254) 左 右 方向 持續時間 效驗
    int header_usb = Serial.read();
    if(header_usb == 'f'){
      int L_speed_usb = Serial.read();
      int R_speed_usb = Serial.read();
      byte dir_usb = Serial.read();
      byte duration_usb = Serial.read();
      byte check_usb  = Serial.read();
      if(check_usb == (L_speed_usb+R_speed_usb+dir_usb+duration_usb)%255){  //確認效驗
        Return();  //回傳資料
        lastUsbTime = millis();  //紀錄最後一次USB成功的時間點
        usbDuration = map(duration_usb,0,255,0,2550);  //最久2.55秒
        if(millis()-lastSbusTime > 5000){  //5秒內沒有遙控
          LSpeed = L_speed_usb;
          RSpeed = R_speed_usb;
          if(!bitRead(dir_usb,1)){  //0是反轉
            LSpeed = -LSpeed;
          }
          if(!bitRead(dir_usb,0)){  //0是反轉
            RSpeed = -RSpeed;
          }
          LRspeedToPWM();
          motorAct();
        }
      }
    }
  }
  
}
//==============================end loop==============================


//==============================副程式==============================

//====================中斷副程式====================
  //=====中斷1~4=====
void myFunctionA(){EncoderRead(0);}
void myFunctionB(){EncoderRead(1);}
void myFunctionC(){EncoderRead(2);}
void myFunctionD(){EncoderRead(3);}
  //=====中斷共用涵式=====
int EncoderRead(byte n){  //根據AB相以及設定的轉向來判斷EncoderValue應加或減
  //若A超前B，則A的數值會與B相異(A先變化，B還沒變化，因此相異)
  if((digitalRead(EncoderPinA[n]) != digitalRead(EncoderPinB[n]))==enableDirectionConfig[n]){  //A超前B==方向
    EncoderValue[n]++;
  }else{
    EncoderValue[n]--;
  }
}
//====================end 中斷副程式====================

//====================速度計算副程式====================
//將遙控器的"前後"及"左右轉"換算成左右輪速度
void differentialCalculation(int Speed,int turnSpeed){
  //計算差速後速度
  if(Speed-turnSpeed/2 > 255){  //左輪前進過快
    LSpeed = 255;
    RSpeed = 255+turnSpeed;
  }else if(Speed+turnSpeed/2 > 255){  //右輪前進過快
    LSpeed = 255-turnSpeed;
    RSpeed = 255;
  }else if(Speed-turnSpeed/2 < -255){  //左輪後退過快
    LSpeed = -255;
    RSpeed = -255+turnSpeed;
  }else if(Speed+turnSpeed/2 < -255){  //右輪後退過快
    LSpeed = -255-turnSpeed;
    RSpeed = -255;
  }else{
    LSpeed = Speed-turnSpeed/2;
    RSpeed = Speed+turnSpeed/2;
  }
}
//====================end 速度計算副程式====================

//====================速度換算PWM副程式====================
void LRspeedToPWM(){
  if(RSpeed>0){
    motorDir[0] = motorDirectionConfig[0];
    motorDir[1] = motorDirectionConfig[1];
    motorPWM[0] = RSpeed;
    motorPWM[1] = RSpeed;
  }else{
    motorDir[0] = !motorDirectionConfig[0];
    motorDir[1] = !motorDirectionConfig[1];
    motorPWM[0] = -RSpeed;
    motorPWM[1] = -RSpeed;
  }
  if(LSpeed>0){
    motorDir[2] = motorDirectionConfig[2];
    motorDir[3] = motorDirectionConfig[3];
    motorPWM[2] = LSpeed;
    motorPWM[3] = LSpeed;
  }else{
    motorDir[2] = !motorDirectionConfig[2];
    motorDir[3] = !motorDirectionConfig[3];
    motorPWM[2] = -LSpeed;
    motorPWM[3] = -LSpeed;
  }
}
//====================end 速度換算PWM副程式====================

//====================馬達動作副程式====================
void motorAct(){
  for(byte n=0;n<4;n++){
    digitalWrite(SlpPin[n],HIGH);
    digitalWrite(DirPin[n],motorDir[n]);
    analogWrite(PwmPin[n],motorPWM[n]);
  }
}
//====================end 馬達動作副程式====================

//====================資料回傳副程式====================
void Return(){  //以 python dict
  Serial.print("{");
  Serial.print("\"TOM\":");Serial.print(millis());  //時間輟 Time Of Motor
  Serial.print(",\"LSpeed\":");Serial.print(LSpeed);
  Serial.print(",\"RSpeed\":");Serial.print(RSpeed);
  for(byte n=0;n<4;n++){  //編碼器
    Serial.print(",\"Encoder");Serial.print(n);Serial.print("\":");
    Serial.print(EncoderValue[n]);
  }
  Serial.print("}");
  Serial.println();
}
//====================end 資料回傳副程式====================


/* //測試紀錄
 * 07/17  有編碼器計數和SBUS讀取並print功能，迴圈一次約1.13ms
 * 
 * 做一次long x=micros()的讀寫要3us
 * 
 * 
 * 
 * 
 * 
 */
