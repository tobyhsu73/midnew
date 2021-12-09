# -*- coding: utf-8 -*-
"""
motor_V3

巡航車馬驅V3  (V3_1025)

配合Arduino MotorDrive_V3_1025
資料 標頭(f) 左 右 方向 效驗(左+右+方向%255)
"""
import serial
    
#本檔案自行連接Serial
def begin(SerPort):
    global ser
    ser = serial.Serial(SerPort,115200,timeout=0.1)
    
#他處連接Serial之後提供給此
def getSer(SerObj):
    global ser
    ser = SerObj
    ser.timeout = 0.1

def go(lsp=0,rsp=0,duration = 0.5):
    global ser
    Dir = 0
    duration = min(duration,2.25)
    duration = int(duration*100)
    if lsp > 0:
        Dir = 2
    if rsp > 0:
        Dir = Dir+1
    lsp = abs(lsp)  #轉成正數
    rsp = abs(rsp)
    lsp = min(lsp,255)  #限制最大值
    rsp = min(rsp,255)
    chick = (lsp + rsp + Dir + duration)%255  #計算效驗
    pack = bytearray(b'f\x00\x00\x00\x00\x00')  #標頭(f) 左 右 方向 持續時間 效驗((左+右+方向+持續時間)%255)
    pack[1] = int(lsp)
    pack[2] = int(rsp)
    pack[3] = int(Dir)
    pack[4] = int(duration)
    pack[5] = int(chick)
    ser.write(pack)
    
def read():
    global ser
    data = eval(ser.readline())
    return data

if __name__=='__main__':
    global ser
    begin("/dev/ttyACM_Micro")









"""
/* ==================================================
 * MotorDrive_V3_1001
 * 2021/10/01
 * 執行基本遙控功能+USB控制功能
 * ==================================================
 */
#include <EnableInterrupt.h>  //引腳變化中斷函式庫
#include "sbus.h"  //函式庫 S-bus
const bool motorDirectionConfig[4] = {1,1,0,0};  //馬達轉向配置(調整DIR腳位的正反轉定義) 預設0
const bool enableDirectionConfig[4] = {0,0,1,1};  //編碼器轉向配置(調整A-B超前的正反轉定義)  預設1:A超前B為正向
const byte sbusChannelSize = 12;
const int SbusValueRange[3] = {306,996,1694};
const byte DirPin[4] = {2, 3, 12, 13};
const byte PwmPin[4] = {5, 6, 10, 11};
const byte SlpPin[4] = {4, 4, 7, 7};  //始能  高電位始能
const byte CsPin[4] = {A5, A4, A3, A2};  //電流檢測
const byte EncoderPinA[4] = {17, 16, 9, 8};  //PCINT　17-SS 、 16-MOSI
const byte EncoderPinB[4] = {14, 15, A0, A1};  //14-MISO 、 15-SCK
int LSpeed,RSpeed;  //左右輪的速度(由Sbus和USB共用)
int motorPWM[4] = {0, 0, 0, 0};  //四只馬達的PWM
bool motorDir[4] = {0, 0, 0, 0};  //四只馬達的方向，(0正轉，1反轉)
byte MaxSpeed;  //允許的最高速度
volatile long EncoderValue[4] = {0, 0, 0, 0};   //編碼器觸發次數累計
long MemoryEncoderValue[2][4] = {{0, 0, 0, 0},{0, 0, 0, 0}};  //本次紀錄與前次紀錄
float TargetEncoderValue[4] = {0, 0, 0, 0};  //換算後的目標
float DurationStepSize[4] = {0, 0, 0, 0};  //每次的變動量
float DifferenceDuration[4] = {0, 0, 0, 0};  //距離目標的差額
int DeltaDuration[4] = {0, 0, 0, 0};  //與上次的變化量
unsigned long lastSbusTime;
unsigned long lastUsbTime;
SbusRx sbus_rx(&Serial1);
void setup(){
  delay(100);
  delay(10);
  while(Serial.read()!=-1);  //清緩存
  sbus_rx.Begin();
  enableInterrupt(EncoderPinA[0], myFunctionA, CHANGE); 
  enableInterrupt(EncoderPinA[1], myFunctionB, CHANGE);
  enableInterrupt(EncoderPinA[2], myFunctionC, CHANGE); 
  enableInterrupt(EncoderPinA[3], myFunctionD, CHANGE);
  for(byte n=0;n<4;n++){
    pinMode(DirPin[n],OUTPUT);  //方向控制
    pinMode(SlpPin[n],OUTPUT);  //致能控制
  }
  for(byte n=0;n<4;n++){
    pinMode(EncoderPinA[n],INPUT_PULLUP); //編碼器脈衝讀取-輸入上拉
    pinMode(EncoderPinB[n],INPUT_PULLUP);
  }
}
void loop(){
  if(sbus_rx.Read()) {
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
      differentialCalculation(Speed,turnSpeed);
      LRspeedToPWM();
      motorAct();
    }
  }
  if(Serial.available()>4){  //標頭(f) 左 右 方向 效驗(左+右+方向%255)
    int header_usb = Serial.read();
    if(header_usb == 'f'){
      int L_speed_usb = Serial.read();
      int R_speed_usb = Serial.read();
      byte dir_usb = Serial.read();
      byte check_usb  = Serial.read();
      if(check_usb == (L_speed_usb+R_speed_usb+dir_usb)%255){  //確認效驗
        Return();  //回傳資料
        lastUsbTime = millis();  //紀錄最後一次USB成功的時間點
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
  if(millis()-lastUsbTime > 5000){  //5秒沒接收USB指令
      LSpeed = 0;
      RSpeed = 0;
      LRspeedToPWM();
      motorAct();
  }
  
}
void myFunctionA(){EncoderRead(0);}
void myFunctionB(){EncoderRead(1);}
void myFunctionC(){EncoderRead(2);}
void myFunctionD(){EncoderRead(3);}
int EncoderRead(byte n){  //根據AB相以及設定的轉向來判斷EncoderValue應加或減
  //若A超前B，則A的數值會與B相異(A先變化，B還沒變化，因此相異)
  if((digitalRead(EncoderPinA[n]) != digitalRead(EncoderPinB[n]))==enableDirectionConfig[n]){  //A超前B==方向
    EncoderValue[n]++;
  }else{
    EncoderValue[n]--;
  }
}
void differentialCalculation(int Speed,int turnSpeed){
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
void motorAct(){
  for(byte n=0;n<4;n++){
    digitalWrite(SlpPin[n],HIGH);
    digitalWrite(DirPin[n],motorDir[n]);
    analogWrite(PwmPin[n],motorPWM[n]);
  }
}
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
"""