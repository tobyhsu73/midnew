#include <SoftwareSerial.h>

SoftwareSerial ul1 (7,6 );//設定軟串口
SoftwareSerial ul2 (5,4);
SoftwareSerial ul3 (3,2);
byte hdr, data_h, data_l, chksum;
unsigned int distance;
int u4,u5,u6;
void setup()
{
  Serial.begin(57600);
  while (!Serial);
  ul1.begin(9600);//軟串口鮑率設定
  ul2.begin(9600);
  ul3.begin(9600);
}

void loop(){ 
  ul1.listen();
  ul1.print("dat");
  delay(50);
  if (ul1.available()){
    hdr = (byte)ul1.read();
    if (hdr == 255)
    {
      data_h = (byte)ul1.read();
      data_l = (byte)ul1.read();
      chksum = (byte)ul1.read();
      if (chksum == ((hdr + data_h + data_l)&0x00FF))
      {    
        distance = data_h * 256 + data_l;
        u4=distance;
      }
    }
  }

  ul2.listen();
  ul2.print("dat");
  delay(50);
  if (ul2.available())
  {
    hdr = (byte)ul2.read();
    if (hdr == 255)
    {
      data_h = (byte)ul2.read();
      data_l = (byte)ul2.read();
      chksum = (byte)ul2.read();
      if (chksum == ((hdr + data_h + data_l)&0x00FF))
      {    
        distance = data_h * 256 + data_l;
        u5=distance;
      }
    }
  }

  ul3.listen();
  ul3.print("ggg");
  delay(50);
  if (ul3.available())
  {
    hdr = (byte)ul3.read();
    if (hdr == 255)
    {
      data_h = (byte)ul3.read();
      data_l = (byte)ul3.read();
      chksum = (byte)ul3.read();
      if (chksum == ((hdr + data_h + data_l)&0x00FF))
      {    
        distance = data_h * 256 + data_l;
        u6=distance;
      }
    }
  }

  //輸出格式python dictionary 
  Serial.print("{\'u4\':");
  Serial.print(u4);
  Serial.print(",\'u5\':");
  Serial.print(u5);
  Serial.print(",\'u6\':");
  Serial.print(u6);
  Serial.println("}");

}
