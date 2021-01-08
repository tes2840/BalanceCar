//
// BalaC balancing robot (IMU:MPU6886)
// by Kiraku Labo
//
// 1. Lay the robot flat, and power on.
// 2. Wait until Gal-1 (Pitch Gyro calibration) completes.
// 3. Hold still the robot upright in balance until Cal-2 (Accel & Yaw Gyro cal) completes.
//
// short push of power button: Gyro calibration
// long push (>1sec) of power button: switch mode between standig and demo(circle)
// 

#include <M5StickC.h>
#include <ArduinoJson.h>
#include "Network.h"
#include "InvertedPendulumControl.h"
#include "Global.h"

/* Json */
#define JSON_CAPACITY 1024    //JSONを保存する最大サイズ(byte)
StaticJsonDocument<JSON_CAPACITY> doc;
JsonObject obj = doc.to<JsonObject>();

struct json_member{
  const char *id;
  float value;
};
json_member json_object_list[] ={
  { KEY_TIME, 0.0 },
  { KEY_POWER_R, 0.0 },
  { KEY_POWER_L, 0.0 },
  { KEY_GYRO_OFFSET_X, 0.0 },
  { KEY_GYRO_OFFSET_Y, 0.0 },
  { KEY_GYRO_OFFSET_Z, 0.0 },
  { KEY_ACC_OFFSET_X, 0.0 },
  { KEY_GYRO_X, 0.0 },
  { KEY_GYRO_Y, 0.0 },
  { KEY_GYRO_Z, 0.0 },
  { KEY_ACC_X, 0.0 },
  { KEY_ACC_Z, 0.0 },
  { KEY_AVE_ACC_Z, 0.0 },
  { KEY_AVE_ABS_OMG, 0.0 }
};
const unsigned int json_object_list_size = sizeof(json_object_list)/ sizeof(json_object_list[0]);

char logData[JSON_CAPACITY] = "";

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  Serial.begin(115200);
  M5.begin();
  Wire.begin(0, 26); //SDA,SCL
  M5.Axp.ScreenBreath(11);
  M5.Lcd.setRotation(2);
  M5.Lcd.setTextFont(4);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
}

void loop() {
  //グローバルにインスタンスを生成するとリセットするためループ内に宣言する(実行順番の関係？)
  static InvertedPendulumControl IPD;
  static Network network;  
  static int counter = 0;
  
  IPD.run();

  counter++;
  //50 -> 0.5秒に一回ログを出力する　動作上もOK 10ms周期に実行されている模様
  if( counter >20){
    outputLog(&IPD, &network);
    counter  = 0;
  }
}

/*
* logを出力する関数
* input :IPD(倒立振り子制御クラスのインスタンスのインスタンスのポインタ)
* input :network(udpで送信するクラスのインスタンスのポインタ)
* output:void
*/
void outputLog(InvertedPendulumControl *IPD, Network *network) {
  for(int i = 0; i < json_object_list_size; i++ ){
    //データの収集
    json_object_list[i].value = IPD->GetKeyInfo(json_object_list[i].id);
    //Jsonのデータ構造体を更新
    doc[json_object_list[i].id] = json_object_list[i].value;
  }
  //出力
  serializeJson(doc, logData);
  network->SendPacket(logData, strlen(logData));
  
  printf("CSendPacket\n");
}
