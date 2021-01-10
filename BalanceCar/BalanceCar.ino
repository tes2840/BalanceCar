/**
 * @file BalanceCar.ino
 * @brief 2輪で倒立制御するスケッチ for BalaC(M5StickC)
 */

#include <M5StickC.h>
#include "Network.h"
#include "InvertedPendulumControl.h"
#include "Config.h"

//log出力用Buff
char logData[1024] = "";

//受信用buff
char receiveBuff[1024] = "";

/**
 * @fn      void setup()
 * @brief   初期化処理
 * @param   (void) void
 * @return  void
 */
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

/**
 * @fn      void loop()
 * @brief   メイン処理
 * @param   (void) void
 * @return  void
 */
void loop() {
  //グローバルにインスタンスを生成するとリセットするためループ内に宣言する
  static InvertedPendulumControl IPD;
  static Network network;  
  static int counter = 0;
  
  IPD.run();

  counter++;
  //50 -> 0.5秒に一回ログを出力する　動作上もOK 10ms周期に実行されている模様
  if( counter >50 ){
    updateInfo(&IPD, &network);
    counter  = 0;
  }
}

/**
 * @fn      void updateInfo()
 * @brief   情報を更新する
 * @details BalanceCarの情報を取得しPCに送信する
 *          PCからの情報を取得しBalanceCarに設定する
 * @param   (InvertedPendulumControl*) IPD
 * @param   (Network*) network
 * @return  void
 */
void updateInfo(InvertedPendulumControl *IPD, Network *network) {
  // BalanceCarの情報を取得しPCに送信する
  IPD->GetInfo(logData, sizeof(logData));
  network->SendPacket(logData, strlen(logData));

  // PCからの情報を取得しBalanceCarに設定する
  network->ReceivePacket(receiveBuff, sizeof(receiveBuff));
  IPD->SetInfo(receiveBuff);
}
