#ifndef NETWORK_H
#define NETWORK_H

#include <M5StickC.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <ArduinoJson.h>

#include "Global.h"

class Network {
  public:
    Network();
    ~Network();
    
    void SendPacket(const char *buff, const unsigned int len);
    
  private:
    /* Wi-Fi */
    const char *m_ssid = "";      //WiFIのSSIDを入力
    const char *m_pass = "";      // WiFiのパスワードを入力

    WiFiUDP m_wifiUdp; 
    const char *m_pc_addr = "";   //送信先のIPアドレス;
    const int m_pc_port = 50007;  //送信先のポート
    const int m_my_port = 50008;  //自身のポート
};

#endif //NETWORK_H
