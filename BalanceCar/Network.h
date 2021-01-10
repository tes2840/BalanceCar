/**
 * @file Network.h
 */
#ifndef NETWORK_H
#define NETWORK_H

#include <M5StickC.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include "Config.h"

class Network {
  public:
    Network();
    ~Network();
    
    void SendPacket(const char *buff, const unsigned int len);
    int ReceivePacket(char *buff, const unsigned int len);
    
  private:
    /* Wi-Fi */
    const char *m_ssid = SSID;             // Wi-FiのSSIDを入力
    const char *m_pass = PASSWORD;         // WiFiのパスワードを入力
    const byte *m_ip = BALAC_IP_ADDR;      // Fixed IP Address
    const byte *m_gateway = BALAC_GATEWAY; // Gateway
    const byte *m_subnet = BALAC_SUBNET;   // Subnetmask
    const int m_my_port = BALAC_PORT;      // Port

    WiFiUDP m_wifiUdp;
    const byte *m_pc_addr = PC_ADDR;      // 送信先のIPアドレス
    const int m_pc_port = PC_PORT;        // 送信先のポート
};

#endif //NETWORK_H
