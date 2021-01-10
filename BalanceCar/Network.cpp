/**
 * @file Network.cpp
 */
#include "Network.h"

Network::Network(void){
  if (!WiFi.config(m_ip, m_gateway, m_subnet)){
    Serial.println("Failed to configure!");
  }   // Set fixed IP address
  WiFi.begin(m_ssid, m_pass);
  
  while( WiFi.status() != WL_CONNECTED) {
    delay(500); 
  }
  Serial.print(WiFi.localIP());
  m_wifiUdp.begin(m_my_port);
}

Network::~Network(void){
  //WiFi1の切断
  WiFi.disconnect();
#ifdef _DEBUG
  printf("Call Destructor Netowrk Class\n");
#endif
}

void Network::SendPacket(const char *buff, const unsigned int len)
{
  m_wifiUdp.beginPacket(m_pc_addr, m_pc_port);
  for(int i=0;i<len;i++){
    // データ送信
    m_wifiUdp.write(buff[i]);
  }
#ifdef _DEBUG
  //Serial.println(buff);
#endif
  m_wifiUdp.endPacket();
}

/**
 * @fn Network::ReceivePacket
 * @brief 
 * @param buff 
 * @param len 
 * @return int(0:No error, 1:no receive, -1:buff over)
 */
int Network::ReceivePacket(char *buff, const unsigned int len)
{
  int packetSize = m_wifiUdp.parsePacket();
#ifdef _DEBUG
  Serial.print("packetsize:");Serial.println(packetSize);
#endif

  if (packetSize > 0){
    if(packetSize <= len){
      memset(buff, 0x00, len);
      int len = m_wifiUdp.read(buff, packetSize);
#ifdef _DEBUG
      Serial.print("receive data:");Serial.println(buff);
#endif
    }else{
      return -1;      // receive buff over
    }

  }else{
    return 1;         // no receive
  }

  return 0;
}

