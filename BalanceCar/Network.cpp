#include "Network.h"

Network::Network(void){
  WiFi.begin(m_ssid, m_pass);
  
  while( WiFi.status() != WL_CONNECTED) {
    delay(500); 
  }
#ifdef _DEBUG
  char IP[] = "xxx.xxx.xxx.xxx";
  IPAddress ip = WiFi.localIP();
  ip.toString().toCharArray(IP, 16);
  printf("WiFi Local IP Address is %s\n", IP);
  //M5.Lcd.println(WiFi.localIP());
#endif
  
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
  m_wifiUdp.endPacket();
}
