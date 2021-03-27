[English](README.md) | Japanese  
# BalanceCar
BalaceCarはM5StickCによる2輪倒立振子ロボットを実現することができます。  
[BalanceCarMonitor](https://github.com/tes2840/BalanceCarMonitor)を合わせて使用することでネットワーク経由で操縦、チューニング、モニタリングすることができます。   

# DEMO
## 倒立デモ
![BalanceCar_Start](https://github.com/tes2840/BalanceCar/wiki/images/BalanceCar_Start.gif)　　

[BalanceCarMonitor](https://github.com/tes2840/BalanceCarMonitor)を使用することで以下のことが可能です。  
## 操縦デモ
BalanceCarを十字キーで操縦できます。  
十字キーの同時入力で、斜め方向への移動も可能です。  
![demo_operation](https://github.com/tes2840/BalanceCarMonitor/wiki/images/BalanceCarMonitor_Demo_Operation.gif)

## 設定デモ
BalanceCarの内部チューニングパラメータの設定が可能です。  
パラメータを変更したいときは、M5StickCへソフトを書き込みする必要はなく、[BalanceCarMonitor](https://github.com/tes2840/BalanceCarMonitor)で設定可能です。  
<img src="https://github.com/tes2840/BalanceCarMonitor/wiki/images/BalanceCarMonitor_Demo_Setting.png" width="540">

## 測定デモ
BalanceCarの内部状態が確認可能です。これにより、パラメータチューニングがしやすくなります。  
<img src="https://github.com/tes2840/BalanceCarMonitor/wiki/images/BalanceCarMonitor_Demo_Mesurement.gif" width="540">
 
# Requirement
## Libraries
動作には以下のライブラリが必要です。    
https://github.com/m5stack/M5StickC  
https://github.com/bblanchon/ArduinoJson  

## Equipment
[BALA-C](https://m5stack-store.myshopify.com/products/bala-c-esp32-development-mini-self-balancing-car)が必要です。  
<img src="https://cdn.shopify.com/s/files/1/0056/7689/2250/products/3_83d2543a-6ece-4edd-9f34-973f55dbfbbd_1200x1200.jpg?v=1590970987" width="270">

# Usage
このリポジトリをクローンし、フォルダの中に移動します。  
```bash
git clone https://github.com/tes2840/BalanceCar
cd BalanceCar
```
Config.hをご自身のネットワーク環境に合わせて設定してください。  

```cpp:Config.h
//Network setting.
const char SSID[] = "";                             // Wi-Fi SSID name.
const char PASSWORD[] = "";                         // Wi-Fi password.
const byte BALAC_IP_ADDR[4] = {192, 168, 11, 101};  // IP Address of BalanceCar.
const byte BALAC_GATEWAY[4] = {192, 168, 11, 1};    // Gateway of BalanceCar.
const byte BALAC_SUBNET[4] = {255, 255, 255, 0};    // Subnetmask of BalanceCar.
const int BALAC_PORT = 50008;                       // Port of BalanceCar.
const byte PC_ADDR[] = {192, 168, 11, 100};         // IP address of the data destination PC.
const int PC_PORT = 50007;                          // Port of the data destination PC.
```

ビルドし、M5StickCに書き込みます。  
BALA-CのSwitchをON方向に倒します(LEDが点灯します)。  
BALA-Cを横に倒した状態でButtonBを長押しします。   
<img src="https://github.com/tes2840/BalanceCar/wiki/images/BalanceCar_functionButton.png" width="270">  

M5StickCのLEDが点灯し、画面に"Stand"が表示されるまで待ちます。  
![BalanceCar_Init](https://github.com/tes2840/BalanceCar/wiki/images/BalanceCar_Init.gif)  

"Stand"が表示されたら、BALA-Cを倒立状態で維持し、LEDが再度点灯するまで待ちます。  
LEDが点灯されると、倒立制御が開始されます。  
![BalanceCar_Start](https://github.com/tes2840/BalanceCar/wiki/images/BalanceCar_Start.gif)

# Note
以下のソースコードを使用させて頂きました。  
https://github.com/m5stack/M5-ProductExampleCodes/tree/master/App/BalaC/Arduino/Balac

 
# Author
github : [tes2840](https://github.com/tes2840/)  
Twitter : [@tes2840](https://twitter.com/tes2840)
 
# License
BalanceCarは[MIT license](https://en.wikipedia.org/wiki/MIT_License)に準拠します。  