English | [Japanese](README_jp.md)  
# BalanceCar
If you use BalaceCar, you can make a two-wheeled inverted pendulum robot with M5StickC.  
You can maneuver, tune parameters and monitor it over the network with [BalanceCarMonitor](https://github.com/tes2840/BalanceCarMonitor).  

# DEMO
## Inverted pendulum control demo
![BalanceCar_Start](https://github.com/tes2840/BalanceCar/wiki/images/BalanceCar_Start.gif)　　

By using [BalanceCarMonitor](https://github.com/tes2840/BalanceCarMonitor), you can do the following.  
## Maneuver demo
You can maneuver BalanceCar with the cross key and also move it diagonally by inputting the cross key at the same time.  
![demo_operation](https://github.com/tes2840/BalanceCarMonitor/wiki/images/BalanceCarMonitor_Demo_Operation.gif)

## Setting demo
You can set the tuning parameters of BalanceCar.  
If you want to change the parameters,  you can set it with this tool without write the software to M5StickC.  
<img src="https://github.com/tes2840/BalanceCarMonitor/wiki/images/BalanceCarMonitor_Demo_Setting.png" width="540">

## Measurement demo
You can check the status of BalanceCar. This makes it easier to tune parameters of it.  
<img src="https://github.com/tes2840/BalanceCarMonitor/wiki/images/BalanceCarMonitor_Demo_Mesurement.gif" width="540">
 
# Requirement
## Libraries
BalanaceCar requires the following libraries.  
https://github.com/m5stack/M5StickC  
https://github.com/bblanchon/ArduinoJson  

## Equipment
[BALA-C](https://m5stack-store.myshopify.com/products/bala-c-esp32-development-mini-self-balancing-car) is required.    
<img src="https://cdn.shopify.com/s/files/1/0056/7689/2250/products/3_83d2543a-6ece-4edd-9f34-973f55dbfbbd_1200x1200.jpg?v=1590970987" width="270">

# Usage
Clone this repository and go into it.  
```bash
git clone https://github.com/tes2840/BalanceCar
cd BalanceCar
```
Please configure Config.h to match your network environment.  

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

Build and write to the M5StickC.  
Turn on the switch of BALA-C then the LED lights up.  
Press and hold Button B while BALA-C is tilted sideways.   
<img src="https://github.com/tes2840/BalanceCar/wiki/images/BalanceCar_functionButton.png" width="270">  

Wait until the LED on the M5StickC lights up and "Stand" is displayed on the screen.  
![BalanceCar_Init](https://github.com/tes2840/BalanceCar/wiki/images/BalanceCar_Init.gif)  

When "Stand" is displayed, keep the BALA-C in the inverted position and wait until the LED lights up again.  
When the LED lights up, the inversion control will start.  
![BalanceCar_Start](https://github.com/tes2840/BalanceCar/wiki/images/BalanceCar_Start.gif)

# Note
I have used the following source code.  
https://github.com/m5stack/M5-ProductExampleCodes/tree/master/App/BalaC/Arduino/Balac

 
# Author
github : [tes2840](https://github.com/tes2840/)  
Twitter : [@tes2840](https://twitter.com/tes2840)
 
# License
BalanceCar is under [MIT license](https://en.wikipedia.org/wiki/MIT_License).