/**
 * @file Config.h
 * @brief 環境設定用ファイル
 */
#ifndef CONFIG_H
#define CONFIG_H

//#define _DEBUG                                    // Enable Debug output.

#define LED 10                                      // LED pin no.
#define N_CAL1 100                                  // The number of times data is acquired to determine the offset of the gyro sensor for CAL1.
#define N_CAL2 100                                  // The number of times data is acquired to determine the offset of the gyro sensor for CAL2.
#define LCDV_MID 60                                 // Character display position.
#define UPDATE_CYCLE 50                             // Data transmission and reception interval(e.g. 1->0.1ms,50->500ms).

//Network setting.
const char SSID[] = "";                             // Wi-Fi SSID name.
const char PASSWORD[] = "";                         // Wi-Fi password.
const byte BALAC_IP_ADDR[4] = {192, 168, 11, 101};  // IP Address of BalanceCar.
const byte BALAC_GATEWAY[4] = {192, 168, 11, 1};    // Gateway of BalanceCar.
const byte BALAC_SUBNET[4] = {255, 255, 255, 0};    // Subnetmask of BalanceCar.
const int BALAC_PORT = 50008;                       // Port of BalanceCar.
const byte PC_ADDR[] = {192, 168, 11, 100};         // IP address of the data destination PC.
const int PC_PORT = 50007;                          // Port of the data destination PC.

#endif //CONFIG_H
