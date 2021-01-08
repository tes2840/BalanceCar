#ifndef INVERTED_PENDULUM_CONTROL_H
#define INVERTED_PENDULUM_CONTROL_H

#include <M5StickC.h>
#include "Global.h"

//情報キー
extern const char *KEY_TIME;
extern const char *KEY_POWER_R;
extern const char *KEY_POWER_L;
extern const char *KEY_GYRO_OFFSET_X;
extern const char *KEY_GYRO_OFFSET_Y;
extern const char *KEY_GYRO_OFFSET_Z;
extern const char *KEY_ACC_OFFSET_X;
extern const char *KEY_GYRO_X;
extern const char *KEY_GYRO_Y;
extern const char *KEY_GYRO_Z;
extern const char *KEY_ACC_X;
extern const char *KEY_ACC_Z;
extern const char *KEY_AVE_ACC_Z;
extern const char *KEY_AVE_ABS_OMG;

class InvertedPendulumControl {
  public:
    InvertedPendulumControl();
    ~InvertedPendulumControl();
    void run();
    
    float GetKeyInfo(const char key[]);
    float GetTime();
    float GetpowerR();
    float GetpowerL();
    float GetGyroOffsetX();
    float GetGyroOffsetY();
    float GetGyroOffsetZ();
    float GetAccOffsetX();
    float GetGyroX();
    float GetGyroY();
    float GetGyroZ();
    float GetAccX();
    float GetAccY();
    float GetAveAccZ();
    float GetAveAbsOmg();

   private:
    void calib1();
    void calib2();
    void checkButtonP();
    void calDelay();
    void calDelay(int n);
    void setMode(bool inc);
    void startDemo();
    void resetPara();
    void getGyro();
    void readGyro();
    void drive();
    void drvMotorL(int16_t pwm);
    void drvMotorR(int16_t pwm);
    void drvMotor(byte ch, int8_t sp);
    void resetMotor();
    void resetVar();
    void sendStatus ();
    void imuInit();
    void dispBatVolt();
    
  private:
    boolean serialMonitor=true;
    boolean standing=false;
    int16_t counter=0;
    uint32_t time0=0, time1=0;
    int16_t counterOverPwr=0, maxOvp=20;
    float power, powerR, powerL, yawPower;
    float varAng, varOmg, varSpd, varDst, varIang;
    float gyroXoffset, gyroYoffset, gyroZoffset, accXoffset;//センサーのドリフトを覗くためのオフセット
    float gyroXdata, gyroYdata, gyroZdata;                  //ジャイロセンサーの取得値[deg/s]
    float accXdata, accZdata;                               //加速度センサの取得値[G(9.8m/s^2)]
    float aveAccX=0.0, aveAccZ=0.0, aveAbsOmg=0.0;
    float cutoff=0.1; //~=2 * pi * f (Hz) 
    const float clk = 0.01; // in sec,
    const uint32_t interval = (uint32_t) (clk*1000);  // in msec
    float Kang;//PID制御の角度の比例ゲイン
    float Komg;//PID制御の角度の微分ゲイン(角速度)
    float KIang;//PID制御の角度の積分ゲイン
    float Kyaw;//PID制御
    float Kdst;//距離ゲイン(距離ゲインを上げていくと、移動し続けるのをやめ、大きく前後移動しながらバランスをとる)
    float Kspd;//速度ゲイン(速度ゲインを上げていくと、大きく前後移動するのをやめ、その場でバランスを取る)
    int16_t maxPwr;
    float yawAngle=0.0;
    float moveDestination, moveTarget;
    float moveRate=0.0;
    const float moveStep=0.2*clk;
    int16_t fbBalance=0;
    int16_t motorDeadband=0;
    float mechFactR, mechFactL;
    int8_t motorRDir=0, motorLDir=0;
    bool spinContinuous=false;
    float spinDest, spinTarget, spinFact=1.0;
    float spinStep=0.0; //deg per 10msec
    int16_t ipowerL=0, ipowerR=0;
    int16_t motorLdir=0, motorRdir=0; //0:stop 1:+ -1:-
    float vBatt, voltAve=3.7;
    int16_t punchPwr, punchPwr2, punchDur, punchCountL=0, punchCountR=0;//サーボーモーター初動の動き出しが早くなるようための変数
    byte demoMode=0;
};

#endif //INVERTED_PENDULUM_CONTROL_H
