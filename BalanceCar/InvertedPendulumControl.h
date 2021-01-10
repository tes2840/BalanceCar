/**
 * @file InvertedPendulumControl.h
 */
#ifndef INVERTED_PENDULUM_CONTROL_H
#define INVERTED_PENDULUM_CONTROL_H

#include <M5StickC.h>
#include <ArduinoJson.h>
#include "Config.h"

#define JSON_CAPACITY 1024    //JSONを保存する最大サイズ(byte)

enum OPERATION{
  SET_PARAM = 0,  // Balacの設定変更
  SET_OPE,    // Balacの移動操作
};

/**
 * @class InvertedPendulumControl
 * @brief 2輪の倒立制御クラス
 */
class InvertedPendulumControl {
  public:
    InvertedPendulumControl();
    ~InvertedPendulumControl();
    void run();
    
    int GetInfo(char *buff, const unsigned int buff_size);
    void SetInfo(char *json);
    float GetKeyInfo(const char key[]);

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

    // 情報取得関数
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

    // 情報設定関数
    void SetKang(void *pValue);
    void SetKomg(void *pValue);
    void SetKIang(void *pValue);
    void SetKyaw(void *pValue);
    void SetKdst(void *pValue);
    void SetKspd(void *pValue);
    void SetKspin(void *pValue);
    void SetTurnRight(void *pValue);
    void SetTurnLeft(void *pValue);
    void SetFoward(void *pValue);
    void SetBackward(void *pValue);
    void SetStand(void *pValue);
    
  private:
    /* 内部状態 */
    boolean serialMonitor=true;
    boolean standing=false;
    byte demoMode=0;
    int16_t counterOverPwr=0, maxOvp=20;
    float power, powerR, powerL;
    float varAng, varOmg, varSpd, varDst, varIang;
    float gyroXoffset, gyroYoffset, gyroZoffset, accXoffset;  //センサーのドリフトを取り除くためのオフセット
    float gyroXdata, gyroYdata, gyroZdata;                    //ジャイロセンサーの取得値[deg/s]
    float accXdata, accZdata;                                 //加速度センサの取得値[G(9.8m/s^2)]
    float aveAccX=0.0, aveAccZ=0.0, aveAbsOmg=0.0;
    float cutoff=0.1;                                         //~=2 * pi * f (Hz) 
    const float clk = 0.01;                                   // in sec,
    const uint32_t interval = (uint32_t) (clk*1000);          // in msec
    int16_t maxPwr;
    float yawAngle=0.0;
    float moveTarget;
    float moveRate=0.0;
    int16_t fbBalance=0;
    int16_t motorDeadband=0;
    float mechFactR, mechFactL;
    int8_t motorRDir=0, motorLDir=0;
    bool spinContinuous=false;
    float spinDest, spinTarget=1.0;
    float spinStep=0.0;                                       //deg per 10msec
    int16_t motorLdir=0, motorRdir=0;                         //0:stop 1:+ -1:-

    /* log出力用 */
    int16_t counter=0;
    uint32_t time0=0, time1=0;

    /* サーボーモーター初動の動き出し調整用パラメータ */
    int16_t punchPwr, punchPwr2, punchDur, punchCountL=0, punchCountR=0;

    /* PID制御調整用パラメータ */
    float Kang;   //PID制御の角度の比例ゲイン
    float Komg;   //PID制御の角度の微分ゲイン(角速度)
    float KIang;  //PID制御の角度の積分ゲイン
    float Kyaw;   //ヨー角の比例ゲイン
    float Kdst;   //距離ゲイン(距離ゲインを上げていくと、移動し続けるのをやめ、大きく前後移動しながらバランスをとる) */
    float Kspd;   //速度ゲイン(速度ゲインを上げていくと、大きく前後移動するのをやめ、その場でバランスを取る)
    float Kspin;  //旋回角度のゲイン
    
    /* 情報取得用変数 */
    // Json document obj
    StaticJsonDocument<JSON_CAPACITY> doc;

    // 関数ポインタ
    typedef float (InvertedPendulumControl::*p_read_func)();

    //キーと対応する情報取得関数の構造体
    struct info_struct{
      const char *id;   //id
      p_read_func func_ptr;  //idの情報を取得する関数のポインタ
    };

    //情報キー
    const char *KEY_TIME = "time";
    const char *KEY_POWER_R = "power_R";
    const char *KEY_POWER_L = "power_L";
    const char *KEY_GYRO_OFFSET_X ="gyro_offset_X";
    const char *KEY_GYRO_OFFSET_Y ="gyro_offset_Y";
    const char *KEY_GYRO_OFFSET_Z ="gyro_offset_Z";
    const char *KEY_ACC_OFFSET_X ="acc_offset_x";
    const char *KEY_GYRO_X ="gyro_X";
    const char *KEY_GYRO_Y ="gyro_Y";
    const char *KEY_GYRO_Z ="gyro_Z";
    const char *KEY_ACC_X ="acc_X";
    const char *KEY_ACC_Z ="acc_Z";
    const char *KEY_AVE_ACC_Z="aveAccZ";
    const char *KEY_AVE_ABS_OMG="aveAbsOmg";

    //キーに対応する情報取得関数のテーブルのサイズ
    static const unsigned int info_table_size = 14;
    //キーに対応する情報取得関数のテーブル
    info_struct info_table[info_table_size] = {
      { KEY_TIME,           &InvertedPendulumControl::GetTime },
      { KEY_POWER_R,        &InvertedPendulumControl::GetpowerR },
      { KEY_POWER_L,        &InvertedPendulumControl::GetpowerL },
      { KEY_GYRO_OFFSET_X,  &InvertedPendulumControl::GetGyroOffsetX },
      { KEY_GYRO_OFFSET_Y,  &InvertedPendulumControl::GetGyroOffsetY },
      { KEY_GYRO_OFFSET_Z,  &InvertedPendulumControl::GetGyroOffsetZ },
      { KEY_ACC_OFFSET_X,   &InvertedPendulumControl::GetAccOffsetX },
      { KEY_GYRO_X,         &InvertedPendulumControl::GetGyroX },
      { KEY_GYRO_Y,         &InvertedPendulumControl::GetGyroY },
      { KEY_GYRO_Z,         &InvertedPendulumControl::GetGyroZ },
      { KEY_ACC_X,          &InvertedPendulumControl::GetAccX },
      { KEY_ACC_Z,          &InvertedPendulumControl::GetAccY },
      { KEY_AVE_ACC_Z,      &InvertedPendulumControl::GetAveAccZ },
      { KEY_AVE_ABS_OMG,    &InvertedPendulumControl::GetAveAbsOmg }
    };

    /* 情報設定用変数 */
    StaticJsonDocument<JSON_CAPACITY> m_readDoc;

    // 関数ポインタ
    typedef void (InvertedPendulumControl::*p_set_func)(void*);

    // キーと対応する情報取得関数の構造体
    struct set_struct{
      const char *id;   //id
      OPERATION ope;    //操作種別
      p_set_func func_ptr;       //パラメータに設定するデータ
    };

    // 情報キー
    const char *KEY_K_ANG = "Kang";
    const char *KEY_K_OMG = "Komg";
    const char *KEY_KI_ANG = "KIang";
    const char *KEY_K_YAW = "Kyaw";
    const char *KEY_K_DST = "Kdst";
    const char *KEY_K_SPD = "Kspd";
    const char *KEY_K_SPIN = "Kspin";
    const char *KEY_OPE_RIGHT = "right";
    const char *KEY_OPE_LEFT = "left";
    const char *KEY_OPE_FOWARD = "foward";
    const char *KEY_OPE_BACKWARD = "backward";
    const char *KEY_OPE_STAND = "stand";

    //キーに対応する情報取得関数のテーブルのサイズ
    static const unsigned int set_table_size = 12;

    //キーに対するパラメータのテーブル
    set_struct set_table[set_table_size] = {
      { KEY_K_ANG,        SET_PARAM,  &InvertedPendulumControl::SetKang },
      { KEY_K_OMG,        SET_PARAM,  &InvertedPendulumControl::SetKomg },
      { KEY_KI_ANG,       SET_PARAM,  &InvertedPendulumControl::SetKIang },
      { KEY_K_YAW,        SET_PARAM,  &InvertedPendulumControl::SetKyaw },
      { KEY_K_DST,        SET_PARAM,  &InvertedPendulumControl::SetKdst },
      { KEY_K_SPD,        SET_PARAM,  &InvertedPendulumControl::SetKspd },
      { KEY_K_SPIN,       SET_PARAM,  &InvertedPendulumControl::SetKspin },
      { KEY_OPE_RIGHT,    SET_OPE,    &InvertedPendulumControl::SetTurnRight },
      { KEY_OPE_LEFT,     SET_OPE,    &InvertedPendulumControl::SetTurnLeft },
      { KEY_OPE_FOWARD,   SET_OPE,    &InvertedPendulumControl::SetFoward },
      { KEY_OPE_BACKWARD, SET_OPE,    &InvertedPendulumControl::SetBackward },
      { KEY_OPE_STAND,    SET_OPE,    &InvertedPendulumControl::SetStand }
    };
};

#endif //INVERTED_PENDULUM_CONTROL_H
