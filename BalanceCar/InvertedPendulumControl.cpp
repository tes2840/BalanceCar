#include "InvertedPendulumControl.h"

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

//情報取得用関数
typedef float (InvertedPendulumControl::*p_func)(void);
struct info_struct{
  const char *id;   //id
  p_func func_ptr;  //idの情報を取得する関数のポインタ
};

info_struct info_table[] = {
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
const unsigned int info_table_size = sizeof(info_table) / sizeof(info_table[0]);

InvertedPendulumControl::InvertedPendulumControl(){
#ifdef _DEBUG
  printf("Call Constuctor InvertedPendulumControl Class\n");
#endif
  
  imuInit();
  resetMotor();
  resetPara();
  resetVar();
  calib1();
  #ifdef DEBUG
  debugSetup();
  #else
  setMode(false);
  #endif
}

InvertedPendulumControl::~InvertedPendulumControl(){
#ifdef _DEBUG
  printf("Call Destructor InvertedPendulumControl Class\n");
#endif
}

void InvertedPendulumControl::run(){
  checkButtonP(); //Powerボタンの確認
  getGyro();      //ジャイロセンサから角速度を取得し角度を推定する

  if (!standing) {  //倒立制御開始していない
    dispBatVolt();

    //本体の角度を算出
    aveAbsOmg = aveAbsOmg * 0.9 + abs(varOmg) * 0.1;    //LPF(差分方程式),係数は0.9
    aveAccZ = aveAccZ * 0.9 + accZdata * 0.1;           //LPF(差分方程式),係数は0.9
    M5.Lcd.setCursor(10,130);
    M5.Lcd.printf("%5.2f   ", -aveAccZ);
    if (abs(aveAccZ)>0.9 && aveAbsOmg<1.5) {            //倒立状態の判定(aveAccZは地面と垂直に近づくと1に近づく、aveAbsOmgは垂直になっている時安定しているかを見ている)
      calib2();
      if (demoMode==1) startDemo();
      standing=true;
    }
  }
  else {          //倒立制御開始している
    if (abs(varAng)>30.0 || counterOverPwr>maxOvp) {    //転倒した
      resetMotor();
      resetVar();
      standing=false;
      setMode(false);
    }
    else {                                              //倒立状態
      drive();
    }
  }

  //画面の表示、logのシリアル出力
  counter += 1;
  if (counter >= 100) {
    counter = 0;
    dispBatVolt();
    if (serialMonitor) sendStatus();
  }
  do time1 = millis();
  while (time1 - time0 < interval);
  time0 = time1;
}

/**
 * @brief ジャイロセンサ(Y軸)のオフセットを算出する
 * @details ジャイロセンサは静止状態でも変動するバイアスが乗る(ドリフト)
 *          そのバイアスを取り除くためジャイロセンサーの値のオフセットを算出する(一定時間の平均値)
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::calib1() {
  calDelay(30);
  digitalWrite(LED, LOW);
  calDelay(80);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, LCDV_MID);
  M5.Lcd.print(" Cal-1  ");

  //ジャイロセンサの値をN_CAL1回取得しその平均値をY軸のオフセットとする
  gyroYoffset=0.0;
  for (int i=0; i <N_CAL1; i++){
    readGyro();
    gyroYoffset += gyroYdata;
    delay(9);
  }
  gyroYoffset /= (float)N_CAL1;

  M5.Lcd.fillScreen(BLACK);
  digitalWrite(LED, HIGH);
}

/**
 * @brief 倒立状態でのジャイロセンサ(Z軸)、加速度センサ(X軸)のオフセットを算出する
 * @details ジャイロセンサ、加速度センサは静止状態でも変動するバイアスが乗る(ドリフト)
 *          そのバイアスを取り除くためオフセットを算出する(一定時間の平均値)
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::calib2() {
  //状態のリセット(offsetは除く)
  resetVar();
  resetMotor();

  //calib2スタートの表示
  digitalWrite(LED, LOW);
  calDelay(80);
  M5.Lcd.setCursor(0, LCDV_MID);
  M5.Lcd.println(" Cal-2  ");

  //オフセットの算出
  accXoffset=0.0;
  gyroZoffset=0.0;
  for (int i=0; i <N_CAL2; i++){
    readGyro();
    accXoffset += accXdata;
    gyroZoffset += gyroZdata;
    delay(9);
  }
  accXoffset /= (float)N_CAL2;
  gyroZoffset /= (float)N_CAL2;

  //calib2の終了を表示
  M5.Lcd.fillScreen(BLACK);
  digitalWrite(LED, HIGH);
}

/**
 * @brief Powerボタン押下を確認しモードの切り替えを行う
 * @details short pushではcalib1を開始
 *          long push(1.5s)ではでもモードへ切り替え
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::checkButtonP() {
  byte pbtn=M5.Axp.GetBtnPress();
  if (pbtn==2) calib1(); //short push
  else if (pbtn==1) setMode(true); //long push
}

void InvertedPendulumControl::calDelay(int n) {
  for (int i=0; i<n; i++) {
    getGyro();
    delay(9);
  }
}

/**
 * @brief Mode選択
 * @param (inc) モード(false:倒立制御,true:デモ)
 * @return void
 */
void InvertedPendulumControl::setMode(bool inc) {
  if (inc) demoMode=++demoMode%2;
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(5, 5);
  if (demoMode==0) M5.Lcd.print("Stand ");
  else if (demoMode==1) M5.Lcd.print("Demo ");
}

/**
 * @brief デモモードの開始
 * @details その場で回転する
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::startDemo() {
  moveRate=1.0;
  spinContinuous=true;
  spinStep=-40.0*clk;
}

/**
 * @brief パラメータの初期化
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::resetPara() {
  Kang=37.0;
  Komg=0.84;
  KIang=800.0;
  Kyaw=4.0;
  Kdst=85.0;
  Kspd=2.7;
  mechFactL=0.45;
  mechFactR=0.45;
  punchPwr=20;    //サーボーモーター初動の動き出しが早くなるような操作量のオフセット値
  punchDur=1;     //サーボーモーター書道のパンチングの継続ステップ
  fbBalance=-3;
  motorDeadband=10;
  maxPwr=120;
  punchPwr2=max(punchPwr, motorDeadband);////サーボーモーター初動の動き出しが早くなるような操作量のオフセット値(モーターのデッドバンドでリミット処理)
}

/**
 * @brief ジャイロセンサから角速度を取得し角度を推定する
 * @param (void) void
 * @return void
 * @note 
 * 相補フィルターを使用して角度を算出する(ジャイロセンサー、加速度センサーから算出した角度を使用する)
 * ①ジャイロセンサーから推定した角度はLPFを通す(ジャイロセンサーで取得される値は常に微小な変化(ノイズ)が乗るためそれを取り除く)
 * ②加速度センサから取得された角度はHPFを通す(加速度センサーは重力加速度の影響を受けるためその成分を取り除く)
 * 角度 =①+②
 *     =θ(n-1)+θgΔt-ωθ(n-1)Δt+ωθaΔt
 * θ(n-1):前回の角度、θg:ジャイロセンサの角速度、θa:加速度センサの角度、Δt:サンプリング時間、ω=2πfc:角周波数、fc:カットオフ周波数
 * θa:加速度センサの角度の求め方
 * 加速度センサーの角度が90degのとき、accXdataは1のため加速度センサ(AccXdata)*57.3をかけることで算出している
 * 参考:https://www2.himdx.net/mcr/product/download/2gyro_3acc_module_c_language_headstand_control_program_explanation_manual.pdf
 */
void InvertedPendulumControl::getGyro() {
  readGyro();
  //ジャイロセンサーから角速度、ヨー角を求める
  varOmg = (gyroYdata-gyroYoffset);           //ピッチ角速度[deg/sec]
  yawAngle += (gyroZdata-gyroZoffset) * clk;  //ヨー角速度[deg/s] * 経過時間(clock)[s] = ヨー角[deg]

  //相補フィルタにより角度を算出する
  varAng += (varOmg + ((accXdata-accXoffset) * 57.3 - varAng) * cutoff ) * clk; //complementary filter
}

/**
 * @brief ジャイロセンサ、加速度センサから値を取得する
 *        加速度センサ：ローカル座標からグローバル座標に変換している(M5StickCが地面と垂直状態のとき上方向をz,全身方向をx)
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::readGyro() {
  float gX, gY, gZ, aX, aY, aZ;
  M5.Imu.getGyroData(&gX,&gY,&gZ);
  M5.Imu.getAccelData(&aX,&aY,&aZ);
  gyroYdata=gX;
  gyroZdata=-gY;
  gyroXdata=-gZ;
  accXdata=aZ;
  accZdata=aY;
}

/**
 * @brief モーター制御
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::drive() {
  
  //旋回するか確認
  if (abs(moveRate)>0.1){
    spinFact=constrain(-(powerR+powerL)/10.0, -1.0, 1.0); //moving
  } else {
    spinFact=1.0; //standing
  }

  //継続して旋回する確認
  if (spinContinuous){
    //継続して回転する場合は、タイヤの目標回転量を算出
    spinTarget += spinStep * spinFact;
  } else {
    //継続して旋回しない場合は、目標回転角(spinDest)を維持するようにする
     if (spinTarget < spinDest) spinTarget += spinStep;
     if (spinTarget > spinDest) spinTarget -= spinStep;
  }

  //PIDでモーターをPWM制御
  moveTarget += moveStep * (moveRate +(float)fbBalance/100.0);                        //目標移動距離[m]
  varSpd += power * clk;                                                              //速度[m/s](モーターに加えるpwmのデューテ比)
  varDst += Kdst * (varSpd * clk -moveTarget);                                        //距離[m](モーターに加えるPWMの積分値)
  varIang += KIang * varAng * clk;                                                    //角度の積分要素

  //角度、距離、速度、角速度の係数をかけたものを足し合わせて操作量を求める
  power = varIang + varDst + (Kspd * varSpd) + (Kang * varAng) + (Komg * varOmg);

  //モーター制御の操作量が許容よりも大きいかカウントする
  if (abs(power) > 1000.0){
    counterOverPwr += 1;
  } else {
    counterOverPwr =0;
  }

  //倒立を維持できない制御状態だった場合モーターの制御はしない(→いずれ転倒する)
  if (counterOverPwr > maxOvp) return;
  
  //その場で旋回するためヨーの操作量を算出
  power = constrain(power, -maxPwr, maxPwr);
  yawPower = (yawAngle - spinTarget) * Kyaw;  //ヨーの操作量を算出(現状のヨー角と目標回転角の差を算出し、ヨーの比例ゲインをかける)
  //ヨーの操作量を片方に上乗せし、片方からは引くことでトータルの操作量を維持する(→その場で旋回するようになる)
  powerR = power - yawPower;
  powerL = power + yawPower;
  
  int16_t mdbn=-motorDeadband;
  int16_t pp2n=-punchPwr2;

  //左モーターの制御処理
  //mechの実力に合わせて出力を調整(mechFactL)
  ipowerL = (int16_t) constrain(powerL * mechFactL, -maxPwr, maxPwr);
  if (ipowerL > 0) { 
    //前進
    if (motorLdir == 1){//前回からすでに前進状態
      punchCountL = constrain(++punchCountL, 0, 100);
    } else {
      punchCountL=0;
    }

    if (punchCountL<punchDur) {//モーターの動き出し
      //サーボーモーター初動の動き出しが早くなるように操作量をpuchPwr2でリミット処理
      drvMotorL(max(ipowerL, punchPwr2));
    } else {
      //モーター制御には不感帯があるため最低操作量以上の出力にする
      drvMotorL(max(ipowerL, motorDeadband));
    }
    //モーターの状態を更新
    motorLdir = 1;
  }
  else if (ipowerL < 0) {
    //後退
    if (motorLdir == -1){//前回からすでに後退状態
      punchCountL = constrain(++punchCountL, 0, 100);
    } else {
      punchCountL=0;
    }

    if (punchCountL<punchDur) {//モーターの動き出し
      //サーボーモーター初動の動き出しが早くなるように操作量をpp2nでリミット処理(pp2nは負なのでminでリミット処理)
      drvMotorL(min(ipowerL, pp2n));
    } else {
      //モーター制御には不感帯があるため最低操作量以上の出力にする(mdbnは負なのでminでリミット処理する)
      drvMotorL(min(ipowerL, mdbn));
    }
    //モーターの状態を更新
    motorLdir = -1;
  } else {
    //停止
    drvMotorL(0);
    motorLdir = 0;
  }

  //右モーターの制御処理
  ipowerR = (int16_t) constrain(powerR * mechFactR, -maxPwr, maxPwr);
  if (ipowerR > 0) {
    if (motorRdir == 1) punchCountR = constrain(++punchCountR, 0, 100);
    else punchCountR=0;

    if (punchCountR<punchDur) drvMotorR(max(ipowerR, punchPwr2));
    else drvMotorR(max(ipowerR, motorDeadband));

    motorRdir = 1;
  }
  else if (ipowerR < 0) {
    if (motorRdir == -1) punchCountR = constrain(++punchCountR, 0, 100);
    else punchCountR=0;

    if (punchCountR<punchDur) drvMotorR(min(ipowerR, pp2n));
    else drvMotorR(min(ipowerR, mdbn));
    
    motorRdir = -1;
  }
  else {
    drvMotorR(0);
    motorRdir = 0;
  }
}

/**
 * @brief MotorLを回転させる(回転角は-127~127の範囲内に制限する)
 * @param (pwm) 回転角
 * @return void
 */
void InvertedPendulumControl::drvMotorL(int16_t pwm) {
  drvMotor(0, (int8_t)constrain(pwm, -127, 127));
}

/**
 * @brief MotorRを回転させる(回転角は-127~127の範囲内に制限する)
 * @param (pwm) 回転角
 * @return void
 */
void InvertedPendulumControl::drvMotorR(int16_t pwm) {
  drvMotor(1, (int8_t)constrain(-pwm, -127, 127));
}

/**
 * @brief I2C通信でモータドライバ(L9110S)にデータを転送する
 * @param (ch) モーターの番号
 * @param (sp) 回転角
 * @return void
 */
void InvertedPendulumControl::drvMotor(byte ch, int8_t sp) {
  Wire.beginTransmission(0x38);   // Slaveデバイスによってアドレスは固定
  Wire.write(ch);
  Wire.write(sp);
  Wire.endTransmission();
}

/**
 * @brief モーターの初期化
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::resetMotor() {
 drvMotorR(0);
 drvMotorL(0);
 counterOverPwr=0;
}

/**
 * @brief 内部ステータスの初期化
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::resetVar() {
  power=0.0;
  moveTarget=0.0;
  moveRate=0.0;
  spinContinuous=false;
  spinDest=0.0;
  spinTarget=0.0;
  spinStep=0.0;
  yawAngle=0.0;
  varAng=0.0;
  varOmg=0.0;
  varDst=0.0;
  varSpd=0.0;
  varIang=0.0;
}

void InvertedPendulumControl::sendStatus () {
  Serial.print(millis()-time0);
  Serial.print(" stand="); Serial.print(standing);
  Serial.print(" accX="); Serial.print(accXdata);
  Serial.print(" power="); Serial.print(power);
  Serial.print(" ang=");Serial.print(varAng);
  Serial.print(", "); 
  Serial.print(millis()-time0);
  Serial.println();
}

/**
 * @brief IMU(SH200Q）の初期化
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::imuInit() {
  M5.Imu.Init();
  if (M5.Imu.imuType=M5.Imu.IMU_MPU6886) {
    M5.Mpu6886.SetGyroFsr(M5.Mpu6886.GFS_250DPS); //250DPS 500DPS 1000DPS 2000DPS
    M5.Mpu6886.SetAccelFsr(M5.Mpu6886.AFS_4G); //2G 4G 8G 16G
    if (serialMonitor) Serial.println("MPU6886 found");
  }
  else if (serialMonitor) Serial.println("MPU6886 not found");
}

/**
 * @brief DisplayにM5StickCの電圧を表示
 * @param (void) void
 * @return void
 */
void InvertedPendulumControl::dispBatVolt() {
  M5.Lcd.setCursor(5, LCDV_MID);
  vBatt= M5.Axp.GetBatVoltage();
  M5.Lcd.printf("%4.2fv ", vBatt);
}

//情報取得用関数
float InvertedPendulumControl::GetTime(){return float(float(millis())/1000);}  // 秒単位で経過時間を返す
float InvertedPendulumControl::GetpowerR(){return powerR;}
float InvertedPendulumControl::GetpowerL(){return powerL;}
float InvertedPendulumControl::GetGyroOffsetX(){return gyroXoffset;}
float InvertedPendulumControl::GetGyroOffsetY(){return gyroYoffset;}
float InvertedPendulumControl::GetGyroOffsetZ(){return gyroZoffset;}
float InvertedPendulumControl::GetAccOffsetX(){return accXoffset;}
float InvertedPendulumControl::GetGyroX(){return gyroXdata;}
float InvertedPendulumControl::GetGyroY(){return gyroYdata;}
float InvertedPendulumControl::GetGyroZ(){return gyroZdata;}
float InvertedPendulumControl::GetAccX(){return accXdata;}
float InvertedPendulumControl::GetAccY(){return accZdata;}
float InvertedPendulumControl::GetAveAccZ(){return aveAccZ;}
float InvertedPendulumControl::GetAveAbsOmg(){return aveAbsOmg;}

float InvertedPendulumControl::GetKeyInfo(const char key[]){
  float data = 0.0;
  for (int i = 0; i < info_table_size; i++){
    if ( 0 == strcmp(info_table[i].id, key) ){
        data = (this->*info_table[i].func_ptr)();
        break;
    }
  }
  return data;
}
