#include <Arduino.h>

// microros definition =====================================================

#include "micro_ros_arduino_simpler/simpler_base.h"

//#include <custom_msgs/msg/commands.h>
//#include <sensor_msgs/msg/joy.h>
#include <custom_msgs/msg/commands.h>
#include <custom_msgs/msg/electric_suspension_pwm.h>
#include <custom_msgs/msg/electric_suspension_state.h>
#include <custom_msgs/msg/limit_switch.h>

extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;

rcl_publisher_t estate_publisher;
rcl_publisher_t ls_publisher;
rcl_timer_t estate_timer;
//rcl_timer_t ls_timer;
custom_msgs__msg__ElectricSuspensionState msg_estate;
custom_msgs__msg__LimitSwitch msg_ls;

rcl_subscription_t cmd_subscriber;
rcl_subscription_t espwm_subscriber;
custom_msgs__msg__Commands msg_cmd;
//sensor_msgs__msg__Joy msg_joy;
custom_msgs__msg__ElectricSuspensionPwm msg_espwm;

// ==========================================================================

// NITK.K Libraries =========================================================

#include <MD.h>
#include <MDC.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

TwoWire Wire1;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

MD mt_crawler_left(PC11,PC10,PC9);
MD mt_crawler_right(PG1,PF9,PF7);
MD mt_rf(PA6,PD14,PD15);
MD mt_rfad(PE11,PF14,PE13);
MD mt_decon(PF15,PG9,PE8);
MDC mdc(PF0,PF1);

// ==========================================================================

// Other Libraries ==========================================================

// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
#include <STM32TimerInterrupt.h>

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
STM32Timer ITimer10(TIM10);

// ==========================================================================

// Setup Variables for callbacks ============================================

#define CMD_MAX 34000.0f
#define PITCH_RADIUS 15.0f
#define CABLE_RADIUS 20.2f
#define MCNM_RADIUS  100.0f
#define HEIGHT_GAIN 0.08f
#define PITCH_GAIN  0.04f
#define ES_DUTY_MAX 0.2f
#define ES_DUTY_MIN -0.2f
#define ES_HOR_DUTY_MAX 0.4f
#define ES_HOR_DUTY_MIN -0.4f
#define ES_MINUS_DUTY -0.05f
#define MCNM_DUTY_MAX 0.15f
#define MCNM_DUTY_MIN -0.15f
#define CRQL_DUTY_MAX 0.95f
#define CRQL_DUTY_MIN -0.95f

//limit switch
#define ES0     0
#define ES1     1
#define ES2     2
#define ES3     3
#define RF_UP   4
#define RF_DN   5
#define RFAD_UP 6
#define RFAD_DN 7
#define DECON_L 8
#define DECON_R 9

//cmd
#define LEFT_X  0
#define LEFT_Y  1
#define RIGHT   2
#define DECON   3
#define RF_PWM  4
#define ES_HTF  5
#define ES_HTB  6

#define MC_MODE 0
#define ES_MODE 1
#define RF_MODE 2
#define CABLE   3

//target_height
#define FRONT 0
#define BACK  1

// MDC I2C addresses
int es_mdc_ids[4] = {0x10,0x11,0x12,0x13};  //off-off-x-x
int mt_mdc_ids[4] = {0x14,0x15,0x16,0x17};  //on-off-x-x

// ES angle
float mt_angle[4] = {0.0f};
float mt_angle_diff[2] = {0.0f};

// duty
float mt_pwm[4] = {0.0f};
float cr_pwm[2] = {0.0f};

// ES target height
int16_t target_height[4];
int16_t pre_target_height[4];

unsigned long s_time;

//cmd_callback
custom_msgs__msg__Commands cmd;

//espwm_callback
float es_pwm[4] = {0.0};

//ls_callback
int ls_pins[10] = {PD12,PD11,PA0,PE0,PB10,PE15,PE14,PE12,PE10,PE7};
bool ls_states[10] = {true};
bool pre_ls_states[10] = {true};

//estate_callback
float estates[4] = {0.0};

//cable
float cable_length = 0.0f;
float run_length = 0.0f;

// gyro
#define ROLL  0
#define PITCH 1
#define YAW   2

bool BNO055_is_connected = false;
float degree[3] = {0.0};
float init_degree[3] = {0.0};

float cable_state = 0.0f;

void es_timer_callback(void){
  
  for(int i = 0; i < 4; i++){
    pre_target_height[i] = target_height[i];
  }
  target_height[0] = target_height[3] = cmd.data[ES_HTF];
  target_height[1] = target_height[2] = cmd.data[ES_HTB];

  //Serial.print(cmd.data[ES_HTF]);
  //Serial.print("\t");
  //Serial.print(cmd.data[ES_HTB]);
  //Serial.print("\t");

  //エンコーダの値を取得
  for(int i = 0; i < 4; i++){ 
    if(i % 2 == 1){
      estates[i] = mdc.angle(es_mdc_ids[i]) * PITCH_RADIUS;
    }else{
      estates[i] = mdc.angle(es_mdc_ids[i]) * PITCH_RADIUS * -1.0f;
    }
  }
    
  if(cmd.mode[ES_MODE] == 0){  //指定高度制御

    for(int i = 0; i < 4; i++){
      //duty比の計算
      es_pwm[i] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[i] - estates[i]) * HEIGHT_GAIN ));
      //Serial.print(estates[i]);
      //Serial.print("\t");
      if(target_height[i] < 0){ //目標高度が負なら
        if(ls_states[i] == LOW){  //リミットスイッチが押されていなければ
          mdc.move(es_mdc_ids[i], -0.12f);  //少しずつエレサスを引っ込める
        }else{                    //リミットスイッチが押されていれば
          mdc.move(es_mdc_ids[i], ES_MINUS_DUTY);  //高さ維持(負)
        }
      }else if(target_height[0] == 0 && target_height[1] == 0){ //前後方の目標硬度が0なら
        mdc.move(es_mdc_ids[i], 0.0f);
        if(estates[i] != 0.0f){ //エンコーダの値が0でないならリセット
          mdc.reset_encoder(es_mdc_ids[i]);
        }
      }else{  //目標高度が正なら
        mdc.move(es_mdc_ids[i], es_pwm[i]);
      }

      // if(ls_states[i] == HIGH){ //リミットスイッチが押されていたら=最も低くなったら
      //   if(es_pwm[i] < 0.0f){ //縮めようとするとき
      //     mdc.move(es_mdc_ids[i], -0.03f);
      //   }else{  //伸ばそうとするとき
      //     mdc.move(es_mdc_ids[i], es_pwm[i]);
      //   }
      // }else if(estates[i] > 160.f){ //高さが16cm以上なら
      //   mdc.move(es_mdc_ids[i], 0.0f);
      // }else{  //リミットスイッチが押されておらず高さが16cm未満なら
      //   if(target_height[0] == 0 && target_height[1] == 0){ //全後方の目標高度が0なら    目標高度との差が4mm以下のとき
      //     mdc.move(es_mdc_ids[i], 0.0f);
      //     if(estates[i] != 0.0f){
      //       mdc.reset_encoder(es_mdc_ids[i]);
      //     }
      //   }else{  //目標との差が4mm以上あるとき
      //     mdc.move(es_mdc_ids[i], es_pwm[i]);
      //   }
      // }
    }
    //Serial.println();
  }else{  //水平維持制御
    /*
    if(degree[PITCH] <= 0.0f){ //上り 前方(0,3)下げ・後方(1,2)高さ維持
      if(estates[1] < target_height[BACK]-5 || estates[2] < target_height[BACK]-5){ //後方が下がっていたら優先して上げる
        es_pwm[0] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[0] - estates[0]) * HEIGHT_GAIN ));
        es_pwm[1] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, 0.08f - degree[PITCH] * -PITCH_GAIN));
        es_pwm[2] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, 0.08f - degree[PITCH] * -PITCH_GAIN));
        es_pwm[3] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[3] - estates[3]) * HEIGHT_GAIN ));
      }else{  //前後方の高さが同じなら
        es_pwm[0] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[0] - estates[0]) * HEIGHT_GAIN ));
        es_pwm[1] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[1] - estates[1]) * HEIGHT_GAIN ));
        es_pwm[2] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[2] - estates[2]) * HEIGHT_GAIN ));
        es_pwm[3] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[3] - estates[3]) * HEIGHT_GAIN ));
      }
    }else{  //下り 後方(1,2)下げ・前方(0,3)高さ維持
      if(estates[0] < target_height[FRONT]-5 || estates[3] < target_height[FRONT]-5){ //前方が下がっていたら優先して上げる
        es_pwm[0] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, 0.08f - degree[PITCH] * PITCH_GAIN));
        es_pwm[1] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[1] - estates[1]) * HEIGHT_GAIN ));
        es_pwm[2] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[2] - estates[2]) * HEIGHT_GAIN ));
        es_pwm[3] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, 0.08f - degree[PITCH] * PITCH_GAIN));
      }else{  //前後方の高さが同じなら
        es_pwm[0] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[0] - estates[0]) * HEIGHT_GAIN ));
        es_pwm[1] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[1] - estates[1]) * HEIGHT_GAIN ));
        es_pwm[2] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[2] - estates[2]) * HEIGHT_GAIN ));
        es_pwm[3] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[3] - estates[3]) * HEIGHT_GAIN ));
      }
    }
    */
    if(target_height[0] == 0 && target_height[1] == 0){
      es_pwm[0] = es_pwm[1] = es_pwm[2] = es_pwm[3] = 0.0f;
    }else if(degree[PITCH] <= 0.0f){ //上り 前方(0,3)下げ・後方(1,2)高さ維持
      // if(estates[0] > estates[1]+5 || estates[3] > estates[2]+5){ //後方が下がっていたら優先して上げる
      //   es_pwm[0] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[0] - estates[0]) * HEIGHT_GAIN));
      //   es_pwm[1] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[1] - estates[1]) * HEIGHT_GAIN ));
      //   es_pwm[2] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[2] - estates[2]) * HEIGHT_GAIN ));
      //   es_pwm[3] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[3] - estates[3]) * HEIGHT_GAIN ));
      // }else{  //後ろのほうが高くなったら
      //   es_pwm[0] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, 0.08f + degree[PITCH] * PITCH_GAIN));
      //   es_pwm[1] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[1] - estates[1]) * HEIGHT_GAIN ));
      //   es_pwm[2] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[2] - estates[2]) * HEIGHT_GAIN ));
      //   es_pwm[3] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, 0.08f + degree[PITCH] * PITCH_GAIN));
      // }
        es_pwm[0] = min(ES_HOR_DUTY_MAX, max(ES_HOR_DUTY_MIN, 0.08f + degree[PITCH] * PITCH_GAIN));
        es_pwm[1] = min(ES_HOR_DUTY_MAX, max(ES_HOR_DUTY_MIN, (target_height[1] - estates[1]) * HEIGHT_GAIN ));
        es_pwm[2] = min(ES_HOR_DUTY_MAX, max(ES_HOR_DUTY_MIN, (target_height[2] - estates[2]) * HEIGHT_GAIN ));
        es_pwm[3] = min(ES_HOR_DUTY_MAX, max(ES_HOR_DUTY_MIN, 0.08f + degree[PITCH] * PITCH_GAIN));
    }else{  //下り 後方(1,2)下げ・前方(0,3)高さ維持
      // if(estates[0]+5 < estates[1] || estates[3]+5 < estates[2]){ //前方が下がっていたら優先して上げる
      //   es_pwm[0] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[0] - estates[0]) * HEIGHT_GAIN ));
      //   es_pwm[1] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[1] - estates[1]) * HEIGHT_GAIN ));
      //   es_pwm[2] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[2] - estates[2]) * HEIGHT_GAIN ));
      //   es_pwm[3] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[3] - estates[3]) * HEIGHT_GAIN ));
      // }else{  //前のほうが高くなったら
      //   es_pwm[0] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[0] - estates[0]) * HEIGHT_GAIN ));
      //   es_pwm[1] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, 0.08f - degree[PITCH] * PITCH_GAIN));
      //   es_pwm[2] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, 0.08f - degree[PITCH] * PITCH_GAIN));
      //   es_pwm[3] = min(ES_DUTY_MAX, max(ES_DUTY_MIN, (target_height[3] - estates[3]) * HEIGHT_GAIN ));
      // }
        es_pwm[0] = min(ES_HOR_DUTY_MAX, max(ES_HOR_DUTY_MIN, (target_height[0] - estates[0]) * HEIGHT_GAIN ));
        es_pwm[1] = min(ES_HOR_DUTY_MAX, max(ES_HOR_DUTY_MIN, 0.08f - degree[PITCH] * PITCH_GAIN));
        es_pwm[2] = min(ES_HOR_DUTY_MAX, max(ES_HOR_DUTY_MIN, 0.08f - degree[PITCH] * PITCH_GAIN));
        es_pwm[3] = min(ES_HOR_DUTY_MAX, max(ES_HOR_DUTY_MIN, (target_height[3] - estates[3]) * HEIGHT_GAIN ));
    }
    
    /*
    es_pwm[0] = min(0.1f, max(-0.1f, degree[ROLL] * -0.02f + degree[PITCH] *  0.02f));
    es_pwm[1] = min(0.1f, max(-0.1f, degree[ROLL] * -0.02f + degree[PITCH] * -0.02f));
    es_pwm[2] = min(0.1f, max(-0.1f, degree[ROLL] *  0.02f + degree[PITCH] * -0.02f));
    es_pwm[3] = min(0.1f, max(-0.1f, degree[ROLL] *  0.02f + degree[PITCH] *  0.02f));
    */
    for(int i = 0; i < 4; i++){
      if(ls_states[i] == HIGH){
        if(es_pwm[i] < 0.0f){ //縮めようとするとき
          mdc.move(es_mdc_ids[i], ES_MINUS_DUTY);
          //mdc.reset_encoder(es_mdc_ids[i]);
        }else{  //伸ばそうとするとき
          mdc.move(es_mdc_ids[i], es_pwm[i]);
        }
      }else if(estates[i] > 160.0f){  //高さが16cm以上なら
        mdc.move(es_mdc_ids[i], 0.0f);
      }else{  //リミットスイッチが押されていないなら
        // if(abs(degree[PITCH]) > 1.0f || abs(degree[ROLL]) > 1.0f){  //ROLLかPITCHが1度以上なら
        //   mdc.move(es_mdc_ids[i], es_pwm[i]);
        // }else{
        //   mdc.move(es_mdc_ids[i], 0.0f);
        // }
        mdc.move(es_mdc_ids[i], es_pwm[i]);
      }
      //Serial.print(estates[i]);
      //Serial.print("\t");
    }
    //Serial.println();
  }
}

void undercarriage_timer_callback(void){
  if(cmd.mode[MC_MODE] == 0){ //mecanum
    //メカナムのduty比を計算
    mt_pwm[0] = min(0.3f, max(-0.3f, (-cmd.data[LEFT_X] + cmd.data[LEFT_Y] + cmd.data[RIGHT])/CMD_MAX));
    mt_pwm[1] = min(0.3f, max(-0.3f, (-cmd.data[LEFT_X] - cmd.data[LEFT_Y] + cmd.data[RIGHT])/CMD_MAX));
    mt_pwm[2] = min(0.3f, max(-0.3f, (+cmd.data[LEFT_X] - cmd.data[LEFT_Y] + cmd.data[RIGHT])/CMD_MAX));
    mt_pwm[3] = min(0.3f, max(-0.3f, (+cmd.data[LEFT_X] + cmd.data[LEFT_Y] + cmd.data[RIGHT])/CMD_MAX));

    //クローラのduty比を0にする
    for(int i = 0; i < 2; i++){
      cr_pwm[i] = 0.0f;
    }
  }else{  //crawler

    //クローラのduty比を計算
    cr_pwm[0] = cmd.data[LEFT_X] / CMD_MAX;
    cr_pwm[1] = cmd.data[RIGHT]  / CMD_MAX;

    //メカナムのduty比を0にする伸ばすとき
    for(int i = 0; i < 4; i++){
      mt_pwm[i] = 0.0f;
    }
  }

  //メカナムのモータを回転
  for(int i = 0; i < 4; i++){
    //Serial.print(mt_pwm[i]);
    //Serial.print("\t");
    mdc.move(mt_mdc_ids[i], mt_pwm[i]);
  }
  //Serial.println();

  //クローラのモータを回転
  for(int i = 0; i < 2; i++){
    mt_crawler_left.move(cr_pwm[0]);
    mt_crawler_right.move(cr_pwm[1]);
  }
  //Serial.println("m");
}

void decomission_timer_callback(void){

  // Rise & Fall ============================================================

  if(cmd.mode[RF_MODE] == 0){ //全体モード
    if(ls_states[RF_UP] == true && cmd.data[RF_PWM] > 0){       //上のリミットスイッチが押されていて上げるとき
      mt_rf.move(0.0f);
    }else if(ls_states[RF_DN] == true && cmd.data[RF_PWM] < 0){ //下のリミットスイッチが押されていて下げるとき
      mt_rf.move(0.0f);
    }else{
      mt_rf.move(cmd.data[RF_PWM]/CMD_MAX);
    }
    mt_rfad.move(0.0f);
  }else{                      //調整モード
    if(ls_states[RFAD_UP] == true && cmd.data[RF_PWM] > 0){       //上のリミットスイッチが押されていて上げるとき
      mt_rfad.move(0.0f);
    }else if(ls_states[RFAD_DN] == true && cmd.data[RF_PWM] < 0){ //下のリミットスイッチが押されていて下げるとき
      mt_rfad.move(0.0f);
    }else{
      //Serial.print(cmd.data[RF_PWM]/CMD_MAX);
      mt_rfad.move(cmd.data[RF_PWM]/CMD_MAX);
    }
    mt_rf.move(0.0f);
  }

  // ========================================================================

  // Decomission Mechanism ==================================================

  if(ls_states[DECON_L] == true && cmd.data[DECON] > 0){        //左のリミットスイッチが押されていて左に伸ばすとき
    mt_decon.move(0.0f);
  }else if(ls_states[DECON_R] == true && cmd.data[DECON] < 0){  //右のリミットスイッチが押されていて右に伸ばすとき
    mt_decon.move(0.0f);
  }else{
    mt_decon.move(cmd.data[DECON]/CMD_MAX/2.0f);
  }

  // cable
  //cable_state = mdc.read(0x18) * CABLE_RADIUS;
  //Serial.print(cmd.mode[CABLE]);
  mdc.move(0x18,cmd.mode[CABLE]*-0.9f);

}

int interrupt_count = 0;

void timer_interrupt(void){
  interrupt_count += 1;
  //Serial.print(degree[ROLL]);
  // Serial.print("\t");
  // Serial.print(degree[PITCH]);
  // Serial.print("\t");
  // Serial.println(degree[YAW]);
  if(interrupt_count%2 == 0){
    //Serial.print(degree[PITCH]);
    //Serial.print("\t");
    if(BNO055_is_connected){
      imu::Vector<3> gyro_vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      degree[ROLL] = gyro_vec.z() - init_degree[ROLL];
      degree[PITCH] = gyro_vec.y() - init_degree[PITCH];
      degree[YAW] = gyro_vec.x();
      //Serial.print(degree[PITCH]);
      //Serial.print("\t");
    }
    es_timer_callback();
    undercarriage_timer_callback();
  }
  if(interrupt_count % 10 == 0){
    decomission_timer_callback();
  }
  if(interrupt_count == 200){
    for(int i = 0; i < 5; i ++){
      cmd.data[i] = 0;
    }
    cmd.mode[3] = 0;
    interrupt_count = 0;
  }
}

// ==========================================================================

// Subscription callbacks ===================================================

void cmd_callback(const void *msgin)
{
  const custom_msgs__msg__Commands *_msg = (const custom_msgs__msg__Commands *)msgin;
  if(_msg == NULL){
    Serial.print("/ns/cmd is NULL ");
  }else{
    for(int i = 0; i < 7; i++){
      cmd.data[i] = _msg->data[i];
    }
    for(int i = 0; i < 4; i++){
      cmd.mode[i] = _msg->mode[i];
    }
    //s_time = millis();
  }
  //Serial.println("cmd");
}

void espwm_callback(const void *msgin){
  const custom_msgs__msg__ElectricSuspensionPwm *_msg = (const custom_msgs__msg__ElectricSuspensionPwm *)msgin;
  for(int i = 0; i < 4; i++){
    es_pwm[i] = _msg->es_pwm[i] / CMD_MAX;
  }
  //Serial.println("espwm");
}

void estate_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  (void)timer;

  for(int i = 0; i < 4; i++){
    if(estates[i] >= 0.0f){
      msg_estate.estate[i] = (int)(estates[i]);
    }else{
      msg_estate.estate[i] = 0;
    }
  }
  //Serial.println("estate");
  rcl_publish(&estate_publisher, &msg_estate, NULL);
}

// ==========================================================================

// Setup micro_ros_arduino =================================================
void setup()
{
  //Serial
  Serial.begin(115200);
  Serial.println("Setup Serial");

  //On-board LED&Switch
  pinMode(PB7,OUTPUT);  //
  pinMode(PB14,OUTPUT); //

  //Micro-ROS setup
  byte arduino_mac[] = {0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF};
  IPAddress arduino_ip(192, 168, 0, 46);
  IPAddress agent_ip(192, 168, 0, 10);
  int agent_port = 2000;

  //Limitswitch setup
  for(int i = 0; i < 10; i++){
    pinMode(ls_pins[i],INPUT_PULLUP);
    delay(1);
    ls_states[i] = digitalRead(ls_pins[i]);
    msg_ls.ls[i] = ls_states[i];
  }

  //BNO055 setup
  delay(100);
  if(bno.begin()){
    Serial.println("BNO055 was connected");
    BNO055_is_connected = true;
    digitalWrite(PB14,HIGH);
  }else{
    Serial.println("BNO055 was not found");
  }

  //BNO055 Calibration
  if(BNO055_is_connected == true){
    delay(100);
    imu::Vector<3> gyro_vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    init_degree[ROLL] = gyro_vec.z();
    init_degree[PITCH] = gyro_vec.y();
  }

  setup_microros_ethernet("uros_node", "ns", 2, arduino_mac, arduino_ip, agent_ip, agent_port);

  // rclc-publisher-subscriber-timer ======================================
  rclc_publisher_init_default(&ls_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, LimitSwitch), "ls");
  rclc_publisher_init_default(&estate_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, ElectricSuspensionState), "estate");
  //rclc_create_timer_and_add(&ls_timer, 500, ls_callback);
  rclc_create_timer_and_add(&estate_timer, 100, estate_callback);

  //subscription
  //rclc_create_subscription_and_add(&espwm_subscriber, ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, ElectricSuspensionPwm), &msg_espwm, &espwm_callback, "espwm");
  rclc_create_subscription_and_add(&cmd_subscriber, ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Commands), &msg_cmd, &cmd_callback, "cmd");

  //Start Timer Interrupt
  if(ITimer10.attachInterruptInterval(5000,timer_interrupt)){  //us,callback
    Serial.println("Start Timer Interrupt");
  }else{
    Serial.println("Failed to setup Timer Interrupt");
  }

  digitalWrite(PB7,HIGH);
}

// loop micro_ros_arduino ===============================================

void loop()
{
  //Limit Switch ========================================================

  int ls_cnt = 0;
  for(int i = 0; i < 10; i++){
    pre_ls_states[i] = ls_states[i];
    ls_states[i] = digitalRead(ls_pins[i]);
    if(pre_ls_states[i] != ls_states[i]){
      ls_cnt += 1;
    }
  }

  if(ls_cnt > 0){
    for(int i = 0; i < 10; i++){
      msg_ls.ls[i] = ls_states[i];
    }
    //Serial.println("ls");
    rcl_publish(&ls_publisher, &msg_ls, NULL);
  }

  //暴走停止
  // if(millis() - s_time > 200){
  //   for(int i = 0; i < 5; i ++){
  //     cmd.data[i] = 0;
  //   }
  //   cmd.mode[3] = 0;
  //   Serial.println("### break ###");
  // }
  
  // cable_length = mdc.angle(0x18) * -CABLE_RADIUS;
  // run_length = mdc.angle(0x14) * -MCNM_RADIUS;
  
  // Serial.print(cable_length);
  // Serial.print("\t");
  // Serial.println(run_length);

  //rclc_delay(1);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}