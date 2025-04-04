#include <Arduino.h>
#include <Wire.h>
#include "Mecanum.h"
#include "GY85.h"

#include "painlessMesh.h"
#include <set> // 用於 std::set

#define MESH_PREFIX "MESH_PREFIX"
#define MESH_PASSWORD "MESH_PASSWORD"
#define MESH_PORT 5555

#define motor_1_GPIO 32  // 前面左邊
#define motor_2_GPIO 33  // 前面右邊
#define motor_3_GPIO 25  // 後面左邊
#define motor_4_GPIO 26  // 後面右邊

#define motor_1_reverse_GPIO 16
#define motor_2_reverse_GPIO 17
#define motor_3_reverse_GPIO 4
#define motor_4_reverse_GPIO 13

#define motor_1_encoder_GPIO 36
#define motor_2_encoder_GPIO 39
#define motor_3_encoder_GPIO 34
#define motor_4_encoder_GPIO 35

#define motor_1_encoder_b_GPIO 14
#define motor_2_reverse_b_GPIO 15
#define motor_3_reverse_b_GPIO 27
#define motor_4_reverse_b_GPIO 23

#define PWM_freq 1000
#define PWM_resolution 11  // PWM精度 (2^resolution)

#define PID_BUFFERSIZE 10  

#define VEL2SQ 74.58       // (m/s) * VEL2SQ
#define DEGREED_2_RAD 0.017453
#define RPM2SQ 0.322727
#define M_S_2_CM_S 0.01


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 結構體

struct PID_t{
  float Kp;
  float Ki;
  float Kd;
  float target_val;
  float err_buffer[PID_BUFFERSIZE];
  int cir_queue_index;
  float dt;
  float max_ctrl_val;
  float min_ctrl_val;
  float ctrl_val;
  float static_friction_comp;  // 靜摩擦補償值
  float friction_threshold;    // 靜摩擦啟動的速度門檻
};

struct Motor{
  int GPIO;
  int Register_Pin;
  int Reverse_control_GPIO;
  int encoder_GPIO;
  int now_rev;
  struct PID_t *PID;
  float last_pwm_output;
};

struct Timer{
  hw_timer_t *name;
  int channel;  // 0~3
  int freq;
  bool mode;  // (上升緣/下降緣)觸發 -> (true/false)
  void (*function)(void);
  bool autoreload;  // 重複執行
  bool enable;
};

//wifi mesh data
struct list_node{
  float data_yaw;
  float data_spd;
  struct list_node *next;
};

struct data_queue{
  list_node *head;
  list_node *end;
  int length;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 全域變數
int test_2 = 0;
int test_count = 0;
int test_ = 30;

float car1_yaw = 0;
float car1_spd = 0;
int last_code = 9;

// wifi mesh
Scheduler userScheduler;
painlessMesh mesh;
std::set<uint32_t> connectedNodes; // 全域變數，用於追蹤已連線的節點

String wifi_mesh_msg = "test";
String wifi_mesh_msg_receive;
String hand_shake_msg = "OK!";

struct data_queue queue;
bool is_data_ready;

struct PID_t PID_motor_1;
struct PID_t PID_motor_2;
struct PID_t PID_motor_3;
struct PID_t PID_motor_4;
/*
     front
    2      1

    4      3
*/ 
struct Motor motor_1 = {.GPIO=motor_1_GPIO, .Register_Pin=12, .Reverse_control_GPIO=motor_1_reverse_GPIO, .encoder_GPIO=motor_1_encoder_GPIO, .now_rev=0, .PID=&PID_motor_1, .last_pwm_output=0.0};
struct Motor motor_2 = {.GPIO=motor_2_GPIO, .Register_Pin=13, .Reverse_control_GPIO=motor_2_reverse_GPIO, .encoder_GPIO=motor_2_encoder_GPIO, .now_rev=0, .PID=&PID_motor_2, .last_pwm_output=0.0};
struct Motor motor_3 = {.GPIO=motor_3_GPIO, .Register_Pin=14, .Reverse_control_GPIO=motor_3_reverse_GPIO, .encoder_GPIO=motor_3_encoder_GPIO, .now_rev=0, .PID=&PID_motor_3, .last_pwm_output=0.0};
struct Motor motor_4 = {.GPIO=motor_4_GPIO, .Register_Pin=15, .Reverse_control_GPIO=motor_4_reverse_GPIO, .encoder_GPIO=motor_4_encoder_GPIO, .now_rev=0, .PID=&PID_motor_4, .last_pwm_output=0.0};
struct Mecanum_str macanum;

GY85 gy85;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function宣告

void motor_init();
void PWM_output(struct Motor motor, float duty);
void PID_str_init(struct PID_t *PID, float Kp, float Ki, float Kd, float dt, float max, float min);
void motor_targe_speed_set(struct Motor *motor, float m_s);
void motor_targe_set_rpm(struct Motor *motor, float rpm);
void PID_control(struct PID_t *PID, float new_val);
void motor_PID_control(struct Motor *motor);
void Mecanum_calc_speed(struct Mecanum_str *M);
void Mecanum_calc_speed_tmp(struct Mecanum_str *M, float v_turn);
void speed_pid_test();
void macanuum_test();

void queue_insert_data(struct data_queue *Q, float yaw, float spd);
String queue_pop_data(struct data_queue *Q);
void prepare_data();


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 中斷宣告
void Timer_init(struct Timer timer);
portMUX_TYPE mux0 = portMUX_INITIALIZER_UNLOCKED;    //保護機制
// timer中斷宣告

// timer0 (encoder)
static hw_timer_t * timer_0;  
bool timer_0_flag = false;
void IRAM_ATTR Timer_0_ISR();  
struct Timer Timer_0 = {.name=timer_0, .channel=0, .freq=10, .mode=true, .function=&Timer_0_ISR, .autoreload=true, .enable=true};


//functions for wifi mesh
void sendMessage();
Task taskSendMessage(1, TASK_FOREVER, &sendMessage);
// send
void sendMessage() {
  mesh.sendBroadcast(wifi_mesh_msg);
}

void receivedCallback(uint32_t from, String &msg) {
  // 更新時就會呼叫
  // 將收到的 String 解析成三個部分：code、spd 和另一個浮點數
  // Serial.println(msg);
  int firstSpaceIndex = msg.indexOf(' ');  // 找到第一個空格的位置
  int secondSpaceIndex = msg.indexOf(' ', firstSpaceIndex + 1);  // 找到第二個空格的位置
  if (firstSpaceIndex > 0 && secondSpaceIndex > firstSpaceIndex) {  // 確保格式正確
    String code_str = msg.substring(0, firstSpaceIndex);  // 第一部分：code
    String yaw_str = msg.substring(firstSpaceIndex + 1, secondSpaceIndex);  // 第二部分：yaw
    String spd_str = msg.substring(secondSpaceIndex + 1);  // 第三部分：spd

    int code = code_str.toInt();   // 將 code 部分轉換成整數
    float yaw = yaw_str.toFloat();  // 將 yaw 部分轉換成浮點數
    float spd = spd_str.toFloat();  // 將 spd 部分轉換成浮點數
    if (code != last_code) {
      last_code = code;
      queue_insert_data(&queue, yaw, spd);
      //Serial.printf("%d %f %f\n", code, yaw, spd);
      hand_shake_msg = String(last_code) + "OK!";
    }
    //Serial.println(hand_shake_msg);
    mesh.sendBroadcast(hand_shake_msg);
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> New Connection, nodeId = %u\n", nodeId);
  connectedNodes.insert(nodeId);
}

void changedConnectionCallback() {
  Serial.println("Changed connections");

  auto newConnections = mesh.getNodeList();
  std::set<uint32_t> currentNodes(newConnections.begin(), newConnections.end());

  for (const auto& nodeId : connectedNodes) {
      if (currentNodes.find(nodeId) == currentNodes.end()) {
          Serial.printf("--> Disconnected nodeId = %u\n", nodeId);
      }
  }

  connectedNodes = currentNodes;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

// 外部中斷宣告
// encoder 外部中斷計數
void ARDUINO_ISR_ATTR isr_encode_1_count(){
  if(digitalRead(motor_1_encoder_b_GPIO) == LOW){motor_1.now_rev++;}
  else{motor_1.now_rev--;}
}
void ARDUINO_ISR_ATTR isr_encode_2_count(){
  if(digitalRead(motor_2_reverse_b_GPIO) == HIGH){motor_2.now_rev++;}
  else{motor_2.now_rev--;}
}
void ARDUINO_ISR_ATTR isr_encode_3_count(){
  if(digitalRead(motor_3_reverse_b_GPIO) == LOW){motor_3.now_rev++;}
  else{motor_3.now_rev--;}
}
void ARDUINO_ISR_ATTR isr_encode_4_count(){
  if(digitalRead(motor_4_reverse_b_GPIO) == HIGH){motor_4.now_rev++;}
  else{motor_4.now_rev--;}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// main

void setup() {
  Serial.begin(115200);

  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  queue.end = NULL;
  queue.head = NULL;
  queue.length = 0;

  // userScheduler.addTask(taskSendMessage);
  // taskSendMessage.enable();

  gy85.init();

  motor_init();
  MECANUM_INIT(macanum);
  delay(10);
  
  PID_str_init(motor_1.PID, 0.7, 0.006, 0.3, 0.1, 170.0, -170.0);
  PID_str_init(motor_2.PID, 0.7, 0.006, 0.3, 0.1, 170.0, -170.0);
  PID_str_init(motor_3.PID, 0.7, 0.006, 0.3, 0.1, 170.0, -170.0);
  PID_str_init(motor_4.PID, 0.7, 0.006, 0.3, 0.1, 170.0, -170.0);

  Timer_init(Timer_0);
  // car1_spd = 30.0;
  // car1_yaw = 0.0;
  // macanum.vel_x = 10;
  // macanum.vel_y = 0;
  
}

void loop() {
  mesh.update();
  if(timer_0_flag){// 10hz
    timer_0_flag = false;
    gy85.update();
    
    //macanum.omega = -1*gy85.gyro_yaw*DEGREED_2_RAD*2;
    
    prepare_data();
    Serial.printf("QL:%d yaw:%f spd:%f\n", queue.length, car1_yaw, car1_spd);
    macanum.vel_x = car1_spd*cos(car1_yaw/RAD_TO_DEG);//cm/s
    macanum.vel_y = car1_spd*sin(car1_yaw/RAD_TO_DEG);
    // macanum.vel_x = 10;
    // macanum.vel_y = 0;
    Mecanum_calc_speed(&macanum);
    
    motor_targe_set_rpm(&motor_2, macanum.left_front);
    motor_targe_set_rpm(&motor_4, macanum.left_rear);
    motor_targe_set_rpm(&motor_1, macanum.right_front);
    motor_targe_set_rpm(&motor_3, macanum.right_rear);

    motor_PID_control(&motor_1);
    motor_PID_control(&motor_2);
    motor_PID_control(&motor_3);
    motor_PID_control(&motor_4);

  //   // Serial.printf("%f\n", macanum.left_front);
  //   // car1_spd = 0;//  reset the speed, preventing the car from lossing control
    

  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void motor_init() {
  // PWM GPIO設置為輸出
  pinMode(motor_1.GPIO, OUTPUT);
  pinMode(motor_2.GPIO, OUTPUT);
  pinMode(motor_3.GPIO, OUTPUT);
  pinMode(motor_4.GPIO, OUTPUT);

  // 註冊PWM的腳位
  ledcAttachPin(motor_1.GPIO, motor_1.Register_Pin);
  ledcAttachPin(motor_2.GPIO, motor_2.Register_Pin);
  ledcAttachPin(motor_3.GPIO, motor_3.Register_Pin);
  ledcAttachPin(motor_4.GPIO, motor_4.Register_Pin);

  // PWM設定(註冊腳位, freq, resolution)
  ledcSetup(motor_1.Register_Pin, PWM_freq, PWM_resolution);
  ledcSetup(motor_2.Register_Pin, PWM_freq, PWM_resolution);
  ledcSetup(motor_3.Register_Pin, PWM_freq, PWM_resolution);
  ledcSetup(motor_4.Register_Pin, PWM_freq, PWM_resolution);

  // 反轉控制GPIO設置為輸出
  pinMode(motor_1.Reverse_control_GPIO, OUTPUT);
  pinMode(motor_2.Reverse_control_GPIO, OUTPUT);
  pinMode(motor_3.Reverse_control_GPIO, OUTPUT);
  pinMode(motor_4.Reverse_control_GPIO, OUTPUT);

  // encoder GPIO設置為輸入
  pinMode(motor_1.encoder_GPIO, INPUT);
  pinMode(motor_2.encoder_GPIO, INPUT);
  pinMode(motor_3.encoder_GPIO, INPUT);
  pinMode(motor_4.encoder_GPIO, INPUT);

  // encoder 測轉速(外部中斷)
  attachInterrupt(digitalPinToInterrupt(motor_1.encoder_GPIO), isr_encode_1_count, RISING);
  attachInterrupt(digitalPinToInterrupt(motor_2.encoder_GPIO), isr_encode_2_count, RISING);
  attachInterrupt(digitalPinToInterrupt(motor_3.encoder_GPIO), isr_encode_3_count, RISING);
  attachInterrupt(digitalPinToInterrupt(motor_4.encoder_GPIO), isr_encode_4_count, RISING);


  PWM_output(motor_1, 0);
  PWM_output(motor_2, 0);
  PWM_output(motor_3, 0);
  PWM_output(motor_4, 0);
}

void PWM_output(struct Motor motor, float duty) {
  if(duty<0){
    digitalWrite(motor.Reverse_control_GPIO, HIGH);
    ledcWrite(motor.Register_Pin, -1*duty * pow(2, PWM_resolution));
  }else{
    digitalWrite(motor.Reverse_control_GPIO, LOW);
    ledcWrite(motor.Register_Pin, duty * pow(2, PWM_resolution));
  }
}

void PID_str_init(struct PID_t *PID, float Kp, float Ki, float Kd, float dt, float max, float min){
  PID->Kp = Kp;
  PID->Ki = Ki;
  PID->Kd = Kd;
  // PID->target_val = targe_val;
  PID->dt = dt;
  for (int i = 0; i < PID_BUFFERSIZE; i++){
    PID->err_buffer[i] = 0;
  }
  PID->cir_queue_index = 0;
  PID->max_ctrl_val = max;
  PID->min_ctrl_val = min;
  PID->ctrl_val = 0;
  PID->friction_threshold = 20.0;
  PID->static_friction_comp = 10.0;
}

void motor_targe_speed_set(struct Motor *motor, float m_s){
  motor->PID->target_val = m_s * VEL2SQ;
}

void motor_targe_set_rpm(struct Motor *motor, float rpm){
  motor->PID->target_val = rpm * RPM2SQ;
}

void PID_control(struct PID_t *PID, float new_val){
  float P, I, D, d_err;
  float err_sum = 0;
  PID->err_buffer[PID->cir_queue_index] = PID->target_val - new_val;
  
  for (int i = 0; i < PID_BUFFERSIZE; i++){
    err_sum += PID->err_buffer[i];
  }
  if (PID->cir_queue_index == 0){
    d_err = PID->err_buffer[PID->cir_queue_index] - PID->err_buffer[PID_BUFFERSIZE - 1];
  }
  else{
    d_err = PID->err_buffer[PID->cir_queue_index] - PID->err_buffer[PID->cir_queue_index - 1];
  }

  P = PID->err_buffer[PID->cir_queue_index] * PID->Kp;
  I = err_sum * PID->Ki;
  D = PID->Kd * d_err / PID->dt;
  
  PID->ctrl_val = P + I + D;
  // 啟動臨界:50rpm -> 19.5 (sq/0.1s)
  // if(PID->target_val > 19.5){ 
  //     // 靜摩擦補償：在速度接近 0 時，加入靜摩擦補償
  //   if (fabs(new_val) < PID->friction_threshold  && PID->ctrl_val != 0.0) {
  //     if (PID->ctrl_val >= 0.0) {
  //       PID->ctrl_val += PID->static_friction_comp; // 正轉補償
  //     } else {
  //       PID->ctrl_val -= PID->static_friction_comp; // 負轉補償
  //     }
  //   }
  // }else{
  //   if (fabs(new_val) < PID->friction_threshold  && PID->ctrl_val != 0.0) {
  //     if (PID->ctrl_val >= 0.0) {
  //       PID->ctrl_val += PID->static_friction_comp/7; // 正轉補償
  //     } else {
  //       PID->ctrl_val -= PID->static_friction_comp/7; // 負轉補償
  //     }
  //   }
  // }

  // 限制控制輸出在最大/最小範圍內
  if (PID->ctrl_val > PID->max_ctrl_val){
    PID->ctrl_val = PID->max_ctrl_val;
  }
  else if (PID->ctrl_val < PID->min_ctrl_val){
    PID->ctrl_val = PID->min_ctrl_val;
  }

  // 更新循環緩衝區
  PID->cir_queue_index++;
  if (PID->cir_queue_index >= PID_BUFFERSIZE){
    PID->cir_queue_index = 0;
  }
}

void motor_PID_control(struct Motor *motor){
  // if(digitalRead(motor->Reverse_control_GPIO) == HIGH){
  //   motor->now_rev = (-1) * motor->now_rev;
  // }

  PID_control(motor->PID, (float)motor->now_rev);
  motor->now_rev = 0;
  motor->last_pwm_output += motor->PID->ctrl_val;
  
  if (motor->last_pwm_output >= 0.0){
    digitalWrite(motor->Reverse_control_GPIO, LOW);
    ledcWrite(motor->Register_Pin, motor->last_pwm_output*10);
  }else{
    digitalWrite(motor->Reverse_control_GPIO, HIGH);
    ledcWrite(motor->Register_Pin, (-1.0)*motor->last_pwm_output*10);
  }
}

// 用法在結構體裡設定好速度、旋轉，然後執行
// 執行完後自己從結構體取出轉速，拿去控制輪子
// 注意速度不要太快，如果結果超過輪子轉速上限，跑起來會怪怪的
void Mecanum_calc_speed(struct Mecanum_str *M){
    M->left_front = RADS2RPM * (M->vel_x - M->vel_y - ((CAR_WIDTH + CAR_LENGTH)*M->omega/2))/CAR_WHEEL_RADIUS; // leftFront rpm
    M->right_front = RADS2RPM * (M->vel_x + M->vel_y + ((CAR_WIDTH + CAR_LENGTH)*M->omega/2))/CAR_WHEEL_RADIUS; // rightFront rpm
    M->left_rear = RADS2RPM * (M->vel_x + M->vel_y - ((CAR_WIDTH + CAR_LENGTH)*M->omega/2))/CAR_WHEEL_RADIUS; // leftRear rpm
    M->right_rear = RADS2RPM * (M->vel_x - M->vel_y + ((CAR_WIDTH + CAR_LENGTH)*M->omega/2))/CAR_WHEEL_RADIUS; // rightRear rpm
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 基本時脈 : 80M
  // 週期 = 降頻值*週期次數/基本時脈
  // 頻率 = 基本時脈 / (降頻值 * 週期次數)

  // timer變數 = timerBegin(timer編號(0~3), 降頻值, 上升/下降觸發(true/false));
  // timerAttachInterrupt(timer變數, &中斷函式, 上升/下降觸發(true/false));
  // timerAlarmWrite(timer變數, 週期次數, 是否重複執行);
  // timerAlarmEnable(timer變數);

void Timer_init(struct Timer timer){
  uint64_t period_num = 80000000/80/timer.freq;
  timer.name = timerBegin(timer.channel, 80, timer.mode);    
  timerAttachInterrupt(timer.name, timer.function, timer.mode);
  timerAlarmWrite(timer.name, period_num, timer.autoreload);
  if(timer.enable == true){timerAlarmEnable(timer.name);}
}

void IRAM_ATTR Timer_0_ISR(){  
  // 中斷服務常式 不能有Serial
  timer_0_flag = true;
}

void queue_insert_data(struct data_queue *Q, float yaw, float spd){
  list_node *new_node = (list_node*)malloc(sizeof(list_node));
  new_node->data_yaw = yaw;
  new_node->data_spd = spd;
  new_node->next = NULL;
  if (Q->end){  // queue is not empty
    (Q->end)->next = new_node;
    (Q->end) = new_node;
  }
  else{ // queue is empty
    (Q->head) = new_node;
    (Q->end) = new_node;
  }
  Q->length++;
  return;
}

String queue_pop_data(struct data_queue *Q){
  list_node *temp = (Q->head);
  float yaw = temp->data_yaw;
  float spd = temp->data_spd;
  if (!Q->head){ // queue is empty
    return "e";
  }
  if (Q->head == Q->end){ //last term
    (Q->head) = NULL;
    (Q->end) = NULL;
  }
  else{
    (Q->head) = (Q->head)->next;
  }
  free(temp);
  Q->length--;
  return String(0) + " " + String(yaw, 4) + " " + String(spd, 4);
}

void prepare_data(){
  if (is_data_ready){
    if(queue.head){
      wifi_mesh_msg = queue_pop_data(&queue);
      int firstSpaceIndex = wifi_mesh_msg.indexOf(' ');  // 找到第一個空格的位置
      int secondSpaceIndex = wifi_mesh_msg.indexOf(' ', firstSpaceIndex + 1);  // 找到第二個空格的位置
      if (firstSpaceIndex > 0 && secondSpaceIndex > firstSpaceIndex) {  // 確保格式正確
        String code_str = wifi_mesh_msg.substring(0, firstSpaceIndex);  // 第一部分：code
        String yaw_str = wifi_mesh_msg.substring(firstSpaceIndex + 1, secondSpaceIndex);  // 第二部分：yaw
        String spd_str = wifi_mesh_msg.substring(secondSpaceIndex + 1);  // 第三部分：spd

        car1_yaw = yaw_str.toFloat();  // 將 yaw 部分轉換成浮點數
        car1_spd = spd_str.toFloat();  // 將 spd 部分轉換成浮點數
      }
    }
    else{
      car1_yaw = 0;  
      car1_spd = 0;
      is_data_ready = 0;
    }
  }
  else{
    car1_yaw = 0;  
    car1_spd = 0;
    if (queue.length >= 5) is_data_ready = 1;
  }
}