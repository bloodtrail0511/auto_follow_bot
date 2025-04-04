#include <Arduino.h>

// #include "FS.h"
// #include "SD.h"
// #include "SPI.h"
#include "GY85.h"

#include "painlessMesh.h"
#include <set> // 用於 std::set

#define MESH_PREFIX "MESH_PREFIX"
#define MESH_PASSWORD "MESH_PASSWORD"
#define MESH_PORT 5555

#define sensorPin1 36      // 第一個感測器 GPIO 右後輪
//#define sensorPin2 39      // 第二個感測器 GPIO 左後輪
#define back_pin 34
#define LED_PIN 32

//#define PI 3.1415926
#define CAR_W 20
#define EFFECTIVE_R 4.3326 // 3.8 // 4.3326  // 輪子有效半徑
#define DIST_CALC_CONST 0.08939

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
  int code;
  struct list_node *next;
};

struct data_queue{
  list_node *head;
  list_node *end;
};

// wifi mesh
Scheduler userScheduler;
painlessMesh mesh;
std::set<uint32_t> connectedNodes; // 全域變數，用於追蹤已連線的節點

String wifi_mesh_msg = "Hi from node2";
String wifi_mesh_msg_receive;

struct data_queue queue;
bool is_data_ready;
String hand_shake_msg = "OK!";

bool connected = 0;

int pulseCount1 = 0; // 計數器1
//volatile unsigned int pulseCount2 = 0; // 計數器2

unsigned long previousMillis = 0;      // 定時器

float RPS_R = 0, RPS_L = 0;  // RPS
float RPM_R = 0, RPM_L = 0;  // RPM

float spd_l = 0;
float spd_r = 0;
float spd = 0;
float yaw = 0;

GY85 gy85;

int is_go_back = 0;

// 消抖設定
volatile unsigned long lastInterruptTime1 = 0; 
volatile unsigned long lastInterruptTime2 = 0;
const unsigned long debounceTime = 2; //ms

bool timer_0_flag = false;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 中斷宣告
void Timer_init(struct Timer timer);
portMUX_TYPE mux0 = portMUX_INITIALIZER_UNLOCKED;    //保護機制
// timer中斷宣告

// timer0 (encoder)
static hw_timer_t * timer_0;  
void IRAM_ATTR Timer_0_ISR();  
struct Timer Timer_0 = {.name=timer_0, .channel=0, .freq=10, .mode=true, .function=&Timer_0_ISR, .autoreload=true, .enable=true};


// 第一個中斷函數
void IRAM_ATTR countPulse1() {
  // unsigned long currentInterruptTime = millis();
  // if (currentInterruptTime - lastInterruptTime1 > debounceTime) {
  //   pulseCount1++;
  //   lastInterruptTime1 = currentInterruptTime;
  // }
  pulseCount1++;
}

// 第二個中斷函數
// void IRAM_ATTR countPulse2() {
//   unsigned long currentInterruptTime = millis();
//   if (currentInterruptTime - lastInterruptTime2 > debounceTime) {
//     pulseCount2++;
//     lastInterruptTime2 = currentInterruptTime;
//   }
// }
int last_code = 9;

void queue_insert_data(struct data_queue *Q, float yaw, float spd){
  list_node *new_node = (list_node*)malloc(sizeof(list_node));
  new_node->data_yaw = yaw;
  new_node->data_spd = spd;
  new_node->code = (++last_code)%10;
  new_node->next = NULL;
  if (Q->end){  // queue is not empty
    (Q->end)->next = new_node;
    (Q->end) = new_node;
  }
  else{ // queue is empty
    (Q->head) = new_node;
    (Q->end) = new_node;
  }
  return;
}

String queue_pop_data(struct data_queue *Q){
  list_node *temp = (Q->head);
  float yaw = temp->data_yaw;
  float spd = temp->data_spd;
  int encoder = temp->code;
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
  return String(encoder) + " " + String(yaw, 4) + " " + String(spd, 4);
}

void prepare_data(){
  if(queue.head){
    wifi_mesh_msg = queue_pop_data(&queue);
    hand_shake_msg =  String(String(wifi_mesh_msg[0]).toInt()) + "OK!";
    is_data_ready = 1;
  }
}

//functions for wifi mesh
void sendMessage();
Task taskSendMessage(20, TASK_FOREVER, &sendMessage);
// send
void sendMessage() {
  if (is_data_ready){  //queue is not empty
    mesh.sendBroadcast(wifi_mesh_msg);
  }
}

void receivedCallback(uint32_t from, String &msg) {
  // get hand shake
  if (msg == hand_shake_msg){
    is_data_ready = 0;
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> New Connection, nodeId = %u\n", nodeId);
  connectedNodes.insert(nodeId);
  connected = 1;
}

void changedConnectionCallback() {
  Serial.println("Changed connections");

  auto newConnections = mesh.getNodeList();
  std::set<uint32_t> currentNodes(newConnections.begin(), newConnections.end());

  for (const auto& nodeId : connectedNodes) {
      if (currentNodes.find(nodeId) == currentNodes.end()) {
          Serial.printf("--> Disconnected nodeId = %u\n", nodeId);
          //connected = 0;
      }
  }
  connectedNodes = currentNodes;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  //Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

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

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

  pinMode(sensorPin1, INPUT_PULLUP);
  // pinMode(sensorPin2, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);  // 設置 GPIO32 為輸出模式
  pinMode(back_pin, INPUT);

  digitalWrite(LED_PIN, LOW);  // 初始化為低電平

  attachInterrupt(digitalPinToInterrupt(sensorPin1), countPulse1, FALLING);
  // attachInterrupt(digitalPinToInterrupt(sensorPin2), countPulse2, FALLING);

  Timer_init(Timer_0);
  gy85.init();
}

void loop() {
  mesh.update();
  if(!is_data_ready){
    prepare_data();
    digitalWrite(LED_PIN, LOW);
  }
  else{
    digitalWrite(LED_PIN, HIGH);
  }
  if (timer_0_flag && connected){
    timer_0_flag = 0;
    spd = (float)pulseCount1 * DIST_CALC_CONST * EFFECTIVE_R/ 2.5;  /// 2;
    pulseCount1 = 0;
    // spd = (float)pulseCount1;
    // 判斷前進後退，後退高，前進低
    is_go_back = digitalRead(back_pin);
    if (is_go_back){
      spd = -spd;
    }
    gy85.update();
    yaw = -gy85.yaw_angle;
    queue_insert_data(&queue, yaw, spd);
    printf("%f %f\n", yaw, spd);
  }
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
  portENTER_CRITICAL(&mux0); 
  timer_0_flag = true;
  portEXIT_CRITICAL(&mux0); 
}