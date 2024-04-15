//
//  RC controller
//
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_wifi.h"
#include <L298NX2.h>

#define LED 2

const unsigned int EN_A = 3;
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;
const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

enum curren_state {
  ON_STATE,
  OFF_STATE,
};

enum cmd_code {
  ON,
  OFF,
  TURN,
  STOP,
  FORWARD,
  BACKWARD,
  SET_TURN,
};


AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
// wifi名称
const char *ssid = "RC_C";
// wifi 密码
const char *password = "15827330290";
unsigned long previousMillis = 0;
unsigned long turnSpeed = 0;
unsigned long previousMillisLed = 0;
unsigned long interval = 10000;
unsigned long intervalLed = 200;  // 未连接wifi 200ms 闪烁 连接wifi未连接遥控 1500ms 闪烁 都连接 长亮
int led_state = 0;
bool reconning = false;

curren_state current = OFF_STATE;

// uint8_t to string
String converter(uint8_t *str) {
  return String((char *)str);
}

cmd_code hashit(String inString) {
  if (inString == "ON")
    return ON;
  if (inString == "OFF")
    return OFF;
  if (inString == "STOP")
    return STOP;
  if (inString == "TURN")
    return TURN;
  if (inString == "FORWARD")
    return FORWARD;
  if (inString == "BACKWARD")
    return BACKWARD;
  if (inString == "SET_TURN")
    return SET_TURN;
  return STOP;
}

// 启动引擎
void launchEngine() {
  Serial.println("launch engine");
  current = ON_STATE;
}

// 关闭引擎
void closeEngine() {
  if (current == OFF_STATE)
    return;
  Serial.println("close engine");
  current = OFF_STATE;
}

// 电机停止
void motorStop() {
  if (current == OFF_STATE)
    return;
  motors.stop();
}

// 电机正转
void motorForward(int val) {
  if (current == OFF_STATE)
    return;
  motors.forward();
}

// 电机反转
void motorBackward(int val) {
  if (current == OFF_STATE)
    return;
  motors.backward();
}

// 处理远程命令
void onCommond(JsonDocument obj) {
  String CMD = String((const char *)obj["COMMOND"]);
  int val = (int)obj["VALUE"];

  Serial.print(CMD);
  Serial.print(" -- ");
  Serial.print(val);
  Serial.println();

  switch (hashit(CMD)) {
    case ON:
      launchEngine();
      break;
    case OFF:
      closeEngine();
      break;
    case STOP:
      motorStop();
      break;
    case FORWARD:
      motorForward(val);
      break;
    case BACKWARD:
      motorBackward(val);
      break;
    case SET_TURN:
      turnSpeed = val;  // 转动速度
      break;
    case TURN:
      if (current == OFF_STATE)
        return;
      motorStop();
      if (val > 0) {  // 右转
        motors.setSpeedA(turnSpeed);
        motors.forwardA();
      } else {
        motors.setSpeedA(turnSpeed);
        motors.forwardB();
      }
      break;
  }
}

// websocket 事件监听
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  JsonDocument doc;
  switch (type) {
    case WS_EVT_CONNECT:
      intervalLed = -1;
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      closeEngine();
      intervalLed = 1500;
      break;
    case WS_EVT_DATA:
      DeserializationError error = deserializeJson(doc, converter(data));

      // Test if parsing succeeds.
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }
      onCommond(doc);
      break;
  }
}

// 初始化websocket
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

// 初始化wifi
void initWifi() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  intervalLed = 1500;
}

// 初始化引脚
void initPin() {
  pinMode(LED, OUTPUT);
}

void checkWifiState() {
  unsigned long currentMillis = millis();
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    closeEngine();
    intervalLed = 200;
    reconning = true;
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }

  if (reconning && WiFi.status() == WL_CONNECTED) {
    intervalLed = 1500;
    reconning = false;
    Serial.println(WiFi.localIP());
    Serial.println("reconnect success");
  }
}

void checkLed() {
  if (intervalLed == -1 && led_state == 0) {
    digitalWrite(LED, HIGH);
    led_state = 1;
    return;
  }
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisLed >= intervalLed) {
    if (led_state == 0) {
      digitalWrite(LED, HIGH);
      led_state = 1;
    } else {
      digitalWrite(LED, LOW);
      led_state = 0;
    }
    previousMillisLed = currentMillis;
  }
}

void setup() {
  initPin();
  Serial.begin(115200);
  initWifi();
  esp_wifi_set_max_tx_power(1);
  initWebSocket();
  server.begin();
}

void loop() {
  checkWifiState();
  checkLed();
}