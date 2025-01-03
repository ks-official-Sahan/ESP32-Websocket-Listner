#include <map>
#include <L298NX2.h>  // For two motors instance at once
#include <WiFi.h>
#include <WebSocketsClient.h>

/* PINs declaration */
const int LED_PIN = 15;

const int ENA_PIN = 33;  // OUT 1: ADC PINS (PMW PINS)
const int IN1_PIN = 14;
const int IN2_PIN = 12;
const int ENB_PIN = 25;  // OUT 2: ADC PINS (PMW PINS)
const int IN3_PIN = 26;
const int IN4_PIN = 27;

/* Components */
L298NX2 motors = L298NX2(ENA_PIN, IN1_PIN, IN2_PIN, ENB_PIN, IN3_PIN, IN4_PIN);

/* Connection Constants and Globals */
#define WIFI_SSID "Sahan-Fiber-SLT"
#define WIFI_PASSWORD "King7f2d!@#$"
// #define WIFI_SSID = "Sahan's A54";
// #define WIFI_PASSWORD = "Sahan123";

const char* serverAddress = "774e-2402-d000-8128-40d-65a3-4c09-28f0-ef8.ngrok-free.app";  // ngrok public URL (without "https://")
const char* serverPath = "/ESP_APP/esp";                                                  // WebSocket endpoint
const int serverPort = 80;                                                                // port 80 by default for http and ws
const int serverPortSecure = 443;                                                         // For https and wss

/* Websocket variables & constants */
WebSocketsClient webSocket;

unsigned long lastServerMessageTime = 0;  // Track time of the last message
const unsigned long connectionTimeout = 5 * 60 * 1000;
const unsigned long reconnectDelay = 4 * 1000;

/* Default Mechanism */
void setup() {
  initConfig();

  connectToWiFi();

  startWebSocket();
}

void loop() {
  webSocket.loop();

  /* Periodically check the connection status */
  checkWebSocketConnection();
}

void initConfig() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  lightON();
  setSpeed(255);
}

void checkWebSocketConnection() {
  unsigned long now = millis();
  if (now - lastServerMessageTime > connectionTimeout) {
    Serial.println("Server connection timed out. Reconnecting...");
    reconnectWebSocket();
    lastServerMessageTime = now;  // To avoid spamming reconnect
  }
}

/* Websocket */
void startWebSocket() {
  Serial.println("Starting WebSocket...");
  /* Initialize WebSocket connection URL */
  // webSocket.begin(serverAddress, serverPort, serverPath); // non secured (http | ws)
  webSocket.beginSSL(serverAddress, serverPortSecure, serverPath);  // secured (https | wss)
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(reconnectDelay);  // Reconnect if disconnected
  webSocket.enableHeartbeat(5000, 3000, 3);        // Ping every 5 seconds, timeout after 3 tries
}

void reconnectWebSocket() {
  Serial.printf("[%lu] Reconnecting WebSocket to %s:%d%s...\n", millis(), serverAddress, serverPortSecure, serverPath);
  if (webSocket.isConnected()) {
    webSocket.disconnect();
  }
  startWebSocket();
}

void refreshConnectionTimer() {
  lastServerMessageTime = millis();
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      lightOFF();
      break;

    case WStype_CONNECTED:
      Serial.println("WebSocket Connected");
      webSocket.sendTXT("ESP32 Connected!");
      refreshConnectionTimer();
      indicateLight(5);
      break;

    case WStype_TEXT:
      {
        String command = String((char*)payload);
        Serial.printf("Command received: %s\n", command.c_str());

        runCommand(command);
        lastServerMessageTime = millis();
        break;
      }

    case WStype_ERROR:
      Serial.println("WebSocket Error Occurred");
      break;

    case WStype_PONG:
      Serial.println("Ping Working");
      refreshConnectionTimer();
      break;
    case WStype_PING:
      refreshConnectionTimer();
      Serial.println("Ping received");
      break;

    default:
      Serial.printf("Unhandled WebSocket event type: %d\n", type);
      break;
  }
}

/* Motors */
void setSpeed(unsigned short speed) {
  if (speed > 255) speed = 255;
  motors.setSpeed(speed);
}

void checkMotorAction(unsigned long actionTime, unsigned long timeout = 2000) {
  if ((motors.isMovingA() || motors.isMovingB()) && millis() - actionTime >= timeout) {
    motors.stop();
    Serial.println("Motors stopped due to timeout.");
  }
}

/* Actions */
typedef void (*CommandHandler)();

std::map<String, CommandHandler> commandMap = {
  { "TURN_ON", runAction1 },
  { "TURN_OFF", runAction2 },
  { "Action A", runAction3 },
  { "Action B", runAction4 },
  { "Action C", runAction5 },
  { "Action D", runAction6 },
  { "Action E", runAction7 },
};

void runCommand(String command) {
  if (commandMap.find(command) != commandMap.end()) {
    commandMap[command]();
  } else {
    Serial.printf("Unknown command: %s\n", command.c_str());
    // webSocket.sendTXT(String("Unknown command: ") + command);
  }
}

void runAction1() {
  lightON();
}

void runAction2() {
  lightOFF();
}

void runAction3() {
  motors.forward();
  Serial.println("Action A");
  // checkMotorAction(millis(), 10 * 1000);
}

void runAction4() {
  motors.backward();
  Serial.println("Action B");
  // checkMotorAction(millis(), 10 * 1000);
}

void runAction5() {
  // checkMotorAction(millis(), 10);
  Serial.println("Action C");
  if (motors.isMovingA() || motors.isMovingB()) {
    motors.stop();
  }
}

void runAction6() {
  /* Left */
  motors.forwardB();
  motors.backwardA();
  Serial.println("Action D");
  checkMotorAction(millis());
}

void runAction7() {
  /* Right */
  motors.forwardA();
  motors.backwardB();
  Serial.println("Action E");
  checkMotorAction(millis());
}

/* Lights */
void lightON() {
  Serial.println("Light ON");
  digitalWrite(LED_PIN, HIGH);
}

void lightOFF() {
  Serial.println("Light OFF");
  digitalWrite(LED_PIN, LOW);
}

void indicateLight(int limit) {
  static bool lightState = false;

  for (int blinkCount = 0; blinkCount < 2 * limit; blinkCount++) {
    lightState = !lightState;
    digitalWrite(LED_PIN, lightState ? HIGH : LOW);
    delay(200);
  }
}

/* WiFi */
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long startAttemptTime = millis();
  const unsigned long timeoutWiFi = 30 * 1000;  // 30 seconds

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeoutWiFi) {
    delay(1000);
    Serial.println("Connecting...");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());
    lightOFF();
  } else {
    Serial.println("Failed to connect to WiFi. Restarting...");
    ESP.restart();
  }
}