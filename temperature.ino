#include <Arduino_JSON.h>

#include <map>
#include <WiFi.h>
#include <WebSocketsClient.h>

#include "DHT.h"

#define DPIN 26      // Pin to connect DHT sensor (GPIO number)
#define DTYPE DHT11  // Define DHT 11 or DHT22 sensor type

/* PINs declaration */
const int LED_PIN = 15;

/* Connection Constants and Globals */
// #define WIFI_SSID = "Tharindu_D";
// #define WIFI_PASSWORD = "19970319";
#define WIFI_SSID "Sahan's A54"
#define WIFI_PASSWORD "Sahan123"

const char* serverAddress = "f20b-103-21-165-128.ngrok-free.app";  // ngrok public URL (without "https://")
const char* serverPath = "/ESP_APP/esp";                           // WebSocket endpoint
const int serverPort = 80;                                         // port 80 by default for http and ws
const int serverPortSecure = 443;                                  // For https and wss

/* Websocket variables & constants */
WebSocketsClient webSocket;

unsigned long lastServerMessageTime = 0;  // Track time of the last message
const unsigned long connectionTimeout = 5 * 60 * 1000;
const unsigned long reconnectDelay = 4 * 1000;

DHT dht(DPIN, DTYPE);

void getSensorData() {
  float temperature = dht.readTemperature(false);
  float humidity = dht.readHumidity();
  Serial.println(temperature);
  Serial.println(humidity);

  JSONVar sensorData;
  sensorData["temperature"] = temperature;
  sensorData["humidity"] = humidity;

  // JSONVar temperatureData;
  // temperatureData["temperature"] = temperature;

  // JSONVar humidityData;
  // humidityData["humidity"] = humidity;

  String data = JSON.stringify(sensorData);
  webSocket.sendTXT(data);

  // String temperatureJSON = JSON.stringify(temperatureData);
  // String humidityJSON = JSON.stringify(humidityData);
  // webSocket.sendTXT(temperatureJSON);
  // webSocket.sendTXT(humidityJSON);
  delay(1000);
}

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
  if (webSocket.isConnected()) {
    getSensorData();
  }
}

void initConfig() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  lightON();
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
      lightON();
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

/* Actions */
typedef void (*CommandHandler)();

std::map<String, CommandHandler> commandMap = {
  { "TURN_ON", runAction1 },
  { "TURN_OFF", runAction2 },
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

/* Lights */
void lightON() {
  Serial.println("Light ON");
  digitalWrite(LED_PIN, HIGH);
}

void lightOFF() {
  Serial.println("Light OFF");
  digitalWrite(LED_PIN, LOW);
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