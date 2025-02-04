#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <DNSServer.h>

// Конфигурация
#define RELAY_PIN_COUNT 8
#define MQTT_CLIENT_ID "ESP8266_Relay_Controller"
#define MQTT_TOPIC_PREFIX "home/relays/"
#define AP_SSID "RelayController"
#define AP_PASS "config1234"

const uint8_t relayPins[RELAY_PIN_COUNT] = {16, 4, 14, 12, 2, 15, 3, 0};
bool relayStates[RELAY_PIN_COUNT] = {false};
bool isMyMessage = false;
bool lastStates[RELAY_PIN_COUNT] = {false};
bool firstPublish = true;

// Настройки сети (подгружаются из файла)
String ssid = "";
String password = "";
String mqtt_server = "mqtt.eclipseprojects.io";
String mqtt_user = "";
String mqtt_password = "";

WiFiClient wifiClient;
PubSubClient client(wifiClient);
AsyncWebServer server(80);
DNSServer dns;

bool shouldReboot = false;
unsigned long lastMqttReconnect = 0;

void setup();
void loop();
void loadConfiguration();
void loadRelayStates();
void saveRelayStates();
void handleWiFiConnection();
void startAPMode();
void handleMqttConnection();
void mqttReconnect();
void subscribeToTopics();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void updateRelayState(uint8_t relay, bool state);
void publishRelayStates();
void publishSingleRelayState(uint8_t relay);
void handleRoot(AsyncWebServerRequest *request);
void handleConfig(AsyncWebServerRequest *request);
void handleControl(AsyncWebServerRequest *request);
void handleSaveSettings(AsyncWebServerRequest *request);
void handleRelayRequest(AsyncWebServerRequest *request);
void handleRelayStateRequest(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void handleDNSServer();

void setup() {
  Serial.begin(115200);

  // Инициализация пинов реле
  for (int i = 0; i < RELAY_PIN_COUNT; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }

  // Инициализация файловой системы
  if (!LittleFS.begin()) {
    Serial.println("[ERR] LittleFS initialization failed!");
    return;
  }

  loadConfiguration();
  loadRelayStates();

  // Инициализируем lastStates актуальными значениями
  for (int i = 0; i < RELAY_PIN_COUNT; i++) {
    lastStates[i] = relayStates[i];
  }

  // Настройка WiFi (по умолчанию – режим STA)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  // Настройка MQTT
  client.setServer(mqtt_server.c_str(), 1883);
  client.setCallback(mqttCallback);

  // Маршруты веб-сервера
  // Главная страница – можно задать любой из маршрутов, здесь по умолчанию перенаправим на /control
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->redirect("/control");
  });

  // Страница настроек (для режима AP, но доступна по маршруту /config)
  server.on("/config", HTTP_GET, handleConfig);
  // Страница управления реле (для режима STA)
  server.on("/control", HTTP_GET, handleControl);

  // Сохранение настроек (используется на странице настроек)
  server.on("/save", HTTP_POST, handleSaveSettings);

  // Обработчик переключения реле (при нажатии кнопок)
  server.on("/relay", HTTP_GET, handleRelayRequest);

  // Обработчик для получения состояния реле (возвращает JSON)
  server.on("/relaystate", HTTP_GET, handleRelayStateRequest);

  server.onNotFound(handleNotFound);

  // DNS-сервер для режима AP
  dns.start(53, "*", IPAddress(192, 168, 4, 1));
  server.begin();
}

void loop() {
  handleWiFiConnection();
  handleMqttConnection();
  handleDNSServer();

  if (shouldReboot) {
    delay(1000);
    ESP.restart();
  }
  
  delay(10);
}

// Обработка WiFi
void handleWiFiConnection() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 10000) return;
  lastCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Connecting...");
    WiFi.reconnect();
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      // Если не удалось подключиться, переходим в режим AP
      startAPMode();
    }
  }
}

void startAPMode() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WIFI] AP Mode: %s\n", WiFi.softAPIP().toString().c_str());
}

// Обработка MQTT
void handleMqttConnection() {
  if (!client.connected() && millis() - lastMqttReconnect > 5000) {
    mqttReconnect();
    lastMqttReconnect = millis();
  }
  client.loop();
}

void mqttReconnect() {
  Serial.println("[MQTT] Connecting...");
  if (client.connect(MQTT_CLIENT_ID, mqtt_user.c_str(), mqtt_password.c_str())) {
    Serial.println("[MQTT] Connected!");
    subscribeToTopics();
    firstPublish = true;
    publishRelayStates();
  } else {
    Serial.printf("[MQTT] Failed: %d\n", client.state());
  }
}

void subscribeToTopics() {
  for (int i = 0; i < RELAY_PIN_COUNT; i++) {
    char topic[32];
    snprintf(topic, sizeof(topic), MQTT_TOPIC_PREFIX "relay%d", i);
    client.subscribe(topic);
    Serial.printf("[MQTT] Subscribed: %s\n", topic);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (isMyMessage || length < 2 || length > 3) return;

  String message;
  for (unsigned i = 0; i < length; i++) {
    message += (char)toupper(payload[i]);
  }
  message.trim();

  if (message != "ON" && message != "OFF") {
    Serial.printf("[MQTT] Invalid command: %s\n", message.c_str());
    return;
  }

  for (int i = 0; i < RELAY_PIN_COUNT; i++) {
    char t[32];
    snprintf(t, sizeof(t), MQTT_TOPIC_PREFIX "relay%d", i);
    if (String(topic) == t) {
      updateRelayState(i, message == "ON");
      break;
    }
  }
}

void updateRelayState(uint8_t relay, bool state) {
  if (relay >= RELAY_PIN_COUNT) return;
  if (relayStates[relay] != state) {
    relayStates[relay] = state;
    digitalWrite(relayPins[relay], state ? HIGH : LOW);
    saveRelayStates();
    publishSingleRelayState(relay);
    Serial.printf("[RELAY] %d: %s\n", relay, state ? "ON" : "OFF");
  }
}

void publishRelayStates() {
  isMyMessage = true;
  for (int i = 0; i < RELAY_PIN_COUNT; i++) {
    if (firstPublish || lastStates[i] != relayStates[i]) {
      publishSingleRelayState(i);
      lastStates[i] = relayStates[i];
    }
  }
  firstPublish = false;
  isMyMessage = false;
}

void publishSingleRelayState(uint8_t relay) {
  if (relay >= RELAY_PIN_COUNT) return;
  char topic[32];
  snprintf(topic, sizeof(topic), MQTT_TOPIC_PREFIX "relay%d", relay);
  client.publish(topic, relayStates[relay] ? "ON" : "OFF", false);
  Serial.printf("[MQTT] Published: %s -> %s\n", topic, relayStates[relay] ? "ON" : "OFF");
}

// Загрузка и сохранение настроек
void loadConfiguration() {
  File configFile = LittleFS.open("/config.json", "r");
  if (configFile) {
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, configFile);
    if (!error) {
      ssid = doc["ssid"].as<String>();
      password = doc["password"].as<String>();
      mqtt_server = doc["mqtt_server"].as<String>();
      mqtt_user = doc["mqtt_user"].as<String>();
      mqtt_password = doc["mqtt_password"].as<String>();
    }
    configFile.close();
  }
  if (mqtt_server.isEmpty()) mqtt_server = "mqtt.eclipseprojects.io";
}

void loadRelayStates() {
  File stateFile = LittleFS.open("/states.json", "r");
  if (stateFile) {
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, stateFile);
    if (!error) {
      for (int i = 0; i < RELAY_PIN_COUNT; i++) {
        relayStates[i] = doc["relay" + String(i)];
        digitalWrite(relayPins[i], relayStates[i] ? HIGH : LOW);
      }
    }
    stateFile.close();
  }
}

void saveRelayStates() {
  DynamicJsonDocument doc(512);
  for (int i = 0; i < RELAY_PIN_COUNT; i++) {
    doc["relay" + String(i)] = relayStates[i];
  }
  File stateFile = LittleFS.open("/states.json", "w");
  if (stateFile) {
    serializeJson(doc, stateFile);
    stateFile.close();
  }
}

// Обработчики страниц

// Обработчик страницы настроек (/config)
void handleConfig(AsyncWebServerRequest *request) {
  File file = LittleFS.open("/config.html", "r");
  if (!file) {
    request->send(404, "text/plain", "Файл не найден");
    return;
  }
  String html = file.readString();
  file.close();
  html.replace("{{ ssid }}", ssid);
  html.replace("{{ password }}", password);
  html.replace("{{ mqtt_server }}", mqtt_server);
  html.replace("{{ mqtt_user }}", mqtt_user);
  html.replace("{{ mqtt_password }}", mqtt_password);
  request->send(200, "text/html", html);
}

// Обработчик страницы управления реле (/control)
void handleControl(AsyncWebServerRequest *request) {
  File file = LittleFS.open("/control.html", "r");
  if (!file) {
    request->send(404, "text/plain", "Файл не найден");
    return;
  }
  String html = file.readString();
  file.close();
  request->send(200, "text/html", html);
}

// Сохранение настроек (используется на странице /config)
void handleSaveSettings(AsyncWebServerRequest *request) {
  ssid = request->arg("ssid");
  password = request->arg("password");
  mqtt_server = request->arg("mqtt_server");
  mqtt_user = request->arg("mqtt_user");
  mqtt_password = request->arg("mqtt_password");

  DynamicJsonDocument doc(512);
  doc["ssid"] = ssid;
  doc["password"] = password;
  doc["mqtt_server"] = mqtt_server;
  doc["mqtt_user"] = mqtt_user;
  doc["mqtt_password"] = mqtt_password;

  File configFile = LittleFS.open("/config.json", "w");
  if (configFile) {
    serializeJson(doc, configFile);
    configFile.close();
  }

  request->send(200, "text/plain", "Настройки сохранены. Перезагрузка...");
  shouldReboot = true;
}

// Обработчик переключения реле (/relay)
void handleRelayRequest(AsyncWebServerRequest *request) {
  if (!request->hasArg("num") || !request->hasArg("state")) {
    request->send(400, "text/plain", "Отсутствуют параметры");
    return;
  }
  int relay = request->arg("num").toInt();
  String state = request->arg("state");
  state.toUpperCase();
  if (state != "ON" && state != "OFF") {
    request->send(400, "text/plain", "Неверное состояние");
    return;
  }
  updateRelayState(relay, state == "ON");
  request->send(200, "text/plain", "Реле обновлено");
}

// Обработчик получения состояния реле (/relaystate)
// Возвращает JSON: { "state": "ON" } или { "state": "OFF" }
void handleRelayStateRequest(AsyncWebServerRequest *request) {
  if (!request->hasArg("num")) {
    request->send(400, "text/plain", "Отсутствует параметр num");
    return;
  }
  int relay = request->arg("num").toInt();
  if (relay < 0 || relay >= RELAY_PIN_COUNT) {
    request->send(400, "text/plain", "Неверный номер реле");
    return;
  }
  String stateStr = relayStates[relay] ? "ON" : "OFF";
  request->send(200, "application/json", "{\"state\": \"" + stateStr + "\"}");
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Не найдено");
}

void handleDNSServer() {
  if (WiFi.getMode() == WIFI_AP) {
    dns.processNextRequest();
  }
}
