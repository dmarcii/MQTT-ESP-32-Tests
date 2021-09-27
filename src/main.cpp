#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <painlessMesh.h>
#include <WiFiClient.h>

#define WIFI_SSID "marci"
#define WIFI_PASSWORD "marcimarci"
#define MQTT_HOST IPAddress(192, 168, 0, 110)
#define MQTT_PORT 1883


#define   MESH_PREFIX     "ESP32 Mesh"
#define   MESH_PASSWORD   "1234567890"
#define   MESH_PORT       5555
#define HOSTNAME "MQTT_Mesh"

painlessMesh  mesh;
WiFiClient wifiClient;

void receivedCallback( const uint32_t &from, const String &msg ) {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  String topic = "painlessMesh/from/" + String(from);
}

// Create objects to handle MQTT client
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

//Led pin
int pin = 19;



void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// Add more topics that want your ESP32 to be subscribed to
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  // ESP32 subscribed to topic
  uint16_t packetIdSub = mqttClient.subscribe("topic/OnOff", 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  Serial.println(messageTemp);
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  // Check if the MQTT message was received on topic 
  if (strcmp(topic, "topic/OnOff") == 0) {
    Serial.println("Mensage Recibido");

    if (messageTemp == "on") {
      digitalWrite(pin, HIGH);

    }
    else if (messageTemp == "off")
    {
      digitalWrite(pin, LOW);
      //mqttClient.publish("topic/OnOff", 2, true, "Apagado");
    }

    if (strcmp(topic, "topic/state") == 0) {
    //mesh.isConnected();
    Serial.println("Mensage Recibido");
  }
  Serial.print("  total: ");
}

}

void setup() {
  
  Serial.begin(115200);
  pinMode(pin, OUTPUT);

  /* Mesh Config */

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  
  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 6 );
  mesh.onReceive(&receivedCallback);
  mesh.stationManual(WIFI_SSID, WIFI_PASSWORD);
  mesh.setHostname(HOSTNAME);
  mesh.setRoot(true);
  mesh.setContainsRoot(true);
  //End mesh Setup Callbacks


// MQTT
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

}

void loop() {
    mesh.update();
    /* uint16_t packetIdPub2 = mqttClient.publish("topic/hello", 2, true, "siuuuuu");
    uint16_t packetIdPub3 = mqttClient.publish("topic/hello2", 2, true, "El bicho");
    Serial.print("Publishing on topic at QoS 2, packetId: ");
    Serial.println(packetIdPub2);
    */
    delay(5000);
}






