#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <rdm6300.h>
#include "esp_task_wdt.h"
#include "esp_wifi.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "DFRobotDFPlayerMini.h"
#include <Preferences.h>

DFRobotDFPlayerMini player;
Preferences prefer;
/* SH8 SH12 Audio*/
// #define GET_READY_FOR_USE      7
// #define DATA_UPDATE            10
// #define DATA_UPDATE_FAILED     11
// #define WELCOME_TO_SUPER_HOME  18
// #define SEE_YOU_AGAIN          15
// #define POWER_CONNECTED        9
// #define POWER_DISCONNECTED     17
// #define EMERGENCY_OPENING      16
// #define EMERGENCY_CLOSING      8
// #define MACHINE_READY_FOR_USE  14
// #define WRONG_WAY              4
// #define STEP_BACK              5
// #define ACCESS_DENIED          6

// EIND
/* SH8 SH08 Audio*/
#define GET_READY_FOR_USE      7
#define DATA_UPDATE            12
#define DATA_UPDATE_FAILED     13
#define WELCOME_TO_SUPER_HOME  1
#define SEE_YOU_AGAIN          17
#define POWER_CONNECTED        11
#define POWER_DISCONNECTED     10
#define EMERGENCY_OPENING      9
#define EMERGENCY_CLOSING      8
#define MACHINE_READY_FOR_USE  16
#define WRONG_WAY              4
#define STEP_BACK              5
#define ACCESS_DENIED          6

/* Audio Sound List SH6 SH06 */
// #define GET_READY_FOR_USE       7   
// #define DATA_UPDATE             12    
// #define DATA_UPDATE_FAILED      13   
// #define WELCOME_TO_SUPER_HOME    1
// #define SEE_YOU_AGAIN           17    
// #define POWER_CONNECTED         11  
// #define POWER_DISCONNECTED      10  
// #define EMERGENCY_OPENING        9   
// #define EMERGENCY_CLOSING        8     
// #define MACHINE_READY_FOR_USE    16
// #define WRONG_WAY                4
// #define STEP_BACK                5
// #define ACCESS_DENIED            6

/* SH7 SH07
1. Wrong way 
2. Step away
3. Access denied try again 
4. The machine is geating ready please wait 
5. E closing 
6. E opening 
7. P disconnected 
8. P connected 
9.Data updated 
10.Data failed to update
11. Take care 
12. Welcome to eng 
13. Machine is ready for use 
14. See you again 
15.Welcome to superhostel 
16.Thank you for using 
17. This machine is developing by rakib .
*/

/* Audio Sound List SH09 SH9 */
// #define GET_READY_FOR_USE       6
// #define DATA_UPDATE             11    
// #define DATA_UPDATE_FAILED      12   
// #define WELCOME_TO_SUPER_HOME    1
// #define SEE_YOU_AGAIN           16    
// #define POWER_CONNECTED         10  
// #define POWER_DISCONNECTED      9  
// #define EMERGENCY_OPENING        8  
// #define EMERGENCY_CLOSING        7   
// #define MACHINE_READY_FOR_USE    15
// #define WRONG_WAY                17
// #define STEP_BACK                4
// #define ACCESS_DENIED            5


// ---------- Pins ----------
const int ST_LED = 10;
// ---------- One-time offline restart flag (survives ESP.restart) ----------
RTC_DATA_ATTR bool rtc_offlineRestarted = false;

// ---------- Globals ----------
WiFiManager wm;

volatile bool led_ST = false;          // true == connected
String Chipid = String(ESP.getEfuseMac(), DEC);
String ssid = "Entrance-gate:" + Chipid;
String read_data = "";
const char* File_Name = "/card.txt";

// RDM_Card Managment
#define P_RDM_INSIDE 2
#define P_RDM_OUTSIDE 1 
// Input 
#define P_R_OPEN_S 3
#define P_R_CLOSE_S 8

#define P_IR_INSIDE 35
#define P_IR_OUTSIDE 36

#define P_L_DOOR_CLOSE_S 37
#define P_R_DOOR_CLOSE_S 38

#define P_L_DOOR_OPEN_S 39
#define P_R_DOOR_OPEN_S 40

#define AC_POWER 21 //Now 21
//Direction LED
#define INTER_SIGNAL 41
#define EXIT_SIGNAL 42

// Motor Controll //Motor Driver 
#define PWM1_L 4
#define PWM2_L 5
#define PWM3_R 6
#define PWM4_R 7

Rdm6300 RDM_INSIDE;
Rdm6300 RDM_OUTSIDE;

const char* api_secret = "";

unsigned long M_Fault_Time_OUT = 3000;
int PWM_SPEED_L = 180;
int PWM_SPEED_R = 180;

bool INSIDE_IR_STATE = false,OUTSIDE_IR_STATE = false,L_M_CLOSE = false,R_M_CLOSE = false,L_M_OPEN = false,R_M_OPEN = false,INSIDE_CARD = false,OUTSIDE_CARD = false,in_to_out = false,out_to_in = false,openState = false,closeState = false,IN_IR_HIGH = false,OUT_IR_HIGH = false,L_OPEN_GIVEN = false,R_OPEN_GIVEN = false,L_CLOSE_GIVEN = false,R_CLOSE_GIVEN = false;
bool L_DOOR_OPEN_DONE = false,R_DOOR_OPEN_DONE = false,L_DOOR_CLOSE_DONE = false,R_DOOR_CLOSE_DONE = false;
int AC_POWER_STATE = 2; // 1 = POWER AVAILAVLE, 0 = POWER_NOT_AVAILAVLE;

// Reconnect policy (hourly batches)
const uint8_t  MAX_ATTEMPTS_PER_BATCH = 10;
const uint8_t  MAX_FAILED_BATCHES_BEFORE_RESTART = 5;
const unsigned long ATTEMPT_SPACING_MS = 2000;
const unsigned long BATCH_COOLDOWN_MS = 3600000UL;
const unsigned long CONNECT_TRY_MS     = 10000;

//
unsigned long doorTimeout = 8000;

// Reconnect state
volatile uint8_t batchAttempts = 0;
volatile uint8_t failedBatches = 0;
unsigned long batchStartMs = 0;
unsigned long lastAttemptMs = 0;
bool batchActive = false,skiploop = false;

// Cross-core flags / states
volatile bool load_succes = false;

// NEW: schedule NetWorker immediately when needed
volatile bool request_sync_now = false;

// Timers (TRUE 30 min)
const unsigned long delayTime   = 30UL * 60UL * 1000UL; // 30 minutes periodic
const unsigned long delayData   = 30000;                // 30s first-load retry
unsigned long lastEarlySync = 0;
unsigned long lastPeriodicSync = 0;

// Door/tones timing
volatile unsigned long puncTime = 0;
volatile unsigned long openTime = 0;
volatile unsigned long closeTime = 0;
volatile unsigned long BUZZER_DELAY = 0;
volatile unsigned long BUZZER_FAULT_S_DELAY = 0;
volatile unsigned long BUZZER_FAULT_BLINK_DELAY = 0;

// Master list (BOUND FIX)
String m_cardArray[14] = {
  "0005254657","0005132457","0005119029","0005134642","0005128435","0005118248","0000166918",
  "0005128383","0005133906","0000252720","0005147289","0005132417","0005147839","0005136998"
};
const size_t M_CARD_COUNT = sizeof(m_cardArray)/sizeof(m_cardArray[0]);

// ---------- Net scheduling & backoff (like glass door) ----------
TaskHandle_t core0Handle = NULL;
TaskHandle_t core1Handle = NULL;
TaskHandle_t netTaskHandle = NULL;
volatile bool netTaskBusy = false;

unsigned long nextSyncDueMs = 0;
const unsigned long BACKOFF_MIN = 60000;   // 1 min
const unsigned long BACKOFF_MAX = 900000;  // 15 min
unsigned long backoffMs = BACKOFF_MIN;
bool inBackoff = false;

// NEW: restart trigger after deny tone if offline unknown card
volatile bool restartPending = false,FS_Faulty_update_requied = false;

// ........................................................... MQTT Setup.......................................................................
volatile bool MQTT_OPEN_REQ = false,MQTT_CLOSE_REQ = false;
#define MQTT_TOPIC_LEN   128
#define MQTT_PAYLOAD_LEN 512

QueueHandle_t mqttQueue;

typedef struct {
  char topic[MQTT_TOPIC_LEN];
  char payload[MQTT_PAYLOAD_LEN];
  bool retained;
} mqtt_msg_t;


unsigned long mqtttryTime = 0;
unsigned long mqtttryConnectDelay = 30000;
// ====== MQTT Broker ====== IP "" is only for SH8
const char* mqttServer = ""; /*" "*/
const int mqttPort = 1883;

// MQTT Username/Password
const char* mqttUser = "";
const char* mqttPassword = "";
String infoTopic = "devices/" + Chipid + "/fs";
String eventTopic = "devices/" + Chipid + "/event";
String statusTopic = "devices/" + Chipid + "/status";

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// Unique Client ID using ESP32 MAC address
// String clientId;

// Device status topic

// .........................................................................MQTT Setup End..............................................................
// ---------- Forward declarations ----------
void core0Task(void *parameter);
void core1Task(void *parameter);
void NetWorker(void *parameter);
void reconnectWiFi();
void updateWifiLed();
// Net helpers (run inside NetWorker)
// Motor logic (UNCHANGED)
void open();
void close();
void opening();
void closing();
// ====== Callback ======
bool mqttPublishQueue(const char* topic, const char* payload, bool retained) {
  if (mqttQueue == NULL) return false;

  mqtt_msg_t msg;
  mqtt_msg_t dump;   // ✅ declare dump

  memset(&msg, 0, sizeof(msg));

  strncpy(msg.topic, topic, MQTT_TOPIC_LEN - 1);
  strncpy(msg.payload, payload, MQTT_PAYLOAD_LEN - 1);
  msg.retained = retained;

  // Try to send
  if (xQueueSend(mqttQueue, &msg, 0) != pdTRUE) {
    // Queue full → drop oldest
    xQueueReceive(mqttQueue, &dump, 0);
    // Retry send
    if (xQueueSend(mqttQueue, &msg, 0) != pdTRUE) {
      return false; // still failed
    }
  }

  return true;
}

void update(const String& msg) {
  if (mqttPublishQueue(eventTopic.c_str(), msg.c_str(), false)) {
    Serial.println("[MQTT] Feedback update queued: " + eventTopic);
  } else {
    Serial.println("[MQTT] Feedback update FAILED: " + eventTopic);
  }
}

void handleGet() {
  Serial.println("[CMD] GET");
  update("[CMD] GET");
}
void handleReset() {
  Serial.println("[CMD] RESET");
  update("[CMD] RESET");
  wm.resetSettings();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  ESP.restart();
}

void handleRun() {
  Serial.println("[CMD] RUN");
  update("[CMD] RUN");
  // motorRun();
}

void handleClose() {
  MQTT_CLOSE_REQ = true;
  Serial.println("[CMD] Close");
  update("[CMD] STOP"); // old [CMD] STOP
  // motorStop();
}

void handleOpen() {
  MQTT_OPEN_REQ = true;
  update("[ACCESS] Allowed (MQTT CMD). Opening...");
  Serial.println("[ACCESS] Allowed (MQTT CMD). Opening...");
}

void handleSingleCard(JsonDocument &doc) {
  if (!doc["val"]) {
    Serial.println("[CMD] card: val missing");
    update("[CMD] card: val missing");
    return;
  }

  String card = doc["val"].as<String>();
  card.trim();
  Serial.print("[CMD] Single card: ");
  Serial.println(card);

  File file = LittleFS.open(File_Name, "w");
  if (file) {
    file.println(card);
    Serial.println("LittleFS Stored Single Carde.");
    update("LittleFS Stored Single Carde.");
  } else {
    update("LittleFS Failed to open for write.");
    Serial.println("[MQTT] LittleFS Failed to open for write: " + eventTopic);
  }
  file.close();
}

void handleCards(JsonDocument &doc) {
  if (!doc["val"].is<JsonArray>()) {
    Serial.println("[CMD] cards: val is not array");
    return;
  }
  
  JsonArray cardNumbers = doc["val"].as<JsonArray>();
  
  // Only overwrite if we actually have data
  if (cardNumbers.size() > 0) {
      File file = LittleFS.open(File_Name, "w");
      if (file) {
        for (JsonVariant v : cardNumbers) {
          String s = v.as<String>(); 
          s.trim();
          if(s.length() > 0) { // Don't save empty strings
            file.println(s);
          }
        }
        file.close();
        load_succes = true;
        update("LittleFS Stored All Cards.");
        Serial.println("Cards saved successfully.");
      } else {
        Serial.println("[FS] Failed to open file for write.");
        update("LittleFS Failed to open for write.");
      }
  } else {
      Serial.println("[CMD] Received empty card list, ignoring.");
  }
}
void loadSpeed(int lval,int rval){
  prefer.begin("state",false);
  prefer.putUInt("l_speed", lval);
  prefer.putUInt("r_speed", rval);
  prefer.end();
  PWM_SPEED_L = lval;
  PWM_SPEED_R = rval;
  update("Left: " + String(lval) + "Right: "+ String(rval));
}
void loadVolume(uint32_t val){
  player.volume(val);
  prefer.begin("state",false);
  prefer.putUInt("volume", val);
  prefer.end();
  update("volume:" + String(val));
}
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Topic: ");
  Serial.println(topic);

  // -------- Payload to String ----------
  String jsonStr;
  for (unsigned int i = 0; i < length; i++) {
    jsonStr += (char)payload[i];
  }
  Serial.print("[MQTT] Payload: ");
  Serial.println(jsonStr);

  // -------- JSON Parse ----------
  DynamicJsonDocument doc(4096);// increase if cards list is big
  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error) {
    Serial.print("[MQTT] JSON Error: ");
    Serial.println(error.c_str());
    return;
  }

  // -------- CMD Read ----------
  const char* cmd = doc["cmd"];
  if (!cmd) {
    Serial.println("[MQTT] cmd missing");
    return;
  }

  String command = String(cmd);
  command.toLowerCase();

  // -------- CMD ROUTER ----------
  if (command == "cards") {
    handleCards(doc);
  }else if(command == "speed"){
    loadSpeed(doc["l_speed"] | PWM_SPEED_L,doc["r_speed"] | PWM_SPEED_R);
  }else if(command == "volume"){
    loadVolume(doc["volume"] | 25);
  }else if (command == "card") {
    handleSingleCard(doc);
  }else if (command == "open") {
    handleOpen();
  }else if (command == "close"  || command == "stop") {
    handleClose();
  }else if(command == "alt"){
    skiploop = doc["state"] | false; // {"cmd":"alt","state":true} || {"cmd":"alt","state":true}
    if (skiploop) {
      Serial.println("True");
    } else {
      Serial.println("False");
    }
    update("skiploop is now: " + String(skiploop));
  }else if (command == "run") {
    handleRun();
  }else if (command == "reset") {
    handleReset();
  }else if (command == "get") {
    handleGet();
  }else if(command == "restart"){
    update("Restarting....");
    ESP.restart();
  }else if(command == "format"){
    if (LittleFS.format()){
      Serial.println("LittleFS formated.");
      update("LittleFS formated.");
    }else{
      update("LittleFS Can't format ESP Problems.");
      Serial.println("LittleFS Can't format ESP Problems.");
    }
  }else {
    Serial.print("[MQTT] Unknown cmd: ");
    Serial.println(command);
  }
}

void connectMQTT() {
  unsigned long now = millis();
  if(now - mqtttryTime >= mqtttryConnectDelay){
    mqtttryTime = now;
    if (!client.connected()) {

      Serial.print("Connecting to MQTT... ");
      if (client.connect(Chipid.c_str(), mqttUser, mqttPassword,
                        statusTopic.c_str(), 1, true, "offline")) {
        Serial.println("connected");

        // Publish online status (retained)
        client.publish(statusTopic.c_str(), "online", true);
        Serial.print("Status sent to topic: ");
        Serial.println(statusTopic);

        // Subscribe to command topic
        String cmdTopic = "devices/" + Chipid + "/cmd";
        client.subscribe(cmdTopic.c_str());
        if(FS_Faulty_update_requied){
          FS_Faulty_update_requied = false;
          client.publish(infoTopic.c_str(), "LittleFS Faulty", true); // retained
          Serial.println("[MQTT] LittleFS fault reported to topic: " + infoTopic);
        }else{
          client.publish(infoTopic.c_str(), "LittleFS Ok", true); // retained
          Serial.println("[MQTT] LittleFS Ok reported to topic: " + infoTopic);
        }
        Serial.print("Subscribed to topic: ");
        Serial.println(cmdTopic);
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" retrying in 30 seconds");
      }
    }
  }
    
}

// --------- HTTPS helpers (like glass door) ---------
static const char* kHost = "iot.neways3.com";

bool dns_ok() {
  IPAddress ip;
  int r = WiFi.hostByName(kHost, ip);
  if (r != 1) {
    Serial.println("[NET] DNS lookup failed; skipping HTTP this cycle.");
    return false;
  }
  return true;
}

int post_json_https(const char* url, const String& postData, String* out) {
  WiFiClientSecure client;
  client.setInsecure(); // ⚠️ For testing only, use proper cert in production
  client.setTimeout(3000);

  HTTPClient http;
  http.setConnectTimeout(3000);
  http.setTimeout(3000);
  http.setReuse(false);

  String currentUrl = String(url);
  int code = -1;

  // Follow up to 3 redirects
  for (int redirectCount = 0; redirectCount < 3; redirectCount++) {
    if (!http.begin(client, currentUrl)) {
      Serial.println("[HTTP] begin() failed");
      return -1;
    }
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    //http.addHeader("Content-Type", "application/json");
    //http.addHeader("Authorization", "101a1a587f0624b6b30cf78f579f54b5b30862267b99e760a");

    code = http.POST(postData);

    if (code > 0) {
      // Check for redirect codes
      if (code == HTTP_CODE_FOUND || code == HTTP_CODE_MOVED_PERMANENTLY ||
          code == HTTP_CODE_SEE_OTHER || code == HTTP_CODE_TEMPORARY_REDIRECT ||
          code == HTTP_CODE_PERMANENT_REDIRECT) {
        
        String newLocation = http.getLocation();
        Serial.printf("[HTTP] Redirect (%d) to: %s\n", code, newLocation.c_str());
        http.end();
        currentUrl = newLocation;
        continue; // retry with new URL
      }

      // Success
      if (out) {
        if (code == HTTP_CODE_OK) {
          *out = http.getString();
        } else {
          out->remove(0);
        }
      }
      http.end();
      client.stop();
      return code;
    } else {
      Serial.printf("[HTTP] POST failed: %s\n", http.errorToString(code).c_str());
      http.end();
      client.stop();
      return code;
    }
  }

  client.stop();
  return code; // return last code if too many redirects
}


// --------- Safe WDT kick (only for watched tasks) ---------
inline void wdt_kick_safe() {
  TaskHandle_t me = xTaskGetCurrentTaskHandle();
  if (me == core0Handle || me == core1Handle) {
    esp_task_wdt_reset();
  }
}

// --------- Backoff scheduling ----------
void scheduleAfterSuccess() {
  inBackoff = false;
  backoffMs = BACKOFF_MIN;
  // Online again → allow one-time offline restart later
  rtc_offlineRestarted = false;
  nextSyncDueMs = millis() + (load_succes ? delayTime : delayData);
}

void scheduleAfterFailure() {
  inBackoff = true;
  nextSyncDueMs = millis() + backoffMs;
  if (backoffMs < BACKOFF_MAX) backoffMs = min(backoffMs * 2, BACKOFF_MAX);
  Serial.printf("[HTTP] offline/backoff for %lus\n", (nextSyncDueMs - millis())/1000);
}
void initPrefer(){
  prefer.begin("state",false);
  PWM_SPEED_L = prefer.getUInt("l_speed",PWM_SPEED_L);// 
  PWM_SPEED_R = prefer.getUInt("r_speed",PWM_SPEED_R);//
  player.volume(prefer.getUInt("volume",25)); 

  prefer.end();
}
// ================== SETUP ==================
void setup() {
  pinMode(PWM1_L,OUTPUT);
  pinMode(PWM2_L,OUTPUT);
  pinMode(PWM3_R,OUTPUT);
  pinMode(PWM4_R,OUTPUT);

  digitalWrite(PWM1_L,LOW);
  digitalWrite(PWM2_L,LOW);
  digitalWrite(PWM3_R,LOW);
  digitalWrite(PWM4_R,LOW);
  
  delay(500);
  pinMode(P_R_CLOSE_S,INPUT);
  pinMode(P_R_OPEN_S,INPUT);

  pinMode(P_IR_INSIDE,INPUT_PULLDOWN);
  pinMode(P_IR_OUTSIDE,INPUT_PULLDOWN);
  pinMode(P_L_DOOR_CLOSE_S,INPUT_PULLDOWN);
  pinMode(P_R_DOOR_CLOSE_S,INPUT_PULLDOWN);
  pinMode(P_L_DOOR_OPEN_S,INPUT_PULLDOWN);
  pinMode(P_R_DOOR_OPEN_S,INPUT_PULLDOWN);
  pinMode(AC_POWER,INPUT_PULLDOWN);


  pinMode(INTER_SIGNAL,OUTPUT);
  pinMode(EXIT_SIGNAL,OUTPUT);
  digitalWrite(INTER_SIGNAL,LOW);
  digitalWrite(EXIT_SIGNAL,LOW);
  Serial.begin(9600);
  if (player.begin(Serial)) { // Use Serial (UART0) for DFPlayer Mini
    Serial.println("DFPlayer Mini connected!");
    player.volume(30); // Set volume (Range: 0 - 30)
    // player.play(6);    // Play the first track
    delay(500);
  } else {
    Serial.println("Failed to connect to DFPlayer Mini.");
  }
  player.play(GET_READY_FOR_USE); 
  initPrefer();
  delay(100);

  Serial.println("\n[SETUP] Starting device...");
  Serial.print(F("ChipId: ")); Serial.println(Chipid);
  Serial.print(F("MAC: "));    Serial.println(WiFi.macAddress());

  pinMode(ST_LED, OUTPUT);
  digitalWrite(ST_LED, LOW);
  delay(10);
  RDM_INSIDE.begin(P_RDM_INSIDE,1);
  RDM_OUTSIDE.begin(P_RDM_OUTSIDE,2);
  delay(500);

  // LittleFS
  // LittleFS
  Serial.print("File System Mounting...");
  
  // Pass 'true' to formatOnFail
  if (!LittleFS.begin(true)) {
    Serial.println("\nLittleFS Mount Failed. Formatting also failed.");
    Serial.println("Check Partition Scheme in Arduino IDE.");
    FS_Faulty_update_requied = true;
  } else {
    Serial.println(" Success!");
    FS_Faulty_update_requied = false;
    
    // Check if we can actually write a test file
    File testFile = LittleFS.open("/boot_check.txt", "w");
    if(testFile){
      testFile.print("OK");
      testFile.close();
    } else {
       Serial.println("LittleFS mounted but write failed!");
    }
  }
  delay(10);
  Serial.println("\nPlace RFID tag near the rdm6300...");

  // Setup MQTT server and callback
  client.setBufferSize(10240);   // 4 KB buffer
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);
  mqttQueue = xQueueCreate(20, sizeof(mqtt_msg_t));
  if (!mqttQueue) {
    Serial.println("MQTT Queue create failed!");
    while (1);
  }

  // Watchdog (clean re-init; only core0/core1 are watched)
  esp_task_wdt_deinit();
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms     = 60000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&twdt_config);

  // WiFiManager non-blocking portal
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  wm.setDebugOutput(true);
  wm.setConnectTimeout(5);
  wm.setConfigPortalBlocking(false);

  unsigned long startTime = millis();
  if (wm.autoConnect(ssid.c_str())) {
    Serial.println(F("Wifi autoConnect!"));
  } else {
    Serial.println(F("Configportal running"));
  }

  // Give up to 10s for first connection
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(200);
    updateWifiLed();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected at boot.");
    led_ST = true;
    digitalWrite(ST_LED, HIGH);
    // Online -> allow single offline restart later
    rtc_offlineRestarted = false;
    nextSyncDueMs = millis() + 2000; // quick first sync
  } else {
    Serial.println("\n[WIFI] Failed to connect at boot.");
    led_ST = false;
    digitalWrite(ST_LED, LOW);
    if (!wm.getConfigPortalActive()) {
      Serial.println("[WM] Starting config portal (SDL-...)");
      wm.startConfigPortal(ssid.c_str());
    }
    nextSyncDueMs = millis() + 5000; // try soon even if offline
  }

  // Create tasks pinned to cores
  BaseType_t t0 = xTaskCreatePinnedToCore(core0Task, "Core0Task", 8192,  NULL, 1, &core0Handle, 0);
  BaseType_t t1 = xTaskCreatePinnedToCore(core1Task, "Core1Task", 10000, NULL, 1, &core1Handle, 1);

  if (t0 == pdPASS && t1 == pdPASS) {
    esp_task_wdt_add(core0Handle);
    esp_task_wdt_add(core1Handle);
    Serial.println("[TASK] Both tasks started.");
  } else {
    Serial.println("[TASK] Task creation failed! Restarting...");
    delay(500);
    ESP.restart();
  }
  player.play(MACHINE_READY_FOR_USE);
}

void loop() {
  // Serial.printf("[Arduino loop ] Running on core: %d\n", xPortGetCoreID());
  // vTaskDelay(10 / portTICK_PERIOD_MS);
}

// ================== CORE 0: orchestrator (WiFi/HTTP scheduling) ==================
void core0Task(void *parameter) {
  mqtt_msg_t msg;
  Serial.printf("[CORE 0] Running on core: %d\n", xPortGetCoreID());
  for (;;) {

    wdt_kick_safe();
    wm.process();
    reconnectWiFi();
    updateWifiLed();
    if (!client.connected() && led_ST) {
      connectMQTT();
    }else{
      // Limit publish per cycle
      int sent = 0;
      while (xQueueReceive(mqttQueue, &msg, 0) == pdTRUE && sent < 5) {
        Serial.print("[MQTT] Topic   : ");
        Serial.println(msg.topic);

        Serial.print("[MQTT] Payload : ");
        Serial.println(msg.payload);

        Serial.print("[MQTT] Retain  : ");
        Serial.println(msg.retained);

        client.publish(msg.topic, msg.payload, msg.retained);
        Serial.println("[MQTT] Event published\n");
        sent++;
      }
    }
    client.loop();

    unsigned long now = millis();

    // Early sync (first-load retries) -> nudge when not in backoff
    if (led_ST && !load_succes && (now - lastEarlySync >= delayData)) {
      lastEarlySync = now;
      if (!inBackoff) nextSyncDueMs = millis();
    }

    // Periodic sync every 10 minutes -> respect backoff
    if (led_ST && (now - lastPeriodicSync >= delayTime)) {
      lastPeriodicSync = now;
      if (!inBackoff) nextSyncDueMs = millis();
    }

    // Immediate sync requested (e.g., from cardCK while online)
    if (led_ST && request_sync_now) {
      request_sync_now = false;
      if (!inBackoff) nextSyncDueMs = millis();
    }

    // Start NetWorker only when due and not already running
    if (led_ST && !netTaskBusy && netTaskHandle == NULL && millis() >= nextSyncDueMs) {
      netTaskBusy = true;
      xTaskCreatePinnedToCore(NetWorker, "NetWorker", 8192, NULL, 1, &netTaskHandle, 0);
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ================== CORE 1: RFID/Motor ==================
void GET_INPUT(){
  INSIDE_IR_STATE = digitalRead(P_IR_INSIDE);
  OUTSIDE_IR_STATE = digitalRead(P_IR_OUTSIDE);
  L_M_CLOSE = digitalRead(P_L_DOOR_CLOSE_S);
  R_M_CLOSE = digitalRead(P_R_DOOR_CLOSE_S);
  L_M_OPEN = digitalRead(P_L_DOOR_OPEN_S);
  R_M_OPEN = digitalRead(P_R_DOOR_OPEN_S);
  if (RDM_INSIDE.get_new_tag_id() && !INSIDE_CARD){
    INSIDE_CARD = true;
  }else if(INSIDE_CARD){
    INSIDE_CARD = false;
  }
  if(RDM_OUTSIDE.get_new_tag_id() && !OUTSIDE_CARD){
    OUTSIDE_CARD = true;
  }else if(OUTSIDE_CARD){
    OUTSIDE_CARD = false;
  }
  if(digitalRead(P_R_OPEN_S) == HIGH){
    if(!openState){
      // open
      player.play(EMERGENCY_OPENING); 
      openAllTime();
    }
  }
  if(digitalRead(P_R_CLOSE_S) == HIGH){
    if(!closeState){
      // Close audio EMERGENCY_CLOSING
      clossing();
    }
  }
//.........................................................
  int CurrentState = digitalRead(AC_POWER);
  if(AC_POWER_STATE != CurrentState){
    AC_POWER_STATE = CurrentState;
    if(AC_POWER_STATE == 0){
      if(!openState){
        // open
        player.play(POWER_DISCONNECTED);
        vTaskDelay(1500);
        player.play(EMERGENCY_OPENING); 
        openAllTime();
      }
    }else if(AC_POWER_STATE == 1){
      if(!closeState){
        // Close
        player.play(POWER_CONNECTED); // power connected
        vTaskDelay(1500);
        player.play(EMERGENCY_CLOSING);
        clossing();
      }
    }
  }
  if(MQTT_OPEN_REQ){
    MQTT_OPEN_REQ = false;
    player.play(EMERGENCY_OPENING); 
    openAllTime();
  }
  if(MQTT_CLOSE_REQ){
    MQTT_CLOSE_REQ = false;
    player.play(EMERGENCY_CLOSING);
    clossing();
  }
  
}
bool isGranted(String number){
  File file = LittleFS.open(File_Name, "r");
  if (file) {
    while (file.available()) {
      String cardNumber = file.readStringUntil('\n');
      cardNumber.trim();
      if(cardNumber == number){
        file.close();
        return true;
      }
    }
    Serial.println("Do not match Card Mamber!");
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }
  return false;
}
bool masterCK(String number){
  for(int i = 0;i <12;i++){
    if(m_cardArray[i] == number){
      Serial.println("Your using Master Card !....");
      return true;
    }
  }
  Serial.println("If Use New Card Try Again....! Master or Normal Card don't match");
  return false;
}
void open_for_out(){
  Serial.println("From Opening Funcation");
  L_DOOR_OPEN_DONE = false;
  R_DOOR_OPEN_DONE = false;
  IN_IR_HIGH = false;
  OUT_IR_HIGH = false;
  digitalWrite(INTER_SIGNAL,LOW);
  digitalWrite(EXIT_SIGNAL,HIGH);
  unsigned long openTime = millis();
  unsigned long LM_START_TIME = 0;
  unsigned long RM_START_TIME = 0;
  while(true){
    wdt_kick_safe();

    unsigned long currentMillis = millis();
    L_M_OPEN = digitalRead(P_L_DOOR_OPEN_S);
    R_M_OPEN = digitalRead(P_R_DOOR_OPEN_S);

    INSIDE_IR_STATE = digitalRead(P_IR_INSIDE);
    OUTSIDE_IR_STATE = digitalRead(P_IR_OUTSIDE);
    // SET TIMEOUT
    if(currentMillis - openTime >= doorTimeout){
      clossing();
      return;
    }
    if(INSIDE_IR_STATE){
      IN_IR_HIGH = true;
      Serial.println("Pass frist step.");
    }
    if(OUTSIDE_IR_STATE){
      OUT_IR_HIGH = true;
      Serial.println("Pass last step.");
    }
    if(L_M_OPEN && !L_DOOR_OPEN_DONE){
      L_DOOR_OPEN_DONE = true;
      Serial.println("L_DOOR_OPEN_DONE");
    }
    if(R_M_OPEN && !R_DOOR_OPEN_DONE){
      R_DOOR_OPEN_DONE = true;
      Serial.println("R_DOOR_OPEN_DONE");
    }
    if(R_DOOR_OPEN_DONE && L_DOOR_OPEN_DONE && closeState){
      closeState = false;
      openState = true;
    }
    if(!IN_IR_HIGH && OUT_IR_HIGH){
      Serial.println("Wrang Way");
      player.play(WRONG_WAY);
      if(L_DOOR_OPEN_DONE && R_DOOR_OPEN_DONE){
        clossing();
        return;
      }
    }
    // System Logic For Left Motor
    if(!L_DOOR_OPEN_DONE){
      if(!L_OPEN_GIVEN){
        L_OPEN();
        LM_START_TIME = currentMillis;
      }else if (currentMillis - LM_START_TIME >= M_Fault_Time_OUT) {
        update("Left Motor Fault");
        Serial.println("Left Motor Fault");
        L_STOP();
      }
    }else if(L_OPEN_GIVEN){
      L_STOP();
    }
    // System Logic For Right Motor
    if(!R_DOOR_OPEN_DONE){
      if(!R_OPEN_GIVEN){
        R_OPEN();
        RM_START_TIME = currentMillis;
      }else if (currentMillis - RM_START_TIME >= M_Fault_Time_OUT) {
        update("Right Motor Fault");
        Serial.println("Right Motor Fault");
        R_STOP();
      }
    }else if(R_OPEN_GIVEN){
      R_STOP();
    }
    // When All Door Open
    if(L_DOOR_OPEN_DONE && R_DOOR_OPEN_DONE){
      // Need to create time for automaticly clossing
      if(IN_IR_HIGH && OUT_IR_HIGH){
        Serial.println("Thank You For Using");
        // player.play(5);
        clossing();
        return;
      }
    }
  }
}
void open_for_in(){
  Serial.println("From Opening Funcation");
  L_DOOR_OPEN_DONE = false;
  R_DOOR_OPEN_DONE = false;
  IN_IR_HIGH = false;
  OUT_IR_HIGH = false;
  digitalWrite(INTER_SIGNAL,HIGH);
  digitalWrite(EXIT_SIGNAL,LOW);
  unsigned long openTime = millis();
  unsigned long LM_START_TIME = 0;
  unsigned long RM_START_TIME = 0;
  while(true){
    wdt_kick_safe();
    unsigned long currentMillis = millis();
    
    L_M_OPEN = digitalRead(P_L_DOOR_OPEN_S);
    R_M_OPEN = digitalRead(P_R_DOOR_OPEN_S);

    INSIDE_IR_STATE = digitalRead(P_IR_INSIDE);
    OUTSIDE_IR_STATE = digitalRead(P_IR_OUTSIDE);
    // Set Timeout
    if(currentMillis - openTime >= doorTimeout){
      clossing();
      return;
    }
    if(INSIDE_IR_STATE){
      IN_IR_HIGH = true;
      Serial.println("Pass last step.");
    }
    if(OUTSIDE_IR_STATE){
      OUT_IR_HIGH = true;
      Serial.println("Pass frist step.");
    }
    if(L_M_OPEN && !L_DOOR_OPEN_DONE){
      L_DOOR_OPEN_DONE = true;
      Serial.println("L_DOOR_OPEN_DONE");
    }
    if(R_M_OPEN && !R_DOOR_OPEN_DONE){
      R_DOOR_OPEN_DONE = true;
      Serial.println("R_DOOR_OPEN_DONE");
    }
    if(R_DOOR_OPEN_DONE && L_DOOR_OPEN_DONE && closeState){
      closeState = false;
      openState = true;
    }
    if(!OUT_IR_HIGH && IN_IR_HIGH){
      Serial.println("Wrang Way Detect..");
      player.play(WRONG_WAY);
      if(L_DOOR_OPEN_DONE && R_DOOR_OPEN_DONE){
        clossing();
        return;
      }
    }
    // System Logic For Left Motor
    if(!L_DOOR_OPEN_DONE){
      if(!L_OPEN_GIVEN){
        L_OPEN();
        LM_START_TIME = currentMillis;
      }else if (currentMillis - LM_START_TIME >= M_Fault_Time_OUT) {
        update("Left Motor Fault");
        Serial.println("Left Motor Fault");
        L_STOP();
      }
    }else if(L_OPEN_GIVEN){
      L_STOP();
    }
    // System Logic For Right Motor
    if(!R_DOOR_OPEN_DONE){
      if(!R_OPEN_GIVEN){
        R_OPEN();
        RM_START_TIME = currentMillis;
      }else if (currentMillis - RM_START_TIME >= M_Fault_Time_OUT) {
        update("Right Motor Fault");
        Serial.println("Right Motor Fault");
        R_STOP();
      }
    }else if(R_OPEN_GIVEN){
      R_STOP();
    }
    // When All Door Open
    if(L_DOOR_OPEN_DONE && R_DOOR_OPEN_DONE){
      // Need to create time for automaticly clossing
      if(IN_IR_HIGH && OUT_IR_HIGH){
        Serial.println("Thank You For Using");
        // player.play(17);
        clossing();
        return;
      }
    }
  }
}
void clossing(){
  Serial.println("From Clossing Funcation");
  L_DOOR_CLOSE_DONE = false;
  R_DOOR_CLOSE_DONE = false;
  digitalWrite(INTER_SIGNAL,LOW);
  digitalWrite(EXIT_SIGNAL,LOW);

  unsigned long LM_START_TIME = 0;
  unsigned long RM_START_TIME = 0;

  while(true){
    wdt_kick_safe();
    unsigned long currentMillis = millis();
    L_M_CLOSE = digitalRead(P_L_DOOR_CLOSE_S);
    R_M_CLOSE = digitalRead(P_R_DOOR_CLOSE_S);
    
    if(L_M_CLOSE && !L_DOOR_CLOSE_DONE){
      L_DOOR_CLOSE_DONE = true;
    }
    if(R_M_CLOSE && !R_DOOR_CLOSE_DONE){
      R_DOOR_CLOSE_DONE = true;
    }
    if(L_DOOR_CLOSE_DONE && R_DOOR_CLOSE_DONE){
      Serial.println("Door Close Done.");
      L_STOP();
      R_STOP();
      closeState = true;
      openState = false;
      IN_IR_HIGH = false;
      OUT_IR_HIGH = false;
      return;
    }
    if(!L_DOOR_CLOSE_DONE){
      if(!L_CLOSE_GIVEN){
        L_CLOSE();
        LM_START_TIME = currentMillis;
      }else if (currentMillis - LM_START_TIME >= M_Fault_Time_OUT) {
        update("Left Motor Fault");
        Serial.println("Left Motor Fault");
        L_STOP();
      }
    }else if(L_CLOSE_GIVEN){
      L_STOP();
    }

    if(!R_DOOR_CLOSE_DONE){
      if(!R_CLOSE_GIVEN){
        R_CLOSE();
        RM_START_TIME = currentMillis;
      }else if (currentMillis - RM_START_TIME >= M_Fault_Time_OUT) {
        update("Right Motor Fault");
        Serial.println("Right Motor Fault");
        R_STOP();
      }
    }else if(R_CLOSE_GIVEN){
      R_STOP();
    }
  }
}
void L_OPEN(){
  analogWrite(PWM2_L, 0);
  analogWrite(PWM1_L, PWM_SPEED_L);
  Serial.println("L_Motor opening...");
  L_OPEN_GIVEN = true;
}
void R_OPEN(){
  analogWrite(PWM4_R, 0);
  analogWrite(PWM3_R, PWM_SPEED_L);
  Serial.println("R_Motor opening...");
  R_OPEN_GIVEN = true;
}
void L_CLOSE(){
  analogWrite(PWM1_L,0);
  analogWrite(PWM2_L,PWM_SPEED_L);
  Serial.println("L_Motor clossing...");
  L_CLOSE_GIVEN = true;
}
void R_CLOSE(){
  analogWrite(PWM3_R, 0);
  analogWrite(PWM4_R, PWM_SPEED_L);
  Serial.println("R_Motor clossing...");
  R_CLOSE_GIVEN = true;
}
void L_STOP(){
  analogWrite(PWM1_L,0);
  analogWrite(PWM2_L,0);
  Serial.println("L_Motor Stop");
  L_CLOSE_GIVEN = false;
  L_OPEN_GIVEN = false;
}
void R_STOP(){
  analogWrite(PWM3_R,0);
  analogWrite(PWM4_R,0);
  Serial.println("R_Motor Stop");
  R_CLOSE_GIVEN = false;
  R_OPEN_GIVEN = false;
}

void openAllTime(){
  Serial.println("From Opening Funcation");
  L_DOOR_OPEN_DONE = false;
  R_DOOR_OPEN_DONE = false;
  IN_IR_HIGH = false;
  OUT_IR_HIGH = false;
  digitalWrite(INTER_SIGNAL,HIGH);
  digitalWrite(EXIT_SIGNAL,HIGH);

  unsigned long LM_START_TIME = 0;
  unsigned long RM_START_TIME = 0;
  while(true){
    wdt_kick_safe();
    unsigned long currentMillis = millis();

    L_M_OPEN = digitalRead(P_L_DOOR_OPEN_S);
    R_M_OPEN = digitalRead(P_R_DOOR_OPEN_S);


    if(L_M_OPEN && !L_DOOR_OPEN_DONE){
      L_DOOR_OPEN_DONE = true;
      Serial.println("L_DOOR_OPEN_DONE");
    }
    if(R_M_OPEN && !R_DOOR_OPEN_DONE){
      R_DOOR_OPEN_DONE = true;
      Serial.println("R_DOOR_OPEN_DONE");
    }
    if(R_DOOR_OPEN_DONE && L_DOOR_OPEN_DONE && closeState){
      closeState = false;
      openState = true;
    }
    // System Logic For Left Motor
    if(!L_DOOR_OPEN_DONE){
      if(!L_OPEN_GIVEN){
        L_OPEN();
        LM_START_TIME = currentMillis;
      }else if (currentMillis - LM_START_TIME >= M_Fault_Time_OUT) {
        update("Left Motor Fault");
        Serial.println("Left Motor Fault");
        L_STOP();
      }
    }else if(L_OPEN_GIVEN){
      L_STOP();
    }
    // System Logic For Right Motor
    if(!R_DOOR_OPEN_DONE){
      if(!R_OPEN_GIVEN){
        R_OPEN();
        RM_START_TIME = currentMillis;
      }else if (currentMillis - RM_START_TIME >= M_Fault_Time_OUT) {
        update("Right Motor Fault");
        Serial.println("Right Motor Fault");
        R_STOP();
      }
    }else if(R_OPEN_GIVEN){
      R_STOP();
    }
    // When All Door Open
    if(L_DOOR_OPEN_DONE && R_DOOR_OPEN_DONE){
      // Need to create time for automaticly clossing
      Serial.println("Open For All way in out");
      closeState = false;
      openState = true;
      return;
    }
  }
}
void core1Task(void *parameter) {
  Serial.printf("[CORE 1] Running on core: %d\n", xPortGetCoreID());
  for (;;) {
    wdt_kick_safe();
    if(skiploop){
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    GET_INPUT();
    unsigned long currentMillis = millis();
    
    if (INSIDE_IR_STATE && OUTSIDE_IR_STATE) {
      Serial.println("Step Back");
      player.play(STEP_BACK);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }else if(INSIDE_CARD && OUTSIDE_CARD){
      Serial.println("Need to punch one by one..");
    }else if(INSIDE_CARD){
      String cardString = String(RDM_INSIDE.get_tag_id(), DEC);  // Convert the card number to a string
      while (cardString.length() < 10) {                           // Make the card number 10 digits long by adding trailing zeros
        cardString = "0" + cardString;                             // cardString += "0";
      }
      Serial.println("From InSide : "+cardString);
      if(isGranted(cardString) || masterCK(cardString)){
        Serial.println("Opening For Exit.");
        player.play(SEE_YOU_AGAIN); 
        open_for_out();
      }else{
        player.play(ACCESS_DENIED); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (!inBackoff) nextSyncDueMs = millis();
        Serial.println("[HTTP] Scheduled immediate sync; present card again after sync.");
      }
    }else if(OUTSIDE_CARD){
      String cardString = String(RDM_OUTSIDE.get_tag_id(), DEC);
      while (cardString.length() < 10) {                           // Make the card number 10 digits long by adding trailing zeros
        cardString = "0" + cardString;                             // cardString += "0";
      }
      Serial.println("From OutSide : "+cardString);
      if(isGranted(cardString) || masterCK(cardString)){
        Serial.println("Opening For Inter.");
        player.play(WELCOME_TO_SUPER_HOME); 
        open_for_in();
      }else{
        player.play(ACCESS_DENIED);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (!inBackoff) nextSyncDueMs = millis();
        Serial.println("[HTTP] Scheduled immediate sync; present card again after sync.");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ================== NetWorker (HTTPS + backoff; not WDT-watched) ==================
bool syncCards(const String& response, const char* File_Name) {
  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, response);
  if (err) {
    Serial.printf("[HTTP] JSON parse error: %s\n", err.c_str());
    return false;
  }

  JsonArray serverCards = doc["card_numbers"].as<JsonArray>();

  // Load existing cards from file
  std::vector<String> storedCards;
  if (LittleFS.exists(File_Name)) {
    File file = LittleFS.open(File_Name, "r");
    if (file) {
      while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) storedCards.push_back(line);
      }
      file.close();
    } else {
      file.close();
      Serial.println("[FS] Failed to open file for read, formatting...");
      if (!LittleFS.format()) {
        ESP.restart();
      }
    }
  } else {
    Serial.println("[FS] File not found, formatting...");
    if (!LittleFS.format()) {
      vTaskDelay(100);
      ESP.restart();
    }else{
      File newFile = LittleFS.open("/p.txt", "w");
      if (newFile) {
        newFile.println("first line of data");
        newFile.close();
        Serial.println("File created and written successfully!");
      } else {
        Serial.println("Failed to create/open file.");
        ESP.restart();
      }
    }
  }

  // Convert server cards to vector
  std::vector<String> serverCardsVec;
  for (JsonVariant v : serverCards) {
    String s = v.as<String>(); s.trim();
    if (s.length() > 0) serverCardsVec.push_back(s);
  }

  // Compare sets manually
  bool changed = false;

  // Add new cards
  for (auto& s : serverCardsVec) {
    bool found = false;
    for (auto& c : storedCards) {
      if (c == s) { found = true; break; }
    }
    if (!found) {
      Serial.printf("[SYNC] Adding new card: %s\n", s.c_str());
      storedCards.push_back(s);
      changed = true;
    }
  }

  // Remove cards not in server
  for (auto it = storedCards.begin(); it != storedCards.end();) {
    bool found = false;
    for (auto& s : serverCardsVec) {
      if (*it == s) { found = true; break; }
    }
    if (!found) {
      Serial.printf("[SYNC] Removing card: %s\n", it->c_str());
      it = storedCards.erase(it);
      changed = true;
    } else {
      ++it;
    }
  }

  // Write back if changed OR file was missing
  if (changed || !LittleFS.exists(File_Name)) {
    File file = LittleFS.open(File_Name, "w");
    if (file) {
      for (auto& s : storedCards) {
        file.println(s);
      }
      file.close();

      // ✅ Verify by re-reading
      File verify = LittleFS.open(File_Name, "r");
      if (verify) {
        Serial.println("[SYNC] Verifying stored cards:");
        while (verify.available()) {
          String line = verify.readStringUntil('\n');
          line.trim();
          if (line.length() > 0) {
            Serial.println("  " + line);
          }
        }
        verify.close();
      }

      // ✅ Notify system
      update("Card list updated");
      load_succes = true;
      Serial.printf("[SYNC] Stored %u card(s) to %s\n", (unsigned)storedCards.size(), File_Name);
      return true;
    } else {
      Serial.println("[FS] Failed to open file for write.");
      if (LittleFS.format()) {
        return syncCards(response, File_Name); // retry once
      } else {
        ESP.restart();
      }
    }
  } else {
    Serial.println("[SYNC] No changes, file unchanged.");
    return true; // still OK, nothing to update
  }

  return false; // fallback
}
void NetWorker(void *parameter) {
  Serial.printf("[Network Task runing core ] Running on core: %d\n", xPortGetCoreID());
   String postData = "api_secret=" + String(api_secret) +
                    "&device_code=" + String(Chipid);
  bool ok = false;

  if (WiFi.status() == WL_CONNECTED && dns_ok()) {
    Serial.println("[HTTP] NetWorker: deviceLog + fetch cards");

    // deviceLog
    int code = post_json_https(
                  "put your device status update URL here",
                  postData,
                  nullptr
                );

    Serial.printf("[HTTP] device status code: %d\n", code);

    ok |= (code == 200);
    vTaskDelay(1);


    // ----------- Fetch Cards -----------
    String response;
    code = post_json_https(
             "put your fetch cards URL here",
             postData,
             &response
           );

    Serial.printf("[HTTP] fetch cards code: %d\n", code);

    if (code == 200) {
      ok = syncCards(response, File_Name);
      load_succes = ok;
    } else {
      load_succes = false;
    }

    vTaskDelay(1);
  }else {// ---------------- ELSE BLOCK (correct) -----------------
    Serial.println("[HTTP] NetWorker: skip (offline/DNS)");
  }

  if (ok){
    player.play(DATA_UPDATE); 
    scheduleAfterSuccess();
  } else{


   // ----------- Device Status Update -----------
    Serial.println("[HTTP] NetWorker: deviceLog + fetch cards");
    int code = post_json_https(
                  "https://cloud.neways3.com/api/access-device/information-access/update-device-status",
                  postData,
                  nullptr
                );

    Serial.printf("[HTTP] device status code: %d\n", code);

    ok |= (code == 200);
    vTaskDelay(1);


    // ----------- Fetch Cards -----------
    String response;
    code = post_json_https(
             "https://cloud.neways3.com/api/access-device/information-access/get-permission",
             postData,
             &response
           );

    Serial.printf("[HTTP] fetch cards code: %d\n", code);

    if (code == 200) {
      ok = syncCards(response, File_Name);
      load_succes = ok;
    } else {
      load_succes = false;
    }
    vTaskDelay(1);
    
    if (ok){
    player.play(DATA_UPDATE); 
    scheduleAfterSuccess();
    }else{
    player.play(DATA_UPDATE_FAILED); 
    scheduleAfterFailure();
    }
  }
  netTaskBusy = false;
  netTaskHandle = NULL;
  vTaskDelete(NULL);
}

// ================== Motor control (UNCHANGED) ==================


// ================== WiFi reconnect + LED ==================
void reconnectWiFi() {
  wl_status_t st = WiFi.status();

  if (st == WL_CONNECTED) {
    if (wm.getConfigPortalActive()) wm.stopConfigPortal();
    updateWifiLed();
    // Online -> allow next offline period to trigger one-time restart again
    rtc_offlineRestarted = false;

    batchActive   = false;
    batchAttempts = 0;
    failedBatches = 0;
    batchStartMs  = 0;
    lastAttemptMs = 0;
    return;
  }

  updateWifiLed();

  if (!wm.getConfigPortalActive()) {
    Serial.println("[WM] Disconnected -> starting config portal (SDL-...).");
    wm.startConfigPortal(ssid.c_str());
  }
  wm.process();

  // Pause STA while a user is in the portal so erase/save works cleanly
  int clients = WiFi.softAPgetStationNum();
  if (clients > 0) {
    esp_wifi_disconnect();
    WiFi.disconnect(false, false);
    batchActive   = false;
    batchAttempts = 0;
    lastAttemptMs = 0;
    return;
  }

  unsigned long now = millis();

  // Start a new batch when cooldown passes
  if (!batchActive) {
    if (batchStartMs == 0 || (now - batchStartMs >= BATCH_COOLDOWN_MS)) {
      batchActive   = true;
      batchAttempts = 0;
      batchStartMs  = now;
      Serial.printf("[WIFI] Starting reconnect batch (failed batches so far: %u)\n", failedBatches);
    } else {
      return;
    }
  }

  if (batchAttempts < MAX_ATTEMPTS_PER_BATCH) {
    if (now - lastAttemptMs < ATTEMPT_SPACING_MS) return;

    lastAttemptMs = now;
    batchAttempts++;
    Serial.printf("[WIFI] Batch attempt %u/%u...\n", batchAttempts, MAX_ATTEMPTS_PER_BATCH);

    esp_wifi_disconnect();
    WiFi.begin();
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < CONNECT_TRY_MS) {
      wm.process();
      wdt_kick_safe();
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
      updateWifiLed();
      if (wm.getConfigPortalActive()) wm.stopConfigPortal();

      // Reconnected: reset backoff & schedule quick sync
      inBackoff = false;
      backoffMs = BACKOFF_MIN;
      rtc_offlineRestarted = false;
      nextSyncDueMs = millis() + 2000;

      batchActive   = false;
      batchAttempts = 0;
      failedBatches = 0;
      batchStartMs  = 0;
      lastAttemptMs = 0;
      return;
    }

    Serial.println("[WIFI] Attempt failed.");
    return;
  }

  // Batch exhausted -> 1h cooldown (and eventual restart)
  batchActive = false;
  failedBatches++;
  lastAttemptMs = 0;
  Serial.printf("[WIFI] Batch failed (%u/%u). Cooling down 1 hour...\n",
                failedBatches, MAX_FAILED_BATCHES_BEFORE_RESTART);

  if (failedBatches >= MAX_FAILED_BATCHES_BEFORE_RESTART) {
    Serial.println("[WIFI] 5 failed hourly batches -> restarting device.");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP.restart();
  }
}

void updateWifiLed() {
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(ST_LED, HIGH);  // ON when connected
    led_ST = true;
  } else {
    digitalWrite(ST_LED, LOW);   // OFF when not connected
    led_ST = false;
  }
}

