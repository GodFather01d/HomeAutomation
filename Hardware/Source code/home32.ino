/**
 * @file Refactored_ESP32_Firebase.ino
 * @brief This code is a conversion of an ESP8266 project to run on the ESP32 platform using FreeRTOS and Firebase Streaming.
 * @details The polling mechanism has been replaced with a persistent Firebase Realtime Database stream.
 * The device now listens for changes on the /BUTTONS path and reacts instantly. A stream callback
 * function parses incoming commands, enqueues them for processing, and immediately deletes
 * the command from Firebase to prevent re-execution. This provides near-instantaneous response times.
 *
 * ESP32 & RTOS Conversion Notes:
 * - Replaced polling task with a real-time Firebase stream for commands.
 * - Added a stream callback to handle incoming data asynchronously.
 * - Implemented automatic stream reconnection logic in the WiFi management task.
 * - Kept FreeRTOS tasks for command processing, time/schedules, and WiFi management.
 */

#define FIRMWARE_VERSION 9.5 // Added blocking NTP sync to fix SSL errors on stream reconnect

#include <WiFi.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>

// Pin Definitions
#define wifiLed 2           // D2: WiFi status LED (Built-in LED on many ESP32s)
#define TRIGGER_PIN 13      // D14: Pin to trigger WiFiManager AP mode
#define TRANSFER_IND_PIN 12 // D12: Data transfer indication LED

// Firebase Credentials
#define API_KEY "AIzaSyAQh-tAWmKpcGzwmXISv0-aY-Q4s6MAxZc"
#define DATABASE_URL "homeautomation-e6ef2-default-rtdb.asia-southeast1.firebasedatabase.app/"

// EEPROM Configuration
#define EEPROM_SIZE 512
#define SYS_ID_ADDR 0
#define EEPROM_UPDATE 30
#define UPDATE_LINK_ADDER 35

// System Limits
#define MAX_SWITCH_NO 25
#define MAX_PATH_LEN 120

// WiFi & Update Configuration
const char AP_ID[] = "JT_WiFi_SETUP";
const char AP_PASS[] = "Jyoti@2000";
const long utcOffsetInSeconds = 19800;
const unsigned long UPDATE_TIMEOUT = 20000;

// Timers
const unsigned long OFFLINE_TIMEOUT = 2UL * 60UL * 1000UL;

// Globals
char sys_id[32];
volatile bool buttonPressed = false;
volatile bool stream_timed_out = false; // Flag to indicate a stream timeout event
bool signupOK = false;
bool online_flag = false;
uint8_t time_hours = 0, time_mint = 0, currentMonth = 0, currentDay = 0;
uint16_t currentYear = 0;

// Schedule struct
struct Schedule 
{
  uint8_t onHour;
  uint8_t onMin;
  uint8_t offHour;
  uint8_t offMin;
  bool valid;
};
Schedule schedules[MAX_SWITCH_NO + 1];

// Firebase objects
FirebaseData fbdo;      // Regular read/write
FirebaseData stream;    // Dedicated stream
FirebaseAuth auth;
FirebaseConfig config;


// Network & Time Clients
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
WiFiManager wm;

// ======================= RTOS Task & Queue Definitions =======================
enum TaskType 
{
  TASK_NONE, TASK_SWITCH, TASK_UPDATE, TASK_RESET, TASK_SET_TIME, TASK_ONLINE
};

struct Task 
{
  TaskType type;
  int switch_no;
  int value;
};

QueueHandle_t taskQueueHandle;
SemaphoreHandle_t firebaseMutex;
SemaphoreHandle_t serialMutex;

// Forward declaration for stream callback
void streamCallback(StreamData data);

// =================================== Helper Functions ===================================
void safeSerialPrint(const String &msg) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(msg);
    xSemaphoreGive(serialMutex);
  }
}

void safeSerialPrintln(const String &msg) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(msg);
    xSemaphoreGive(serialMutex);
  }
}

void IRAM_ATTR handleButtonPress() 
{
  buttonPressed = true;
}

String getResetReasonString() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "Unknown";
    case ESP_RST_POWERON:   return "Power On";
    case ESP_RST_EXT:       return "External Reset";
    case ESP_RST_SW:        return "Software Reset";
    case ESP_RST_PANIC:     return "Panic/Crash";
    case ESP_RST_INT_WDT:   return "Interrupt Watchdog";
    case ESP_RST_TASK_WDT:  return "Task Watchdog";
    case ESP_RST_WDT:       return "Other Watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep Sleep Wakeup";
    case ESP_RST_BROWNOUT:  return "Brownout";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "No reason";
  }
}

void Blink_led(int state, int speed) 
{
  for (int i = 0; i <= 3; i++) 
  {
    digitalWrite(wifiLed, state);
    delay(speed);
    digitalWrite(wifiLed, !state);
    delay(speed);
  }
}

void serial_out(int switch_no, int state) 
{
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%02d", switch_no);
  char path[48];
  snprintf(path, sizeof(path), "%s_%s_%d", sys_id, buffer, state);
  safeSerialPrintln("");
  safeSerialPrintln(path);
}

void set_time() {
  if (buttonPressed) return;
  
  timeClient.update();
  
  time_hours = timeClient.getHours();
  time_mint = timeClient.getMinutes();
  unsigned long epochTime = timeClient.getEpochTime();
  currentYear = year(epochTime);
  currentMonth = month(epochTime);
  currentDay = day(epochTime);
}

// ============================= Firebase & System Functions =============================

bool send_switch_state_optimized(int switch_no, bool state) 
{
  if (xSemaphoreTake(firebaseMutex, portMAX_DELAY) != pdTRUE) return false;
  if (!Firebase.ready()) {
    xSemaphoreGive(firebaseMutex);
    return false;
  }

  char timeBuf[48];
  snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u_%02u/%02u/%04u", time_hours, time_mint, currentDay, currentMonth, currentYear);
  char basePath[MAX_PATH_LEN];
  snprintf(basePath, sizeof(basePath), "SYSTEM/%s", sys_id);

  FirebaseJson json;
  char switchPath[64];

  snprintf(switchPath, sizeof(switchPath), "BUTTONS/ONLINE_STATUS");
  json.set(switchPath, 1);
  
  snprintf(switchPath, sizeof(switchPath), "/SWITCH_STATE/SW%d", switch_no);
  String path =  "/SYSTEM/" + String(sys_id) + String(switchPath);
  Firebase.setInt(fbdo, path, (int)state);

  snprintf(switchPath, sizeof(switchPath), "NOTIFICATION/SW%d", switch_no);
  json.set(switchPath, (int)state);
  json.set("NOTIFICATION/NOTIFICATION_TIME", timeBuf);

  bool success = Firebase.updateNode(fbdo, basePath, json);
  if (!success) {
      safeSerialPrint(F("Batched update failed: "));
      safeSerialPrintln(fbdo.errorReason());
  }
  xSemaphoreGive(firebaseMutex);
  return success;
}

void readSchedulesFromFirebase() 
{
    if (xSemaphoreTake(firebaseMutex, portMAX_DELAY) != pdTRUE) return;
    if (!Firebase.ready()) {
        xSemaphoreGive(firebaseMutex);
        return;
    }
    
    char path[MAX_PATH_LEN];
    snprintf(path, sizeof(path), "SYSTEM/%s/SCHEDULED", sys_id);
    
    safeSerialPrintln(F("Fetching all schedules..."));
    if (Firebase.getJSON(fbdo, path)) 
    {
        FirebaseJson &json = fbdo.jsonObject();
        FirebaseJsonData result;
        for (int i = 1; i <= MAX_SWITCH_NO; i++) {
            char sw_key[8];
            snprintf(sw_key, sizeof(sw_key), "SW%d", i);
            if (json.get(result, sw_key)) {
                FirebaseJson scheduleJson;
                result.get<FirebaseJson>(scheduleJson);
                schedules[i].valid = true;
                if(scheduleJson.get(result, "ON_HOUR")) schedules[i].onHour = result.intValue;
                if(scheduleJson.get(result, "ON_MIN")) schedules[i].onMin = result.intValue;
                if(scheduleJson.get(result, "OFF_HOUR")) schedules[i].offHour = result.intValue;
                if(scheduleJson.get(result, "OFF_MIN")) schedules[i].offMin = result.intValue;
            } else {
                schedules[i].valid = false;
            }
        }
        safeSerialPrintln(F("Schedules updated successfully."));
    } else {
        safeSerialPrint(F("Failed to fetch schedules: "));
        safeSerialPrintln(fbdo.errorReason());
    }
    xSemaphoreGive(firebaseMutex);
}

void readlink_from_firebase() 
{
  if (xSemaphoreTake(firebaseMutex, portMAX_DELAY) != pdTRUE) return;
  if (!Firebase.ready()) {
    xSemaphoreGive(firebaseMutex);
    return;
  }
  EEPROM.put(EEPROM_UPDATE, 1);
  const char *path = "ADDITIONAL_INFO/UPDATE_LINK32";
  if (Firebase.getString(fbdo, path)) {
    if (fbdo.dataType() == "string") {
      String link = fbdo.stringData();
      size_t len = link.length();
      if (len > 240) len = 240;
      for (size_t i = 0; i < len; ++i) EEPROM.write(UPDATE_LINK_ADDER + i, link[i]);
      EEPROM.write(UPDATE_LINK_ADDER + len, '\0');
      EEPROM.commit();

      char timeBuffer[48];
      snprintf(timeBuffer, sizeof(timeBuffer), "%02u:%02u_%02u/%02u/%u", time_hours, time_mint, currentDay, currentMonth, currentYear);
      char lastUpdatePath[MAX_PATH_LEN];
      snprintf(lastUpdatePath, sizeof(lastUpdatePath), "SYSTEM/%s/LAST_UPDATE_TIME", sys_id);
      Firebase.setString(fbdo, lastUpdatePath, timeBuffer);

      char updFlag[MAX_PATH_LEN];
      snprintf(updFlag, sizeof(updFlag), "SYSTEM/%s/BUTTONS/UPDATE", sys_id);
      Firebase.setInt(fbdo, updFlag, 0);

      xSemaphoreGive(firebaseMutex);
      delay(500);
      ESP.restart();
    }
  } else {
    safeSerialPrint(F("Failed to fetch update link: "));
    safeSerialPrintln(fbdo.errorReason());
  }
  xSemaphoreGive(firebaseMutex);
}

void checkForUpdates(const char *binUrl) 
{
    safeSerialPrintln(F("Checking for updates..."));
    xSemaphoreTake(firebaseMutex, portMAX_DELAY);
    Firebase.endStream(stream); 
    xSemaphoreGive(firebaseMutex);

    WiFiClientSecure otaClient;
    otaClient.setInsecure();
    
    HTTPUpdateResult result = httpUpdate.update(otaClient, binUrl);
    
    switch (result) {
        case HTTP_UPDATE_FAILED:
            safeSerialPrintln("Update failed. Error (" + String(httpUpdate.getLastError()) + "): " + httpUpdate.getLastErrorString());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            safeSerialPrintln(F("No updates available."));
            break;
        case HTTP_UPDATE_OK:
            safeSerialPrintln(F("Update successful. Rebooting..."));
            break;
    }
}

// ============================= Firebase Streaming =================================
void streamCallback(StreamData data) {
  if (data.eventType() != "put" || data.dataPath() == "/") return;

  String key = data.dataPath();
  key.remove(0, 1); // remove leading /

  int value = 0;
  if (data.dataType() == "int" || data.dataType() == "integer") value = data.intData();
  else if (data.dataType() == "bool" || data.dataType() == "boolean") value = data.boolData() ? 1 : 0;
  else return;

  Task task = {TASK_NONE, 0, 0};
  bool command_found = false;

  if (key.startsWith("SW")) {
    int sw_no = key.substring(2).toInt();
    if ((value == 0 || value == 1) && sw_no > 0 && sw_no <= MAX_SWITCH_NO) {
      task = {TASK_SWITCH, sw_no, value};
      command_found = true;
    }
  } else if (key.startsWith("SET_TIME") && value == 1) {
    task = {TASK_SET_TIME, 0, value};
    command_found = true;
  } else if (key.startsWith("UPDATE") && value == 1) {
    task = {TASK_UPDATE, 0, value};
    command_found = true;
  } else if (key.startsWith("ONLINE_STATUS") && (value == 0 || value == 3)) {
    task = {TASK_ONLINE, 0, value};
    command_found = true;
  } else if (key.startsWith("RESET") && value == 1) {
    task = {TASK_RESET, 0, value};
    command_found = true;
  }

  if (command_found) {
    xQueueSend(taskQueueHandle, &task, (TickType_t)10);

    // Delete using fbdo safely
    if (xSemaphoreTake(firebaseMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      char deletePath[MAX_PATH_LEN];
      snprintf(deletePath, sizeof(deletePath), "SYSTEM/%s/BUTTONS/%s", sys_id, key.c_str());
      Firebase.deleteNode(fbdo, deletePath); // safe because fbdo is separate
      xSemaphoreGive(firebaseMutex);
    }
  }
}

void startFirebaseStream() {
  if (xSemaphoreTake(firebaseMutex, portMAX_DELAY) == pdTRUE) {
    char streamPath[MAX_PATH_LEN];
    snprintf(streamPath, sizeof(streamPath), "SYSTEM/%s/BUTTONS", sys_id);

    Firebase.setStreamCallback(stream, streamCallback, streamTimeoutCallback);

    if (!Firebase.beginStream(stream, streamPath)) {
      safeSerialPrintln("Could not begin stream: " + stream.errorReason());
         FirebaseJson json;
      char path[MAX_PATH_LEN];
      for (int i = 1; i <= MAX_SWITCH_NO; i++) {
          snprintf(path, sizeof(path), "BUTTONS/SW%d", i);
          json.set(path, 3);
       }
    } else {
      safeSerialPrintln("Stream started on path: " + String(streamPath));
      FirebaseJson json;
      char path[MAX_PATH_LEN];
      for (int i = 1; i <= MAX_SWITCH_NO; i++) {
          snprintf(path, sizeof(path), "BUTTONS/SW%d", i);
          json.set(path, 3);
       }
    }

    xSemaphoreGive(firebaseMutex);
  }
}
void streamTimeoutCallback(bool timeout) {
  if (timeout) {
    safeSerialPrintln("Stream timeout, will reconnect...");
    stream_timed_out = true;
  }
}

// ============================= Schedules and System Setup ===============================
void applySchedules() 
{
  unsigned long epochTime = timeClient.getEpochTime();
  unsigned long currentMinute = epochTime / 60UL;
  static unsigned long lastOnTriggerMinute[MAX_SWITCH_NO + 1] = {0};
  static unsigned long lastOffTriggerMinute[MAX_SWITCH_NO + 1] = {0};

  for (int i = 1; i <= MAX_SWITCH_NO; i++) {
    if (!schedules[i].valid) continue;
    Task task = {TASK_NONE, 0, 0};
    bool schedule_triggered = false;

    if (time_hours == schedules[i].onHour && time_mint == schedules[i].onMin) {
      if (lastOnTriggerMinute[i] != currentMinute) {
        task = {TASK_SWITCH, i, 1};
        lastOnTriggerMinute[i] = currentMinute;
        schedule_triggered = true;
      }
    }
    if (time_hours == schedules[i].offHour && time_mint == schedules[i].offMin) {
      if (lastOffTriggerMinute[i] != currentMinute) {
        task = {TASK_SWITCH, i, 0};
        lastOffTriggerMinute[i] = currentMinute;
        schedule_triggered = true;
      }
    }

    if(schedule_triggered){
      xQueueSend(taskQueueHandle, &task, (TickType_t)10);
    }
  }
}

void readSysIdFromEEPROM() {
  for (int i = 0; i < sizeof(sys_id) - 1; i++) sys_id[i] = EEPROM.read(SYS_ID_ADDR + i);
  sys_id[sizeof(sys_id) - 1] = '\0';
  bool isEmpty = true;
  for (int i = 0; i < sizeof(sys_id) - 1; i++) if (sys_id[i] != 0xFF && sys_id[i] != '\0') isEmpty = false;
  if (isEmpty) strcpy(sys_id, "test");
}

String readUpdateLinkFromEEPROM_String() {
  String link = "";
  for (int i = 0; i < 100; i++) {
    char c = EEPROM.read(UPDATE_LINK_ADDER + i);
    if (c == '\0') break;
    link += c;
  }
  return link;
}

void setupConfigPortal() {
  xSemaphoreTake(firebaseMutex, portMAX_DELAY);
  Firebase.endStream(stream);
  xSemaphoreGive(firebaseMutex);
  timeClient.end();
  WiFi.disconnect(true);
  delay(100);
  Blink_led(0, 500);
  WiFiManager wm2;

  wm2.setCaptivePortalEnable(true);
  WiFiManagerParameter sys_id_param("sys_id", "System ID", sys_id, 32);
  wm2.addParameter(&sys_id_param);
  wm2.setConfigPortalTimeout(180);
  delay(1000);
  if (!wm2.startConfigPortal(AP_ID, AP_PASS)) safeSerialPrintln(F("Failed to connect through Config Portal"));
  strncpy(sys_id, sys_id_param.getValue(), sizeof(sys_id) - 1);
  sys_id[sizeof(sys_id) - 1] = '\0';
  for (int i = 0; i < sizeof(sys_id); i++) EEPROM.write(SYS_ID_ADDR + i, sys_id[i]);
  EEPROM.commit();
  Blink_led(0, 500);
  ESP.restart();
}

void resetSwitchesIfPowerOn() {
  String reason = getResetReasonString();
  safeSerialPrint(F("Reset Reason: "));
  safeSerialPrintln(reason);

  if (reason.indexOf("Power") != -1 || reason.indexOf("External") != -1) {
    safeSerialPrintln(F("Power-on detected. Resetting all switch states..."));
    FirebaseJson json;
    char path[MAX_PATH_LEN];
    for (int i = 1; i <= MAX_SWITCH_NO; i++) {
        snprintf(path, sizeof(path), "SWITCH_STATE/SW%d", i);
        json.set(path, 3);
    }

    snprintf(path, sizeof(path), "BUTTONS/ONLINE_STATUS");
    json.set(path, 1);
    snprintf(path, sizeof(path), "SYSTEM/%s", sys_id);
    if (xSemaphoreTake(firebaseMutex, portMAX_DELAY) == pdTRUE) {
      if (!Firebase.updateNode(fbdo, path, json)) safeSerialPrintln(F("Failed to reset switch states on power on."));
      xSemaphoreGive(firebaseMutex);
    }
  }
}

bool setup_firebase() {
  // timeClient.begin() is now called from setup() before this function
  if (xSemaphoreTake(firebaseMutex, portMAX_DELAY) != pdTRUE) return false;
  
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    safeSerialPrintln(F("Sign-in successful!"));

    FirebaseJson json;
    json.set("BUTTONS/ONLINE_STATUS", 1);
    json.set("SYSTEM_INFO/FIRMWARE_VERSION", FIRMWARE_VERSION);
    json.set("SYSTEM_INFO/RESET_REASON", getResetReasonString());
    json.set("SYSTEM_INFO/FREE_RAM", String(ESP.getFreeHeap()));
    
    char path[MAX_PATH_LEN];
    snprintf(path, sizeof(path), "SYSTEM/%s", sys_id);
    online_flag = Firebase.updateNode(fbdo, path, json);
  } else {
    safeSerialPrint(F("Firebase sign-in failed: "));
    safeSerialPrintln(config.signer.signupError.message.c_str());
    online_flag = false;
  }
  xSemaphoreGive(firebaseMutex);

  return online_flag;
}

// =================================== RTOS Tasks ===================================

void taskProcessorTask(void *parameter) {
  for (;;) {
    Task task;
    if (xQueueReceive(taskQueueHandle, &task, portMAX_DELAY) == pdPASS) {
      switch (task.type) {
        case TASK_SWITCH: {
          digitalWrite(TRANSFER_IND_PIN, LOW);
          serial_out(task.switch_no, task.value);
          delay(1000);
          serial_out(task.switch_no, task.value);
          send_switch_state_optimized(task.switch_no, task.value);
          digitalWrite(TRANSFER_IND_PIN, HIGH);
          break;
        }
        case TASK_UPDATE:
          if (task.value == 1) {
            safeSerialPrintln(F("UPDATING ....."));
            readlink_from_firebase();
          }
          break;
        case TASK_RESET:
          if (task.value == 1) {
            safeSerialPrintln(F("Reset command received. Restarting..."));
            ESP.restart();
          }
          break;
        case TASK_SET_TIME:
          if (task.value == 1) {
            readSchedulesFromFirebase();
            Blink_led(1, 100);
          }
          break;
        case TASK_ONLINE: {
          if (xSemaphoreTake(firebaseMutex, portMAX_DELAY) == pdTRUE) {
            char path[MAX_PATH_LEN];
            snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/ONLINE_STATUS", sys_id);
            Firebase.setInt(fbdo, path, 1);
            xSemaphoreGive(firebaseMutex);
          }
          break;
        }
        default: break;
      }
    }
  }
}

void timeAndScheduleTask(void *parameter) {
  for (;;) {
    if (online_flag) {
      set_time();
      applySchedules();
    }
    vTaskDelay(pdMS_TO_TICKS(30000)); // Update time and check schedules every 30 seconds
  }
}

void wifiManagementTask(void *parameter) {
  unsigned long offlineStartTime = 0;
  for (;;) {
    if (buttonPressed) {
      delay(1000);
      if (digitalRead(TRIGGER_PIN) == LOW) {
        buttonPressed = false;
        setupConfigPortal();
      }
    }

    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(wifiLed, HIGH);
      if(online_flag){ // if it was previously online
          xSemaphoreTake(firebaseMutex, portMAX_DELAY);
          Firebase.endStream(stream);
          xSemaphoreGive(firebaseMutex);
      }
      online_flag = false;
      if (offlineStartTime == 0) offlineStartTime = millis();
      if (millis() - offlineStartTime >= OFFLINE_TIMEOUT) {
        ESP.restart();
      }
    } else {
      if (!online_flag) {
        online_flag = setup_firebase();
      } else {
         if (!stream.httpConnected() || stream_timed_out) {
          safeSerialPrintln("Stream disconnected or timed out. Restarting stream...");
          stream_timed_out = false;

          if (xSemaphoreTake(firebaseMutex, portMAX_DELAY) == pdTRUE) {
            Firebase.endStream(stream); // only end the dedicated stream
            xSemaphoreGive(firebaseMutex);
          }
          vTaskDelay(pdMS_TO_TICKS(200));
          startFirebaseStream();
        }

      }
      digitalWrite(wifiLed, LOW);
      offlineStartTime = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =================================== Main Setup ===================================

void setup() {
  pinMode(wifiLed, OUTPUT); digitalWrite(wifiLed, HIGH);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(TRANSFER_IND_PIN, OUTPUT); digitalWrite(TRANSFER_IND_PIN, HIGH);
  Serial.begin(115200);
  delay(1000);

  // Create mutexes and queue before they are used to prevent crashes.
  firebaseMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  taskQueueHandle = xQueueCreate(10, sizeof(Task));

  EEPROM.begin(EEPROM_SIZE);
  uint8_t update_ota = EEPROM.read(EEPROM_UPDATE);
  readSysIdFromEEPROM();

  wm.setConfigPortalTimeout(180);
  if (wm.autoConnect(AP_ID, AP_PASS)) {
    safeSerialPrintln(F("WiFi connected"));
    safeSerialPrintln("Firmware version: " + String(FIRMWARE_VERSION));
    safeSerialPrintln("System ID: " + String(sys_id));
    
    // Force NTP time sync before doing anything that requires SSL/TLS
    safeSerialPrintln(F("Waiting for NTP time sync..."));
    timeClient.begin();
    set_time();
    while (currentYear < 2024) {
      safeSerialPrintln(F("Time not synced yet, retrying..."));
      timeClient.forceUpdate();
      vTaskDelay(pdMS_TO_TICKS(1000));
      set_time();
    }
    safeSerialPrintln("Time synced: " + String(currentDay) + "/" + String(currentMonth) + "/" + String(currentYear));


    if (update_ota == 1) {
      String link = readUpdateLinkFromEEPROM_String();
      EEPROM.write(EEPROM_UPDATE, 0); EEPROM.commit();
      checkForUpdates(link.c_str());
    } else if (EEPROM.read(EEPROM_UPDATE) != 0) {
      EEPROM.write(EEPROM_UPDATE, 0); EEPROM.commit();
    }
    
    // Create RTOS tasks


    if (setup_firebase()) {
      resetSwitchesIfPowerOn();
      readSchedulesFromFirebase();
      startFirebaseStream();
      digitalWrite(wifiLed, LOW);
    }
  } else {
    digitalWrite(wifiLed, HIGH);
  }
    xTaskCreatePinnedToCore(wifiManagementTask, "WiFiTask", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(taskProcessorTask, "ProcessorTask", 10240, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(timeAndScheduleTask, "TimeTask", 4096, NULL, 1, NULL, 0);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), handleButtonPress, FALLING);
}


void loop() {
 // resetSwitchesIfPowerOn();
  // Empty. The FreeRTOS scheduler handles all tasks.
}

