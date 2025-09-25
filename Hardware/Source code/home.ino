/**
 * @file Refactored_ESP8266_Firebase.ino
 * @brief This code has been refactored to use polling instead of a persistent stream.
 * @details The original stream-based approach has been replaced with a periodic
 * polling mechanism. The device now reads the `/BUTTONS` path at a set
 * interval, processes the first command it finds, and then deletes that
 * command from Firebase to prevent duplication. This makes the logic
 * more resilient to brief disconnects.
 */

#define FIRMWARE_VERSION 6.5 // Version updated to handle string/int polling

#include <WiFiManager.h>
#include <EEPROM.h>
#include <FirebaseESP8266.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <Ticker.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <WiFiClientSecure.h>
#include <ESP8266httpUpdate.h>

// Pin Definitions
#define wifiLed 2           // D4: WiFi status LED
#define TRIGGER_PIN 14      // D5: Pin to trigger WiFiManager AP mode
#define TRANSFER_IND_PIN 12 // D6: Data transfer indication LED

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
unsigned long offlineStartTime = 0;
const unsigned long OFFLINE_TIMEOUT = 2UL * 60UL * 1000UL;
const unsigned long PRINT_INTERVAL = 2000UL;
unsigned long lastPollMillis = 0;
const unsigned long POLL_INTERVAL = 1000; // Poll Firebase every 1 second

// Globals
char sys_id[32];
volatile bool buttonPressed = false;
bool signupOK = false;
bool online_flag = false;

uint8_t time_hours = 0, time_mint = 0, currentMonth = 0, currentDay = 0;
uint16_t currentYear = 0;

uint8_t on_transfer_counter[MAX_SWITCH_NO + 1] = {0};
uint8_t off_transfer_counter[MAX_SWITCH_NO + 1] = {0};
unsigned long lastOnMillis[MAX_SWITCH_NO + 1] = {0};
unsigned long lastOffMillis[MAX_SWITCH_NO + 1] = {0};

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
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Network & Time Clients
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
Ticker timer;
WiFiManager wm;

// ======================= Task Queue (Replaces Command Queue) =======================
// This new task system ensures that only one blocking operation is processed at a time,
// keeping the main loop fast and responsive.
enum TaskType 
{
  TASK_NONE,
  TASK_SWITCH,
  TASK_UPDATE,
  TASK_RESET,
  TASK_SET_TIME,
  TASK_ONLINE
};

struct Task 
{
  TaskType type;
  int switch_no;
  int value;
};

const int MAX_QUEUE = 16;
Task taskQueue[MAX_QUEUE];
int queueHead = 0;
int queueTail = 0;

// A simple state to ensure we only process one task at a time
bool isTaskRunning = false;

void enqueueTask(TaskType type, int sw, int val) 
{
  int next = (queueTail + 1) % MAX_QUEUE;
  if (next != queueHead) 
  { // not full
    taskQueue[queueTail].type = type;
    taskQueue[queueTail].switch_no = sw;
    taskQueue[queueTail].value = val;
    queueTail = next;
  } 
  else 
  {
    Serial.println(F("Task queue full, dropping task"));
  }
}

bool dequeueTask(Task &task) 
{
  if (queueHead == queueTail) return false; // empty
  task = taskQueue[queueHead];
  queueHead = (queueHead + 1) % MAX_QUEUE;
  return true;
}

// =================================== Helper Functions ===================================
void IRAM_ATTR handleButtonPress() 
{
  buttonPressed = true;
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
  Serial.println();
  Serial.println(path);
}

void set_time() {
  if (buttonPressed) return;
  if (currentYear < 2024) 
  {
    timeClient.update();
  }
  time_hours = timeClient.getHours();
  time_mint = timeClient.getMinutes();
  unsigned long epochTime = timeClient.getEpochTime();
  currentYear = year(epochTime);
  currentMonth = month(epochTime);
  currentDay = day(epochTime);
}

// ============================= Optimized Firebase Functions =============================

/**
 * @brief Sends switch state and notification in a single batched update.
 * @details This is the key optimization. Instead of 3 separate blocking calls,
 * we create a JSON object and send it in one `updateNode` request. This
 * drastically reduces the time the loop is blocked.
 */
bool send_switch_state_optimized(int switch_no, bool state) 
{
  if (!Firebase.ready()) return false;

  char timeBuf[48];
  snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u_%02u/%02u/%04u", time_hours, time_mint, currentDay, currentMonth, currentYear);

  char basePath[MAX_PATH_LEN];
  snprintf(basePath, sizeof(basePath), "SYSTEM/%s", sys_id);

  FirebaseJson json;
  char switchPath[64];

  // Path for BUTTONS/SW<num>
  snprintf(switchPath, sizeof(switchPath), "BUTTONS/ONLINE_STATUS");
  json.set(switchPath, 1); // Set button to "loading" state

  // Path for SWITCH_STATE/SW<num>
  snprintf(switchPath, sizeof(switchPath), "/SWITCH_STATE/SW%d", switch_no);
  String path =  "/SYSTEM/" + String(sys_id) +String(switchPath);
  Firebase.setInt(fbdo, path, (int)state);

  // Path for NOTIFICATION/SW<num>
  snprintf(switchPath, sizeof(switchPath), "NOTIFICATION/SW%d", switch_no);
  json.set(switchPath, (int)state);

  // Path for NOTIFICATION_TIME
  json.set("NOTIFICATION/NOTIFICATION_TIME", timeBuf);

 // Serial.println(F("Sending batched update to Firebase..."));
  bool success = Firebase.updateNode(fbdo, basePath, json);
  if (!success) 
  {
      Serial.print(F("Batched update failed: "));
      Serial.println(fbdo.errorReason());
  }
  return success;
}


/**
 * @brief Reads all schedules for the device in a single Firebase request.
 * @details Fetches the entire 'SCHEDULED' node as a JSON object, then parses it locally.
 * This is much faster than reading each schedule field individually.
 */
void readSchedulesFromFirebase() 
{
    if (!Firebase.ready()) return;
    
    char path[MAX_PATH_LEN];
    snprintf(path, sizeof(path), "SYSTEM/%s/SCHEDULED", sys_id);
    
    Serial.println(F("Fetching all schedules..."));
    if (Firebase.getJSON(fbdo, path)) 
    {
        FirebaseJson &json = fbdo.jsonObject();
        FirebaseJsonData result;

        for (int i = 1; i <= MAX_SWITCH_NO; i++) 
        {
            char sw_key[8];
            snprintf(sw_key, sizeof(sw_key), "SW%d", i);
            
            if (json.get(result, sw_key)) 
            {
                FirebaseJson scheduleJson;
                result.get<FirebaseJson>(scheduleJson);
                schedules[i].valid = true;
                
                scheduleJson.get(result, "ON_HOUR");
                if(result.success) schedules[i].onHour = result.intValue;

                scheduleJson.get(result, "ON_MIN");
                if(result.success) schedules[i].onMin = result.intValue;

                scheduleJson.get(result, "OFF_HOUR");
                if(result.success) schedules[i].offHour = result.intValue;

                scheduleJson.get(result, "OFF_MIN");
                if(result.success) schedules[i].offMin = result.intValue;
            } 
            else 
            {
                schedules[i].valid = false;
            }
        }
        Serial.println(F("Schedules updated successfully."));
    } 
    else 
    {
        Serial.print(F("Failed to fetch schedules: "));
        Serial.println(fbdo.errorReason());
    }
}


void readlink_from_firebase() 
{
  if (!Firebase.ready()) return;
  EEPROM.put(EEPROM_UPDATE, 1);
  const char *path = "ADDITIONAL_INFO/UPDATE_LINK";
  if (Firebase.getString(fbdo, path)) 
  {
    if (fbdo.dataType() == "string") 
    {
      String link = fbdo.stringData();
      size_t len = link.length();
      if (len > 240) len = 240;
      for (size_t i = 0; i < len; ++i) 
      {
        EEPROM.write(UPDATE_LINK_ADDER + i, link[i]);
      }
      EEPROM.write(UPDATE_LINK_ADDER + len, '\0');
      EEPROM.commit();

      // Set last update time
      char timeBuffer[48];
      snprintf(timeBuffer, sizeof(timeBuffer), "%02u:%02u_%02u/%02u/%u", time_hours, time_mint, currentDay, currentMonth, currentYear);
      char lastUpdatePath[MAX_PATH_LEN];
      snprintf(lastUpdatePath, sizeof(lastUpdatePath), "SYSTEM/%s/LAST_UPDATE_TIME", sys_id);
      Firebase.setString(fbdo, lastUpdatePath, timeBuffer);

      char updFlag[MAX_PATH_LEN];
      snprintf(updFlag, sizeof(updFlag), "SYSTEM/%s/BUTTONS/UPDATE", sys_id);
      Firebase.setInt(fbdo, updFlag, 0);

      delay(500);
      ESP.restart();
    }
  } 
  else 
  {
    Serial.print(F("Failed to fetch update link: "));
    Serial.println(fbdo.errorReason());
  }
}

void checkForUpdates(const char *binUrl) 
{
    Serial.println(F("Checking for updates..."));
    // The endStream call is harmless if no stream is active.
    Firebase.endStream(fbdo); 
    fbdo.clear();

    WiFiClientSecure otaClient;
    otaClient.setInsecure();

    unsigned long startTime = millis();
    t_httpUpdate_return result = ESPhttpUpdate.update(otaClient, binUrl);
    
    if (millis() - startTime >= UPDATE_TIMEOUT) 
    {
        Serial.println(F("Update timeout occurred"));
    }

    switch (result) 
    {
        case HTTP_UPDATE_FAILED:
            Serial.printf("Update failed. Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println(F("No updates available."));
            break;
        case HTTP_UPDATE_OK:
            Serial.println(F("Update successful. Rebooting..."));
            break;
    }
}


// ================================= Polling and Task Handling =================================
/**
 * @brief Polls the Firebase 'BUTTONS' path for commands.
 * @details Instead of a persistent stream, this function periodically reads the
 * 'BUTTONS' node. If it finds a command, it enqueues the task and deletes
 * the node to prevent reprocessing. It processes one command per call.
 */
void poll_firebase_buttons() 
{
    // Skip if a task is running or Firebase not ready
    if (isTaskRunning || !Firebase.ready()) return;

    // Skip if the task queue is not empty
    if (queueHead != queueTail) return;

    char path[MAX_PATH_LEN];
    snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS", sys_id);

    // Fetch JSON from Firebase
    if (!Firebase.getJSON(fbdo, path)) 
    {
        Serial.print(F("Polling failed: "));
        Serial.println(fbdo.errorReason());
        return;
    }

    if (fbdo.dataType() != "json") 
    {
        Serial.print(F("Unexpected data type: "));
        Serial.println(fbdo.dataType());
        return;
    }

    FirebaseJson &json = fbdo.jsonObject();
    size_t len = json.iteratorBegin();

    String key, value_str;
    int type;
    bool command_found = false;

    for (size_t i = 0; i < len; i++) 
    {
        json.iteratorGet(i, type, key, value_str);

        // Trim whitespace and remove hidden characters
        key.trim();
        key.replace("\r", "");
        key.replace("\n", "");

        // Debug: print key/value/type
        // Serial.print(F("Key: ")); Serial.print(key);
        // Serial.print(F(" | Type: ")); Serial.print(type);
        // Serial.print(F(" | Value: ")); Serial.println(value_str);

        // Only process int, string, or bool
        // if (type != FirebaseJson::JSON_INT &&
        //     type != FirebaseJson::JSON_STRING &&
        //     type != FirebaseJson::JSON_BOOL) {
        //     continue;
        // }

        int value = value_str.toInt();
        if (type == FirebaseJson::JSON_BOOL) 
        {
            value = (value_str == "true") ? 1 : 0;
        }
       // key = "SW12";
       // Serial.println("im here");

        // --- Only process SW keys ---
        if (key.startsWith("SW")) 
        {
           // Serial.println("im here");
            int sw_no = key.substring(2).toInt();
            if (value == 0 || value == 1) 
            {
               // Serial.println(F("SW command found!"));
                enqueueTask(TASK_SWITCH, sw_no, value);

                char deletePath[MAX_PATH_LEN];
                snprintf(deletePath, sizeof(deletePath), "%s/%s", path, key.c_str());
                Firebase.deleteNode(fbdo, deletePath);

                // Serial.print(F("Enqueued SW task: "));
                // Serial.print(key);
                // Serial.print(F(" = "));
                // Serial.println(value);

                command_found = true;
                break; // process only one SW command per poll
            }
        }
          else if (key.startsWith("SET_TIME")) 
          {
           // Serial.println("im here");
            int sw_no = key.substring(2).toInt();
            if (value == 1) {
                enqueueTask(TASK_SET_TIME, 0, value);

                char deletePath[MAX_PATH_LEN];
                snprintf(deletePath, sizeof(deletePath), "%s/%s", path, key.c_str());
                Firebase.deleteNode(fbdo, deletePath);

                command_found = true;
                break; // process only one SW command per poll
            }
        }
        else if (key.startsWith("UPDATE")) 
        {
           // Serial.println("im here");
            int sw_no = key.substring(2).toInt();
            if (value == 1) 
            {
               // Serial.println(F("SW command found!"));
                enqueueTask(TASK_UPDATE, 0, value);

                char deletePath[MAX_PATH_LEN];
                snprintf(deletePath, sizeof(deletePath), "%s/%s", path, key.c_str());
                Firebase.deleteNode(fbdo, deletePath);

                // Serial.print(F("Enqueued SW task: "));
                // Serial.print(key);
                // Serial.print(F(" = "));
                // Serial.println(value);

                command_found = true;
                break; // process only one SW command per poll
            }
        }
        else if (key.startsWith("ONLINE_STATUS")) 
        {
           // Serial.println("im here");
            int sw_no = key.substring(2).toInt();
            if (value == 0 || value == 3) 
            {
               // Serial.println(F("SW command found!"));
                enqueueTask(TASK_ONLINE, 0, value);

                char deletePath[MAX_PATH_LEN];
                snprintf(deletePath, sizeof(deletePath), "%s/%s", path, key.c_str());
                Firebase.deleteNode(fbdo, deletePath);

                // Serial.print(F("Enqueued SW task: "));
                // Serial.print(key);
                // Serial.print(F(" = "));
                // Serial.println(value);

                command_found = true;
                break; // process only one SW command per poll
            }
        }
        else if (key.startsWith("RESET")) 
        {
           // Serial.println("im here");
            int sw_no = key.substring(2).toInt();
            if (value == 0) 
            {
               // Serial.println(F("SW command found!"));
                enqueueTask(TASK_RESET, 0, value);

                char deletePath[MAX_PATH_LEN];
                snprintf(deletePath, sizeof(deletePath), "%s/%s", path, key.c_str());
                Firebase.deleteNode(fbdo, deletePath);

                // Serial.print(F("Enqueued SW task: "));
                // Serial.print(key);
                // Serial.print(F(" = "));
                // Serial.println(value);

                command_found = true;
                break; // process only one SW command per poll
            }
        }
    }

    json.iteratorEnd();

    // if (!command_found && len > 0) {
    //     Serial.println(F("Polling found JSON, but no valid SW commands."));
    // }
}





/**
 * @brief The non-blocking task processor.
 * @details Processes one task from the queue at a time. Once a task is finished, 
 * it sets `isTaskRunning` to false, allowing the next task to be processed.
 */
void task_processor() 
{
    if (isTaskRunning || !Firebase.ready()) 
    {
        return;
    }

    Task task;
    if (dequeueTask(task)) 
    {
        isTaskRunning = true; // Mark that we are busy
        
        switch (task.type) 
        {
            case TASK_SWITCH: 
            {
                digitalWrite(TRANSFER_IND_PIN, LOW);
                
                // Send to serial twice
                serial_out(task.switch_no, task.value);
                delay(1000); // Small delay between serial sends if needed
                serial_out(task.switch_no, task.value);
                
                // Send one optimized update to Firebase
                send_switch_state_optimized(task.switch_no, task.value);

                digitalWrite(TRANSFER_IND_PIN, HIGH);
                break;
            }

            case TASK_UPDATE:
                if (task.value == 1) 
                {
                    Serial.println(F("UPDATING ....."));
                    readlink_from_firebase();
                }
                break;

            case TASK_RESET:
                if (task.value == 1) 
                {
                    Serial.println(F("Reset command received. Restarting..."));
                    // The command node was already deleted by the polling function.
                    ESP.restart();
                }
                break;

            case TASK_SET_TIME:
                if (task.value == 1) 
                {
                    readSchedulesFromFirebase(); // Uses the new optimized function
                    Blink_led(1, 100);
                }
                break;

            case TASK_ONLINE: 
            {
                char path[MAX_PATH_LEN];
                snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/ONLINE_STATUS", sys_id);
                Firebase.setInt(fbdo, path, 1);
                break;
            }

            default:
                break;
        }

        isTaskRunning = false; // Task is complete, ready for the next one
    }
}

// Globals (add with your other arrays)
unsigned long lastOnTriggerMinute[MAX_SWITCH_NO + 1] = {0};
unsigned long lastOffTriggerMinute[MAX_SWITCH_NO + 1] = {0};

void applySchedules() 
{
  // Compute current minute number from epoch time
  unsigned long epochTime = timeClient.getEpochTime();
  unsigned long currentMinute = epochTime / 60UL;

  for (int i = 1; i <= MAX_SWITCH_NO; i++) 
  {
    if (!schedules[i].valid) continue;

    // --- ON schedule ---
    if (time_hours == schedules[i].onHour && time_mint == schedules[i].onMin) 
    {
      if (lastOnTriggerMinute[i] != currentMinute) 
      {
        enqueueTask(TASK_SWITCH, i, 1);   // will print twice inside task_processor()
        lastOnTriggerMinute[i] = currentMinute;
      }
    }

    // --- OFF schedule ---
    if (time_hours == schedules[i].offHour && time_mint == schedules[i].offMin) 
    {
      if (lastOffTriggerMinute[i] != currentMinute) 
      {
        enqueueTask(TASK_SWITCH, i, 0);   // will print twice inside task_processor()
        lastOffTriggerMinute[i] = currentMinute;
      }
    }
  }
}



// =================================== System Setup ===================================


void readSysIdFromEEPROM() 
{
  // Read bytes from EEPROM into sys_id
  for (int i = 0; i < sizeof(sys_id) - 1; i++) 
  {
    sys_id[i] = EEPROM.read(SYS_ID_ADDR + i);
  }
  sys_id[sizeof(sys_id) - 1] = '\0'; // Null-terminate

  // Check if empty or uninitialized (0xFF or '\0')
  bool isEmpty = true;
  for (int i = 0; i < sizeof(sys_id) - 1; i++) 
  {
    if (sys_id[i] != 0xFF && sys_id[i] != '\0') 
    {
      isEmpty = false;
      break;
    }
  }

  // If empty, set default "test"
  if (isEmpty) 
  {
    strcpy(sys_id, "test");
  }
}

String readUpdateLinkFromEEPROM_String() 
{
  String link = "";
  for (int i = 0; i < 100; i++) 
  {
    char c = EEPROM.read(UPDATE_LINK_ADDER + i);
    if (c == '\0') break;
    link += c;
  }
  return link;
}
void setupConfigPortal() 
{
  Firebase.endStream(fbdo);
  fbdo.clear();
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
  if (!wm2.startConfigPortal(AP_ID, AP_PASS)) 
  {
    Serial.println(F("Failed to connect through Config Portal"));
  }
  strncpy(sys_id, sys_id_param.getValue(), sizeof(sys_id) - 1);
  sys_id[sizeof(sys_id) - 1] = '\0';
  for (int i = 0; i < sizeof(sys_id); i++) 
  {
    EEPROM.write(SYS_ID_ADDR + i, sys_id[i]);
  }
  EEPROM.commit();
  Blink_led(0, 500);
  ESP.restart();
}

void resetSwitchesIfPowerOn() 
{
  String reason = ESP.getResetReason();
  Serial.print(F("Reset Reason: "));
  Serial.println(reason);

  if (reason.indexOf("Power") != -1 || reason.indexOf("External") != -1) 
  {
    Serial.println(F("Power-on detected. Resetting all switch states..."));
    FirebaseJson json;
    char path[MAX_PATH_LEN];
    for (int i = 1; i <= MAX_SWITCH_NO; i++) 
    {
        // snprintf(path, sizeof(path), "BUTTONS/SW%d", i);
        // json.set(path, 3);
        snprintf(path, sizeof(path), "SWITCH_STATE/SW%d", i);
        json.set(path, 3);
    }
    snprintf(path, sizeof(path), "SYSTEM/%s", sys_id);
    if (!Firebase.updateNode(fbdo, path, json)) 
    {
        Serial.println(F("Failed to reset switch states on power on."));
    }
  }
}

bool setup_firebase() 
{
  timeClient.begin();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) 
  {
    signupOK = true;
    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    delay(1000);
    Serial.println(F("Sign-in successful!"));

    // Set initial online status and system info
    FirebaseJson json;
    //json.set("BUTTONS/UPDATE", 0);
    //json.set("BUTTONS/RESET", 0);
    //json.set("BUTTONS/SET_TIME", 0);
    json.set("BUTTONS/ONLINE_STATUS", 1);
    json.set("SYSTEM_INFO/FIRMWARE_VERSION", FIRMWARE_VERSION);
    json.set("SYSTEM_INFO/RESET_REASON", ESP.getResetInfo());
    json.set("SYSTEM_INFO/FREE_RAM", String(ESP.getFreeHeap()));
    
    char path[MAX_PATH_LEN];
    snprintf(path, sizeof(path), "SYSTEM/%s", sys_id);
    online_flag = Firebase.updateNode(fbdo, path, json);
    
  } 
  else 
  {
    Serial.println(F("Firebase sign-in failed!"));
    online_flag = false;
  }
  return online_flag;
}

void setup() 
{
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, HIGH);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(TRANSFER_IND_PIN, OUTPUT);
  digitalWrite(TRANSFER_IND_PIN, HIGH);
  Serial.begin(9600);
  delay(1000);

  EEPROM.begin(EEPROM_SIZE);
  uint8_t update_ota = EEPROM.read(EEPROM_UPDATE);
  readSysIdFromEEPROM();

  wm.setConfigPortalTimeout(10);
  if (wm.autoConnect(AP_ID, AP_PASS)) 
  {
    Serial.println(F("WiFi connected"));
    Serial.print(F("Firmware version: "));
    Serial.println(FIRMWARE_VERSION);
    Serial.print(F("System ID: "));
    Serial.println(sys_id);

    if (update_ota == 1) 
    {
      String link = readUpdateLinkFromEEPROM_String();
      EEPROM.write(EEPROM_UPDATE, 0);
      EEPROM.commit();
      checkForUpdates(link.c_str());
    } 
    else if (EEPROM.read(EEPROM_UPDATE) != 0) 
    {
      EEPROM.write(EEPROM_UPDATE, 0);
      EEPROM.commit();
    }

    if (setup_firebase()) 
    {
      resetSwitchesIfPowerOn();
      readSchedulesFromFirebase();
      delay(2000);
      digitalWrite(wifiLed, LOW);
    }
  } 
  else 
  {
    digitalWrite(wifiLed, HIGH);
  }

  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), handleButtonPress, FALLING);
}

void loop() 
{
  if (buttonPressed) 
  {
    delay(1000);
    if (digitalRead(TRIGGER_PIN) == LOW) 
    {
      buttonPressed = false;
      setupConfigPortal();
    }
  }

  if (WiFi.status() != WL_CONNECTED) 
  {
    digitalWrite(wifiLed, HIGH);
    online_flag = false;
    if (offlineStartTime == 0) offlineStartTime = millis();
    if (millis() - offlineStartTime >= OFFLINE_TIMEOUT) 
    {
      ESP.restart();
    }
  } 
  else 
  {
    if (!online_flag) 
    {
      online_flag = setup_firebase();
    }
    digitalWrite(wifiLed, LOW);
    offlineStartTime = 0;

    // --- Main non-blocking loop ---
    unsigned long currentMillis = millis();
    if (currentMillis - lastPollMillis >= POLL_INTERVAL) 
    {
        lastPollMillis = currentMillis;
        poll_firebase_buttons(); // Poll for new commands
    }
    
    task_processor(); // Processes one queued task
    set_time();         // Updates internal clock
    applySchedules();   // Applies any time-based schedules
  }
}

