#define FIRMWARE_VERSION 5.5

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

#define wifiLed 2            //  D4   wifi led
#define TRIGGER_PIN 14       // wifi setup pin AP
#define TRANSFER_IND_PIN 12  //  D6   data transfer indication pin

#define API_KEY "AIzaSyAQh-tAWmKpcGzwmXISv0-aY-Q4s6MAxZc"
#define DATABASE_URL "homeautomation-e6ef2-default-rtdb.asia-southeast1.firebasedatabase.app/"

#define EEPROM_SIZE 512
#define SYS_ID_ADDR 0
#define EEPROM_UPDATE 30
#define UPDATE_LINK_ADDER 35
#define MAX_SWITCH_NO 25
#define MAX_PATH_LEN 120
#define STREAM_RETRY_LIMIT 4

const char AP_ID[] = "JT_WiFi_SETUP";
const char AP_PASS[] = "Jyoti@2000";
const long utcOffsetInSeconds = 19800;
unsigned long startTime;
const unsigned long timeout = 20000;

unsigned long offlineStartTime = 0;
const unsigned long OFFLINE_TIMEOUT = 2UL * 60UL * 1000UL;
char sys_id[32];
volatile bool buttonPressed = false;
bool signupOK = false;
bool online_flag = false;

uint8_t update_ota = 1;
uint8_t curr_switch_on = 0, curr_switch_off = 0;
uint8_t time_hours = 0, time_mint = 0, currentMonth = 0, currentDay = 0;
uint16_t offline_counter = 0, currentYear = 0;

uint8_t on_transfer_counter[MAX_SWITCH_NO + 1] = {0};
uint8_t off_transfer_counter[MAX_SWITCH_NO + 1] = {0};
unsigned long lastOnMillis[MAX_SWITCH_NO + 1] = {0};
unsigned long lastOffMillis[MAX_SWITCH_NO + 1] = {0};
const unsigned long PRINT_INTERVAL = 2000UL;

// stream reconnect control
unsigned int stream_fail_count = 0;
unsigned long lastStreamReconnect = 0;
const unsigned long STREAM_RECONNECT_INTERVAL = 10UL * 1000UL; // 10s

// Schedule struct
struct Schedule {
  uint8_t onHour;
  uint8_t onMin;
  uint8_t offHour;
  uint8_t offMin;
  bool valid;
};

// Firebase objects (single FirebaseData)
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
Ticker timer;
WiFiManager wm;
Schedule schedules[MAX_SWITCH_NO + 1];

// ========== Command Queue ==========
enum CommandType { CMD_SWITCH, CMD_UPDATE, CMD_RESET, CMD_SET_TIME, CMD_ONLINE };

struct Command {
  CommandType type;
  int switch_no;
  int value;
};

const int MAX_QUEUE = 16;
Command cmdQueue[MAX_QUEUE];
int queueHead = 0;
int queueTail = 0;

void enqueueCommand(CommandType type, int sw, int val) {
  int next = (queueTail + 1) % MAX_QUEUE;
  if (next != queueHead) { // not full
    cmdQueue[queueTail].type = type;
    cmdQueue[queueTail].switch_no = sw;
    cmdQueue[queueTail].value = val;
    queueTail = next;
  } else {
    // queue full; consider logging or dropping oldest
    Serial.println(F("Command queue full, dropping command"));
  }
}

bool dequeueCommandItem(Command &cmd) {
  if (queueHead == queueTail) return false; // empty
  cmd = cmdQueue[queueHead];
  queueHead = (queueHead + 1) % MAX_QUEUE;
  return true;
}

// ========== Helpers ==========
void IRAM_ATTR handleButtonPress() {
  buttonPressed = true;
}

void Blink_led(int state, int speed) {
  for (int i = 0; i <= 3; i++) {
    digitalWrite(wifiLed, state);
    delay(speed);
    digitalWrite(wifiLed, !state);
    delay(speed);
  }
}

void send_online() {
  char path[MAX_PATH_LEN];
  snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/ONLINE_STATUS", sys_id);
  if (!Firebase.setInt(fbdo, path, 1)) {
    // try again once
    if (!Firebase.setInt(fbdo, path, 1)) {
      ESP.restart();
    }
  }
}

void settime() 
{
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

void initStream() {
  char path[MAX_PATH_LEN];
  snprintf(path, sizeof(path), "/SYSTEM/%s/BUTTONS", sys_id);
  if (!Firebase.beginStream(fbdo, path)) {
    Serial.print(F("Stream begin error: "));
    Serial.println(fbdo.errorReason());
    return;
  } else {
    digitalWrite(wifiLed, LOW);
  }
}

// ---------- notification & state functions (use fbdo single object) ----------
bool send_notification_safe(int switch_no, int state) {
  char path[MAX_PATH_LEN];
  char basePath[MAX_PATH_LEN];
  char timeBuf[48];

  snprintf(path, sizeof(path), "SYSTEM/%s/NOTIFICATION/SW%d", sys_id, switch_no);
  snprintf(basePath, sizeof(basePath), "SYSTEM/%s/BUTTONS/SW%d", sys_id, switch_no);

  // time like "HH:MM_DD/MM/YYYY"
  snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u_%02u/%02u/%04u", time_hours, time_mint, currentDay, currentMonth, currentYear);

  // set button to 3 and notification time (two writes)
  Firebase.setInt(fbdo, basePath, 3);

  char timePath[MAX_PATH_LEN];
  snprintf(timePath, sizeof(timePath), "SYSTEM/%s/NOTIFICATION/NOTIFICATION_TIME", sys_id);
  Firebase.setString(fbdo, timePath, timeBuf);

  return Firebase.setInt(fbdo, path, state);
}

bool send_switch_state_safe(int switch_no, bool state) {
  char path[MAX_PATH_LEN];
  snprintf(path, sizeof(path), "SYSTEM/%s/SWITCH_STATE/SW%d", sys_id, switch_no);
  if (send_notification_safe(switch_no, state)) {
    return Firebase.setInt(fbdo, path, (int)state);
  } else {
    return false;
  }
}

// ---------- readUpdateLink (returns into provided buffer) ----------
void readUpdateLinkFromEEPROM(char *dest, size_t maxLen) {
  for (size_t i = 0; i < maxLen - 1; ++i) {
    char c = EEPROM.read(UPDATE_LINK_ADDER + i);
    if (c == '\0') {
      dest[i] = '\0';
      return;
    }
    dest[i] = c;
  }
  dest[maxLen - 1] = '\0';
}

void readlink_from_firebase() {
  EEPROM.put(EEPROM_UPDATE, 1);
  const char *path = "ADDITIONAL_INFO/UPDATE_LINK";
  if (Firebase.getString(fbdo, path)) {
    if (fbdo.dataType() == "string") {
      String link = fbdo.stringData();
      // store to EEPROM
      size_t len = link.length();
      if (len > 240) len = 240; // avoid overflow
      for (size_t i = 0; i < len; ++i) {
        EEPROM.write(UPDATE_LINK_ADDER + i, link[i]);
      }
      EEPROM.write(UPDATE_LINK_ADDER + len, '\0');
      EEPROM.commit();

      // set last update time in firebase
      char timeBuffer[48];
      snprintf(timeBuffer, sizeof(timeBuffer), "%02u : %02u_%02u / %02u / %u", time_hours, time_mint, currentDay, currentMonth, currentYear);
      char lastUpdatePath[MAX_PATH_LEN];
      snprintf(lastUpdatePath, sizeof(lastUpdatePath), "SYSTEM/%s/LAST_UPDATE_TIME", sys_id);
      Firebase.setString(fbdo, lastUpdatePath, timeBuffer);

      char updFlag[MAX_PATH_LEN];
      snprintf(updFlag, sizeof(updFlag), "SYSTEM/%s/BUTTONS/UPDATE", sys_id);
      Firebase.setInt(fbdo, updFlag, 0);

      Firebase.endStream(fbdo);
      delay(500);
      ESP.restart();
    } else {
      Serial.println(F("Update link data type not string"));
    }
  } else {
    Serial.print(F("Failed to fetch update link: "));
    Serial.println(fbdo.errorReason());
  }
}

void checkForUpdates(const char *binUrl) {
  Serial.println(F("Checking for updates..."));
  Firebase.endStream(fbdo);
  fbdo.clear();

  WiFiClientSecure otaClient;
  otaClient.setInsecure();

  startTime = millis();
  t_httpUpdate_return result = ESPhttpUpdate.update(otaClient, binUrl);

  if (millis() - startTime >= timeout) {
    Serial.println(F("Update timeout occurred"));
  }

  switch (result) {
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

// ---------- schedules ----------
void readSchedulesFromFirebase() {
  char basePath[100], fullPath[120];
  for (int i = 1; i <= MAX_SWITCH_NO; i++) {
    snprintf(basePath, sizeof(basePath), "SYSTEM/%s/SCHEDULED/SW%d", sys_id, i);
    snprintf(fullPath, sizeof(fullPath), "%s/ON_HOUR", basePath);
    if (Firebase.getInt(fbdo, fullPath)) {
      schedules[i].onHour = fbdo.intData();
      snprintf(fullPath, sizeof(fullPath), "%s/ON_MIN", basePath);
      Firebase.getInt(fbdo, fullPath);
      schedules[i].onMin = fbdo.intData();
      snprintf(fullPath, sizeof(fullPath), "%s/OFF_HOUR", basePath);
      Firebase.getInt(fbdo, fullPath);
      schedules[i].offHour = fbdo.intData();
      snprintf(fullPath, sizeof(fullPath), "%s/OFF_MIN", basePath);
      Firebase.getInt(fbdo, fullPath);
      schedules[i].offMin = fbdo.intData();
      schedules[i].valid = true;
    } else {
      schedules[i].valid = false;
    }
  }
}

void applySchedules() {
  unsigned long now = millis();
  for (int i = 1; i <= MAX_SWITCH_NO; i++) {
    if (!schedules[i].valid) continue;

    // ON
    if (time_hours == schedules[i].onHour && time_mint == schedules[i].onMin) {
      if (on_transfer_counter[i] < 2 && now - lastOnMillis[i] >= PRINT_INTERVAL) {
        serial_out(i, 1);
        on_transfer_counter[i]++;
        char basePath[64];
        snprintf(basePath, sizeof(basePath), "SYSTEM/%s/BUTTONS/SW%d", sys_id, i);
        Firebase.setInt(fbdo, basePath, 1);
        lastOnMillis[i] = now;
      }
      digitalWrite(TRANSFER_IND_PIN, (on_transfer_counter[i] < 2) ? LOW : HIGH);
    } else {
      on_transfer_counter[i] = 0;
      lastOnMillis[i] = 0;
    }

    // OFF
    if (time_hours == schedules[i].offHour && time_mint == schedules[i].offMin) {
      if (off_transfer_counter[i] < 2 && now - lastOffMillis[i] >= PRINT_INTERVAL) {
        serial_out(i, 0);
        off_transfer_counter[i]++;
        char basePath[64];
        snprintf(basePath, sizeof(basePath), "SYSTEM/%s/BUTTONS/SW%d", sys_id, i);
        Firebase.setInt(fbdo, basePath, 0);
        lastOffMillis[i] = now;
      }
      digitalWrite(TRANSFER_IND_PIN, (off_transfer_counter[i] < 2) ? LOW : HIGH);
    } else {
      off_transfer_counter[i] = 0;
      lastOffMillis[i] = 0;
    }
  }
}

void serial_out(int switch_no, int state) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%02d", switch_no);
  char path[48];
  snprintf(path, sizeof(path), "%s_%s_%d", sys_id, buffer, state);
  Serial.println();
  Serial.println(path);
}

// ========== Config portal, EEPROM, util ==========
void setupConfigPortal() {
  Serial.print(F("Free RAM before cleanup: "));
  Serial.println(ESP.getFreeHeap());

  Firebase.endStream(fbdo);
  fbdo.clear();
  timeClient.end();
  WiFi.disconnect(true);
  delay(100);

  Serial.print(F("Free RAM after cleanup: "));
  Serial.println(ESP.getFreeHeap());

  Blink_led(0, 500);
  wm.setCaptivePortalEnable(true);
  WiFiManagerParameter sys_id_param("sys_id", "System ID", sys_id, 32);
  wm.addParameter(&sys_id_param);
  wm.setConfigPortalTimeout(180);
  delay(1000);

  if (!wm.startConfigPortal(AP_ID, AP_PASS)) {
    Serial.println(F("Failed to connect through Config Portal"));
  }
  strncpy(sys_id, sys_id_param.getValue(), sizeof(sys_id) - 1);
  sys_id[sizeof(sys_id) - 1] = '\0';
  for (int i = 0; i < sizeof(sys_id); i++) {
    EEPROM.write(SYS_ID_ADDR + i, sys_id[i]);
  }
  EEPROM.commit();
  Blink_led(0, 500);
  ESP.restart();
}

void readSysIdFromEEPROM() {
  for (int i = 0; i < sizeof(sys_id); i++) {
    sys_id[i] = EEPROM.read(SYS_ID_ADDR + i);
  }
  sys_id[sizeof(sys_id) - 1] = '\0';
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

void resetSwitchesIfPowerOn() {
  String reason = ESP.getResetReason();
  Serial.print(F("Reset Reason: "));
  Serial.println(reason);

  if (reason.indexOf("Power") != -1 || reason.indexOf("External") != -1) {
    Serial.println(F("Power-on detected. Resetting all switch states..."));
    char path[MAX_PATH_LEN];
    for (int i = 1; i <= MAX_SWITCH_NO; i++) {
      snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/SW%d", sys_id, i);
      if (!Firebase.setInt(fbdo, path, 3)) {
        Serial.print(F("Failed to reset BUTTONS for SW"));
        Serial.println(i);
      }
      snprintf(path, sizeof(path), "SYSTEM/%s/SWITCH_STATE/SW%d", sys_id, i);
      if (!Firebase.setInt(fbdo, path, 3)) {
        Serial.print(F("Failed to reset STATE for SW"));
        Serial.println(i);
      }
    }
  }
}

// ========== Stream handling (non-blocking) ==========
void handleStream(FirebaseData &fb) {
  // Attempt to read stream; if fails mark offline
  if (!Firebase.readStream(fb)) {
    online_flag = false;
    Serial.print(F("Stream read failed: "));
    Serial.println(fb.errorReason());
    return;
  }
  stream_fail_count = 0;

  if (fb.streamTimeout()) {
    Serial.println(F("Stream timeout, restarting stream..."));
    Firebase.endStream(fb);
    delay(100);
    initStream();
    return;
  }

  if (fb.streamAvailable() && fb.dataType() == "int") {
    int value = fb.intData();
    String dataPath = fb.dataPath(); // small temporary String OK here

    if (dataPath.startsWith("/SW")) 
    {
      int switch_no = dataPath.substring(3).toInt();
      if(value == 1 || value == 0)
      {
        enqueueCommand(CMD_SWITCH, switch_no, value);
      }
    } else if (dataPath.startsWith("/UPDATE")) {
      enqueueCommand(CMD_UPDATE, 0, value);
    } else if (dataPath.startsWith("/RESET")) {
      enqueueCommand(CMD_RESET, 0, value);
    } else if (dataPath.startsWith("/SET_TIME")) {
      enqueueCommand(CMD_SET_TIME, 0, value);
    } else if (dataPath.startsWith("/ONLINE_STATUS")) {
      enqueueCommand(CMD_ONLINE, 0, value);
    }
  }
}
// Add these globals at the top
struct SwitchRetry {
  bool active = false;
  int switch_no = 0;
  int value = 0;
  unsigned long lastMillis = 0;
  uint8_t count = 0;  // how many times sent
};

SwitchRetry switchRetry;

void handle_data() {
  // Process one command per call to keep loop responsive
  if (!Firebase.ready()) return;

  // Handle switch retry first
  if (switchRetry.active) {
    if (millis() - switchRetry.lastMillis >= 1000) { // 1 sec gap
      serial_out(switchRetry.switch_no, switchRetry.value);
      //send_switch_state_safe(switchRetry.switch_no, switchRetry.value);
      switchRetry.count++;
      switchRetry.lastMillis = millis();

      if (switchRetry.count >= 2) {
        switchRetry.active = false; // done sending twice
        digitalWrite(TRANSFER_IND_PIN, HIGH);
        send_online();
      }
    }
  } else {
    Command cmd;
    if (dequeueCommandItem(cmd)) {
      switch (cmd.type) {
        case CMD_SWITCH: {
          digitalWrite(TRANSFER_IND_PIN, LOW);
          serial_out(cmd.switch_no, cmd.value);
          send_switch_state_safe(cmd.switch_no, cmd.value);

          // Start retry
          switchRetry.active = true;
          switchRetry.switch_no = cmd.switch_no;
          switchRetry.value = cmd.value;
          switchRetry.lastMillis = millis();
          switchRetry.count = 1;  // already sent once
        } break;

        case CMD_UPDATE:
          if (cmd.value == 1) {
            Serial.println(F("UPDATING ....."));
            readlink_from_firebase();
          }
          break;

        case CMD_RESET:
          if (cmd.value == 1) {
            char path[MAX_PATH_LEN];
            snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/RESET", sys_id);
            Firebase.setInt(fbdo, path, 0);
            ESP.restart();
          }
          break;

        case CMD_SET_TIME:
          if (cmd.value == 1) {
            char path[MAX_PATH_LEN];
            snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/SET_TIME", sys_id);
            Firebase.setInt(fbdo, path, 0);
            readSchedulesFromFirebase();
            Blink_led(1, 100);
          }
          break;

        case CMD_ONLINE:
          send_online();
          break;
      }
    }
  }
}

// ========== Firebase setup ==========
bool setup_firebase() {
  timeClient.begin();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    delay(1000);

    Serial.println(F("sign-in successful!"));

    char path[MAX_PATH_LEN];
    // Initialize buttons and system info
    snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/UPDATE", sys_id);
    online_flag = Firebase.setInt(fbdo, path, 0);
    snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/RESET", sys_id);
    online_flag = Firebase.setInt(fbdo, path, 0);
    snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/SET_TIME", sys_id);
    Firebase.setInt(fbdo, path, 0);
    snprintf(path, sizeof(path), "SYSTEM/%s/BUTTONS/ONLINE_STATUS", sys_id);
    online_flag = Firebase.setInt(fbdo, path, 1);

    snprintf(path, sizeof(path), "SYSTEM/%s/SYSTEM_INFO/FIRMWARE_VERSION", sys_id);
    Firebase.set(fbdo, path, FIRMWARE_VERSION);
    snprintf(path, sizeof(path), "SYSTEM/%s/SYSTEM_INFO/RESET_REASON", sys_id);
    Firebase.set(fbdo, path, ESP.getResetInfo());

    initStream();
  } else {
    Serial.println(F("Firebase sign-in failed!"));
  }

  char ramPath[MAX_PATH_LEN];
  snprintf(ramPath, sizeof(ramPath), "SYSTEM/%s/SYSTEM_INFO/FREE_RAM", sys_id);
  return Firebase.set(fbdo, ramPath, String(ESP.getFreeHeap()));
}

// ========== Setup & Loop ==========
void setup() {
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, HIGH); // OFF by default
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(TRANSFER_IND_PIN, OUTPUT);
  digitalWrite(TRANSFER_IND_PIN, HIGH); // OFF by default
  Serial.begin(9600);
  delay(1000);

  EEPROM.begin(EEPROM_SIZE);
  update_ota = EEPROM.read(EEPROM_UPDATE);
  readSysIdFromEEPROM();

  wm.setConfigPortalTimeout(10);
  if (wm.autoConnect(AP_ID, AP_PASS)) {
    Serial.println(F("WiFi connected"));
    Serial.print(F("Firmware version: "));
    Serial.println(FIRMWARE_VERSION);
    Serial.print(F("system id: "));
    Serial.println(sys_id);

    if (update_ota == 1) {
      String link = readUpdateLinkFromEEPROM_String();
      EEPROM.write(EEPROM_UPDATE, 0);
      EEPROM.commit();
      checkForUpdates(link.c_str());
    } else if (EEPROM.read(EEPROM_UPDATE) != 0) {
      EEPROM.write(EEPROM_UPDATE, 0);
      EEPROM.commit();
    }

    if (setup_firebase()) {
      resetSwitchesIfPowerOn();
      digitalWrite(wifiLed, LOW);
    }
  } else {
    digitalWrite(wifiLed, HIGH);
  }

  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), handleButtonPress, CHANGE);
}

void loop() {
  if (buttonPressed) {
    delay(1000);
    if (digitalRead(TRIGGER_PIN) == LOW) {
      buttonPressed = false;
      setupConfigPortal();
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(wifiLed, HIGH);
    if (offlineStartTime == 0) offlineStartTime = millis();
    if (millis() - offlineStartTime >= OFFLINE_TIMEOUT) {
      ESP.restart();
    }
  } else {
    if (!online_flag) {
      setup_firebase();
    }
    digitalWrite(wifiLed, LOW);
    offlineStartTime = 0;

    handleStream(fbdo);   // non-blocking: only enqueues commands
    settime();
    handle_data();        // processes one queued command
    applySchedules();

   // Serial.print(F("FreeHeap: "));
    //Serial.println(ESP.getFreeHeap());
  }
}
