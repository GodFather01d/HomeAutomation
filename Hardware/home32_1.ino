

#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#define FIRMWARE_VERSION 3.0

#include <WiFiManager.h>    
#include <FirebaseClient.h>
#include "ExampleFunctions.h"
#include <EEPROM.h>  
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h> 
#include <FirebaseJson.h> 
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>   
#include <esp_system.h>      

// ========== Firebase Configuration ==========
#define API_KEY "AIzaSyAQh-tAWmKpcGzwmXISv0-aY-Q4s6MAxZc"
#define DATABASE_URL "https://homeautomation-e6ef2-default-rtdb.asia-southeast1.firebasedatabase.app"
#define AP_ID "JT_WIFI_SETUP"
#define AP_PASS "Jyoti@2000"

// Pin Definitions
#define wifiLed 2           // D4: WiFi status LED
#define TRIGGER_PIN 13      // D5: Pin to trigger WiFiManager AP mode
#define TRANSFER_IND_PIN 12 // D6: Data transfer indication LED


// ========== EEPROM Layout (bytes) ==========
#define EEPROM_SIZE        200
#define ADDR_SIG           0      // 1 byte signature
#define ADDR_SYSID         1      // 20 bytes
#define SYSID_LEN          20
#define ADDR_EMAIL         (ADDR_SYSID + SYSID_LEN)   // 21..70 (50 bytes)
#define EMAIL_LEN          50
#define ADDR_PASS          (ADDR_EMAIL + EMAIL_LEN)   // 71..120 (50 bytes)
#define PASS_LEN           50
#define EEPROM_SIG         0xAA


#define MAX_SWITCH_NO 25

struct Schedule {
  bool valid;
  int onHour, onMin, offHour, offMin;
};

Schedule schedules[MAX_SWITCH_NO + 1]; 
// ========== Global Variables ==========
char sys_id[SYSID_LEN + 1] = "DEFAULT_SYS";
char user_email[EMAIL_LEN + 1] = "";
char user_pass[PASS_LEN + 1] = "";
const long utcOffsetInSeconds = 19800;
const unsigned long UPDATE_TIMEOUT = 20000;
uint8_t time_hours = 0, time_mint = 0, currentMonth = 0, currentDay = 0;
uint16_t currentYear = 0;
unsigned char readschedules_flag = 0,perform_ota_flag = 0;
String otaURL = "";

unsigned long lastWiFiCheck = 0;
unsigned long wifiDisconnectedSince = 0;
unsigned long firebaseAuthFailSince = 0;
bool wifiWasConnected = true;
bool firebaseWasAuth = true;

// Prevent duplicate prints for same time
int lastTriggeredHour = -1;
int lastTriggeredMinute = -1;
bool triggeredState[MAX_SWITCH_NO + 1][2];  
// [i][0] for OFF, [i][1] for ON

SSL_CLIENT ssl_client, stream_ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client), streamClient(stream_ssl_client);

UserAuth *user_auth = nullptr;
FirebaseApp app;
RealtimeDatabase Database;
AsyncResult streamResult;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);



void processData(AsyncResult &aResult);



// ----------------- EEPROM helpers -----------------
void writeStringToEEPROM(int addr, int maxLen, const char *str) {
  int i;
  for (i = 0; i < maxLen; ++i) {
    if (i < (int)strlen(str)) {
      EEPROM.write(addr + i, (uint8_t)str[i]);
    } else {
      EEPROM.write(addr + i, 0); // pad with NUL
    }
  }
}

void readStringFromEEPROM(int addr, int maxLen, char *outBuf) {
  for (int i = 0; i < maxLen; ++i) {
    uint8_t b = EEPROM.read(addr + i);
    outBuf[i] = (char)b;
  }
  outBuf[maxLen] = '\0'; // ensure null-terminated
  // Trim any tail bytes after first NUL
  for (int i = 0; i < maxLen; ++i) {
    if (outBuf[i] == '\0') {
      outBuf[i] = '\0';
      break;
    }
  }
}

// Save sys_id, email, pass to EEPROM and commit
void saveCredentialsToEEPROM() {
  Serial.println("Saving credentials to EEPROM...");
  EEPROM.write(ADDR_SIG, EEPROM_SIG);
  writeStringToEEPROM(ADDR_SYSID, SYSID_LEN, sys_id);
  writeStringToEEPROM(ADDR_EMAIL, EMAIL_LEN, user_email);
  writeStringToEEPROM(ADDR_PASS, PASS_LEN, user_pass);
  if (EEPROM.commit()) {
    Serial.println("Credentials saved to EEPROM successfully.");
  } else {
    Serial.println("Failed to commit EEPROM!");
  }
}

// Load credentials from EEPROM, return true if signature valid
bool loadCredentialsFromEEPROM() {
  uint8_t sig = EEPROM.read(ADDR_SIG);
  if (sig != EEPROM_SIG) {
    Serial.println("No valid EEPROM signature found. Using defaults.");
    return false;
  }

  readStringFromEEPROM(ADDR_SYSID, SYSID_LEN, sys_id);
  readStringFromEEPROM(ADDR_EMAIL, EMAIL_LEN, user_email);
  readStringFromEEPROM(ADDR_PASS, PASS_LEN, user_pass);

  Serial.println("Credentials loaded from EEPROM:");
  Serial.print(" sys_id: "); Serial.println(sys_id);
  Serial.print(" email : "); Serial.println(user_email);
  // Do not print password in production logs, but show masked for debug:
  Serial.print(" pass  : "); 
  for (int i=0; i< (int)strlen(user_pass); ++i) Serial.print('*');
  Serial.println();
  return true;
}

// Optional: clear EEPROM credentials (factory reset)
void clearCredentialsInEEPROM() {
  Serial.println("Clearing credentials in EEPROM...");
  for (int i = 0; i < EEPROM_SIZE; ++i) EEPROM.write(i, 0);
  EEPROM.commit();
  Serial.println("EEPROM cleared.");
}

// ===== Function to get reset reason =====
String getResetReasonText(esp_reset_reason_t reason)
{
  switch (reason)
  {
    case ESP_RST_POWERON:   return "Power-on Reset";
    case ESP_RST_EXT:       return "External Reset (Reset pin)";
    case ESP_RST_SW:        return "Software Reset";
    case ESP_RST_PANIC:     return "Exception/Panic Reset";
    case ESP_RST_INT_WDT:   return "Interrupt Watchdog Reset";
    case ESP_RST_TASK_WDT:  return "Task Watchdog Reset";
    case ESP_RST_WDT:       return "Other Watchdog Reset";
    case ESP_RST_DEEPSLEEP: return "Wake from Deep Sleep";
    case ESP_RST_BROWNOUT:  return "Brownout Reset";
    case ESP_RST_SDIO:      return "SDIO Reset";
    default:                return "Unknown Reset";
  }
}

void uploadESPInfo()
{
  String basePath = "/SYSTEM/" + String(sys_id) + "/SYSTEM_INFO";

  // ===== Basic Chip Info =====
  String chipModel = String(ESP.getChipModel());
  String chipRev = String(ESP.getChipRevision());
  String cpuFreq = String(ESP.getCpuFreqMHz()) + " MHz";
  String flashSize = String(ESP.getFlashChipSize() / 1024 / 1024) + " MB";
  String sdkVer = String(ESP.getSdkVersion());
  String reasonText = getResetReasonText(esp_reset_reason());

  // ===== Network Info =====
  String ip = WiFi.localIP().toString();
  String mac = WiFi.macAddress();
  String ssid = WiFi.SSID();
  int rssi = WiFi.RSSI();

  // ===== Memory and System Stats =====
  size_t freeHeap = ESP.getFreeHeap();
  size_t minHeap = ESP.getMinFreeHeap();
  size_t sketchSize = ESP.getSketchSize();
  size_t freeSketchSpace = ESP.getFreeSketchSpace();
  uint32_t flashChipSpeed = ESP.getFlashChipSpeed() / 1000000; // MHz


  String buildDate = String(__DATE__) + " " + String(__TIME__);

  // ===== Upload All to Firebase =====
  Database.set<String>(aClient, basePath + "/Chip_Model", chipModel);
  Database.set<String>(aClient, basePath + "/Chip_Revision", chipRev);
  Database.set<String>(aClient, basePath + "/CPU_Frequency", cpuFreq);
  Database.set<String>(aClient, basePath + "/Flash_Size", flashSize);
  Database.set<String>(aClient, basePath + "/Flash_Speed", String(flashChipSpeed) + " MHz");
  Database.set<String>(aClient, basePath + "/SDK_Version", sdkVer);
  Database.set<String>(aClient, basePath + "/IP_Address", ip);
  Database.set<String>(aClient, basePath + "/MAC_Address", mac);
  Database.set<String>(aClient, basePath + "/SSID", ssid);
  Database.set<int>(aClient, basePath + "/RSSI_dBm", rssi);
  Database.set<String>(aClient, basePath + "/Reset_Reason", reasonText);
  Database.set<int>(aClient, basePath + "/Free_Heap", freeHeap);
  Database.set<int>(aClient, basePath + "/Min_Free_Heap", minHeap);
  Database.set<int>(aClient, basePath + "/Sketch_Size", sketchSize);
  Database.set<int>(aClient, basePath + "/Free_Sketch_Space", freeSketchSpace);
  Database.set<int>(aClient, basePath + "/FIRMWARE_VERSION", FIRMWARE_VERSION);
  Database.set<String>(aClient, basePath + "/Build_Date", buildDate);
  Database.set<String>(aClient, basePath + "/Uptime_Start", String(millis() / 1000) + " sec");

  Serial.println("📤 Uploaded full ESP info to Firebase:");
  Serial.printf("Heap: %d bytes | MinHeap: %d bytes | RSSI: %d dBm\n", freeHeap, minHeap, rssi);
}




// ===================================================
// ========== WiFiManager Setup Function =============
// ===================================================
void openPortalAndSaveCredentials() {
  WiFiManager wm;

  // Load existing credentials from EEPROM to pre-populate portal fields (if present)
  loadCredentialsFromEEPROM();

  // Custom parameters for system ID, email, password
  WiFiManagerParameter sysIdParam("sysid", "System ID", sys_id, SYSID_LEN);
  WiFiManagerParameter emailParam("email", "Firebase Email", user_email, EMAIL_LEN);
  WiFiManagerParameter passParam("pass", "Firebase Password", user_pass, PASS_LEN);

  wm.addParameter(&sysIdParam);
  wm.addParameter(&emailParam);
  wm.addParameter(&passParam);

  wm.setConfigPortalTimeout(180); // 3-minute portal timeout

  Serial.println("Starting configuration portal...");

  // Blocking portal: waits until connected or times out
  if (!wm.startConfigPortal(AP_ID, AP_PASS)) {
    Serial.println("⚠️ Portal timed out — restarting...");
    delay(2000);
    ESP.restart();
  }

  // If we get here, the portal was successful and we are connected to WiFi.
  // Get the new values entered by the user.
  strncpy(sys_id, sysIdParam.getValue(), SYSID_LEN);
  sys_id[SYSID_LEN] = '\0';
  strncpy(user_email, emailParam.getValue(), EMAIL_LEN);
  user_email[EMAIL_LEN] = '\0';
  strncpy(user_pass, passParam.getValue(), PASS_LEN);
  user_pass[PASS_LEN] = '\0';

  // Save the new credentials to EEPROM.
  saveCredentialsToEEPROM();

  Serial.println("✅ Portal finished. Wi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
}



// Function to get ESP32 reset reason as readable string
esp_reset_reason_t getResetReason() {
  return esp_reset_reason();
}
// ===================================================
// ========== WiFi connection + load =================
// ===================================================
bool connectWiFiAndLoadCredentials() {
  // Initialize EEPROM and try to load credentials from it
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM begin failed!");
  } else {
    Serial.println("EEPROM initialized.");
    loadCredentialsFromEEPROM(); // attempt load (if signature exists)
  }

  Serial.println("Trying Wi-Fi connection for 10 seconds...");

  WiFi.begin(); // ESP32 automatically uses stored WiFi credentials

  unsigned long startAttemptTime = millis();
  bool wifiConnected = false;

  while (millis() - startAttemptTime < 10000) { // 10 seconds
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      break;
    }
    delay(200);
  }

  if (wifiConnected) {
    Serial.println("✅ Wi-Fi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("❌ Wi-Fi not connected in 10 sec (Skipping WiFiManager Auto Portal)");
    return false;
  }
}


// ===================================================
// ========== FIREBASE SETUP =========================
// ===================================================
void firebase_setup()
{
  Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);

  set_ssl_client_insecure_and_buffer(ssl_client);
  set_ssl_client_insecure_and_buffer(stream_ssl_client);

  Serial.println("Initializing Firebase...");
  user_auth = new UserAuth(API_KEY, user_email, user_pass, 3000);

  initializeApp(aClient, app, getAuth(*user_auth), auth_debug_print, "🔐 authTask");

  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  Serial.println("Waiting for Firebase authentication (Max 60 sec)...");

  unsigned long startTime = millis();
  bool authSuccess = false;

  while (millis() - startTime < 60000) // 60 sec timeout
  {
    app.loop();
    if (app.isAuthenticated())
    {
      authSuccess = true;
      break;
    }
    delay(100);
  }

  if (!authSuccess) {
    Serial.println("❌ Firebase authentication FAILED after 60 seconds!");
    return; // Exit without proceeding
  }

  Serial.println("✅ Authenticated Successfully!");

  // ===== Clear previous nodes =====

    // ===== Check Reset Reason =====
  esp_reset_reason_t reason = getResetReason();
  Serial.print("ESP Reset Reason: ");
  Serial.println((int)reason);

  // Only reset SWITCH_STATE if Power-On or External Reset
  if (reason == ESP_RST_POWERON || reason == ESP_RST_EXT) {
    Serial.println("🔁 Power-on or External Reset detected. Resetting SWITCH_STATE...");
    Database.remove(aClient, "/SYSTEM/" + String(sys_id) + "/SWITCH_STATE");
    Serial.println("SWITCH_STATE reset ✅");
  } else {
    Serial.println("No SWITCH_STATE reset required.");
  }
  Database.remove(aClient, "/SYSTEM/" + String(sys_id) + "/BUTTONS");
  Serial.println("Old nodes cleared ✅");

  // ===== Start streaming BUTTONS path =====
  Database.set<int>(aClient, "/SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1);
  streamClient.setSSEFilters("put,patch,keep-alive");
  Database.get(streamClient, "/SYSTEM/" + String(sys_id) + "/BUTTONS", processData, true, "streamTask");
  uploadESPInfo();
  Serial.println("📡 Firebase stream started...");
}


// ===================================================
// ========== STREAM HANDLER =========================
// ===================================================
void processData(AsyncResult &aResult)
{
  if (!aResult.isResult()) return;

  if (aResult.available())
  {
    RealtimeDatabaseResult &stream = aResult.to<RealtimeDatabaseResult>();
    if (stream.isStream())
    {
      String path = stream.dataPath();
      String data = stream.to<String>();

      if (path.startsWith("/SW"))
      {
        int swValue = data.toInt();
        String swName = path.substring(1); // remove '/'

        if (swValue == 1 || swValue == 0) {
          int switchNo = path.substring(3).toInt(); // extract number from /SWx
          printSystemEvent(sys_id, switchNo, swValue);  // print twice in format

          Database.set<int>(aClient, "/SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1);

          // reset path after action
          String resetpath = "/SYSTEM/" + String(sys_id) + "/BUTTONS/" + swName;
          Database.set<int>(aClient, resetpath, 3);
        }

      }
      else if (path.startsWith("/ONLINE"))
      {
        Database.set<int>(aClient, "/SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1);
      }
      else if (path.startsWith("/SET_TIME"))
      {
       int Value = data.toInt();
       if(Value == 1)
       {
        readschedules_flag = 1;
       }
       else
       {
        readschedules_flag = 0;
       }
       Database.set<int>(aClient, "/SYSTEM/" + String(sys_id) + "/BUTTONS/SET_TIME", 0);
      }
      else if (path.startsWith("/UPDATE"))
      {
          int Value = data.toInt();
          if (Value == 1)
          {
              // Reset the UPDATE flag immediately
            Database.set<int>(aClient, "/SYSTEM/" + String(sys_id) + "/BUTTONS/UPDATE", 0);

            // Read the OTA URL from Firebase
            otaURL = Database.get<String>(aClient, "/ADDITIONAL_INFO/UPDATE_LINK32");
            Serial.println("📡 OTA URL fetched: " + otaURL);
            
            // Serial.println("📡 OTA URL fetched: " + otaURL);

            // Save last update time
            String updatetime = String(time_hours) + ":" + String(time_mint) + " " +
                                String(currentDay) + "/" + String(currentMonth) + "/" + String(currentYear);
            Database.set<String>(aClient, "/SYSTEM/" + String(sys_id) + "/LAST_UPDATE_TIME", updatetime);
            perform_ota_flag = 1;
            // Perform OTA
            //perform_update(otaURL);
            
             
          }
          else
          {
            perform_ota_flag = 0;
          }
      }
    }
  }
}
void set_time() {
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
// 1–25 (ignore index 0)

void checkCachedSchedule() {
    // Update time only once here or call set_time() outside
    for (int i = 1; i <= MAX_SWITCH_NO; i++) {
        if (!schedules[i].valid) continue;

        // ----- ON Trigger -----
        if (time_hours == schedules[i].onHour && time_mint == schedules[i].onMin) {
            if (!triggeredState[i][1]) {   // not triggered yet
                printSystemEvent(sys_id, i, 1);
                triggeredState[i][1] = true;   // mark as triggered
                triggeredState[i][0] = false;  // reset opposite
            }
        }
        // ----- OFF Trigger -----
        else if (time_hours == schedules[i].offHour && time_mint == schedules[i].offMin) {
            if (!triggeredState[i][0]) {
                printSystemEvent(sys_id, i, 0);
                triggeredState[i][0] = true;
                triggeredState[i][1] = false;
            }
        }
        // ----- If minute moved on, reset -----
        else if (time_mint != lastTriggeredMinute || time_hours != lastTriggeredHour) {
            triggeredState[i][0] = false;
            triggeredState[i][1] = false;
        }
    }

    lastTriggeredHour = time_hours;
    lastTriggeredMinute = time_mint;
}
void readSchedulesFromFirebase() {
    if (!app.isAuthenticated()) {
        Serial.println("❌ Firebase not authenticated. Skipping schedule load.");
        return;
    }

    String path = "/SYSTEM/" + String(sys_id) + "/SCHEDULED";
    Serial.println("📥 Requesting schedules JSON...");

    AsyncResult result;
    Database.get(aClient, path.c_str(), result);  // ✅ Non-blocking async request

    // ✅ Wait until Firebase returns data (available())
    unsigned long t = millis();
    while (millis() - t < 3000) {  // max wait 3 sec
        app.loop();
        if (result.available()) break;
    }

    if (!result.available()) {
        Serial.println("⚠️ No data received (timeout or no node found)");
        return;
    }

    if (result.error().code() == 0) {  
        // ✅ Extract data correctly
        RealtimeDatabaseResult &rdb = result.to<RealtimeDatabaseResult>();
        String jsonStr = rdb.to<String>();  // ✅ Now valid
        Serial.println("📦 JSON Received:");
        Serial.println(jsonStr);

        FirebaseJson json;
        json.setJsonData(jsonStr);
        FirebaseJsonData data;

        for (int i = 1; i <= MAX_SWITCH_NO; i++) {
            String sw_key = "SW" + String(i);

            if (json.get(data, sw_key)) {
                schedules[i].valid = true;

                FirebaseJson swObj;
                swObj.setJsonData(data.to<String>());

                swObj.get(data, "ON_HR");
                if (data.success) schedules[i].onHour = data.to<int>();

                swObj.get(data, "ON_MIN");
                if (data.success) schedules[i].onMin = data.to<int>();

                swObj.get(data, "OFF_HR");
                if (data.success) schedules[i].offHour = data.to<int>();

                swObj.get(data, "OFF_MIN");
                if (data.success) schedules[i].offMin = data.to<int>();
            } else {
                schedules[i].valid = false;
            }
        }

        Serial.println("✅ Schedules loaded and cached.");
    } else {
        Serial.print("❌ Firebase Error: ");
        Serial.println(result.error().message());
    }
}

void printAllSchedules() {
  Serial.printf("⏱ Current Time: %02d:%02d\n", time_hours, time_mint);
    Serial.println(F("📋 All Switch Schedules:"));
    for (int i = 1; i <= MAX_SWITCH_NO; i++) {
        if (schedules[i].valid) {
            Serial.printf("SW%d -> ON: %02d:%02d | OFF: %02d:%02d\n", 
                          i, 
                          schedules[i].onHour, 
                          schedules[i].onMin, 
                          schedules[i].offHour, 
                          schedules[i].offMin);
        } else {
            Serial.printf("SW%d -> No schedule set\n", i);
        }
    }
}
void printSystemEvent(const char* sysId, int switchNo, int state) {
  String statePath = "/SYSTEM/" + String(sys_id) + "/SWITCH_STATE/SW" + String(switchNo);
  Database.set<int>(aClient, statePath, state);
  String NOTIPath = "/SYSTEM/" + String(sys_id) + "/NOTIFICATION/SW" + String(switchNo);
  Database.set<int>(aClient, NOTIPath, state);
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "%s_%02d_%d", sysId, switchNo, state);
  Serial.println();
  Serial.println(buffer);
  delay(1000);
  Serial.println(buffer);  // print twice

}
void perform_update() {
  WiFiClientSecure client;
  client.setInsecure();  // Accept all certificates (unsafe but works for ESP32 OTA)
  Serial.println("Updating...");
  t_httpUpdate_return ret = httpUpdate.update(client, otaURL);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("❌ OTA Failed: %d - %s\n", httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("ℹ️ No updates available");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("✅ OTA Success!");
      break;
  }
}
void check_reconnect() {
  unsigned long now = millis();

  // --- Wi-Fi Check every 5 seconds ---
  if (now - lastWiFiCheck > 5000) {
    lastWiFiCheck = now;

    if (WiFi.status() != WL_CONNECTED) {
      if (wifiWasConnected) {
        wifiWasConnected = false;
        wifiDisconnectedSince = now;
        Serial.println("⚠️ Wi-Fi lost, starting reconnect attempts...");
      }

      // Try reconnecting for up to 1 minute
      if (now - wifiDisconnectedSince < 60000) {
        WiFi.reconnect();
      }
      // Restart if Wi-Fi down for 3 minutes
      else if (now - wifiDisconnectedSince > 180000) {
        Serial.println("❌ Wi-Fi not reconnected for 3 minutes → Restarting...");
        delay(1000);
        ESP.restart();
      }
    } else {
      if (!wifiWasConnected) {
        Serial.println("✅ Wi-Fi reconnected.");
        wifiWasConnected = true;
        wifiDisconnectedSince = 0;
      }
    }
  }

  // --- Firebase Auth Check every 10 seconds ---
  if (wifiWasConnected) {
    if (!app.isAuthenticated()) {
      if (firebaseWasAuth) {
        firebaseWasAuth = false;
        firebaseAuthFailSince = now;
        Serial.println("⚠️ Firebase auth lost, trying to reauthenticate...");
      }

      // Try reauth for 1 min
      if (now - firebaseAuthFailSince < 60000) {
        app.loop();
        // if it succeeds:
        if (app.isAuthenticated()) {
          firebaseWasAuth = true;
          firebaseAuthFailSince = 0;
          Serial.println("✅ Firebase re-authenticated.");
        }
      } else {
        Serial.println("❌ Firebase auth failed for 1 minute → Restarting...");
        delay(1000);
       // ESP.restart();
      }
    } else {
      firebaseWasAuth = true;
      firebaseAuthFailSince = 0;
    }
  }
}
void firebaseTask(void *parameter) {
  for (;;) {
    app.loop();        // keep Firebase alive
    vTaskDelay(pdMS_TO_TICKS(10));   // every 100ms
  }
}
// ===================================================
// ========== MAIN SETUP & LOOP ======================
// ===================================================
void setup()
{
  Serial.begin(115200);
  delay(100);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  // Initialize EEPROM early
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM.begin() failed!");
  } else {
    Serial.println("EEPROM ready");
  }

  if(connectWiFiAndLoadCredentials())
  {
     timeClient.begin();
     firebase_setup();  
     readSchedulesFromFirebase();
  } // may open WiFiManager portal if needed
  xTaskCreatePinnedToCore(firebaseTask, "FirebaseTask", 8192, NULL, 2, NULL, 1);
  
}

void loop()
{
 // app.loop();
  set_time(); // Maintain Firebase connection
  checkCachedSchedule();
  check_reconnect();
  //printAllSchedules();
  //delay(1000);
  if (digitalRead(TRIGGER_PIN) == 0)
  {
    delay(500); // debouce & long-press style
    if (digitalRead(TRIGGER_PIN) == 0)
    {
      // Open portal and save to EEPROM (instead of LittleFS)
      openPortalAndSaveCredentials();
      // After portal, re-init firebase with new credentials (simple approach: restart)
      Serial.println("Restarting to apply new credentials...");
      delay(500);
      ESP.restart();
    }
  }
  if(perform_ota_flag == 1)
  {
    perform_ota_flag = 0;
    perform_update();
  }
  if(readschedules_flag == 1)
  {
    readschedules_flag = 0;
    readSchedulesFromFirebase();
  }

}
